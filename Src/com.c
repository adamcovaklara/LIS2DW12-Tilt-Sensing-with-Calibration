/**
 ******************************************************************************
 * @file    com.c
 * @author  MEMS Software Solutions Team
 * @brief   This file provides communication functions
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
#include "com.h"

/** @addtogroup MOTION_APPLICATIONS MOTION APPLICATIONS
 * @{
 */

/** @addtogroup TILT_SENSING TILT SENSING
 * @{
 */

/* Private types -------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define Uart_Msg_Max_Size TMsg_MaxLen

/* Private macro -------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
volatile uint8_t UartRxBuffer[UART_RxBufferSize];
volatile uint32_t UsartBaudRate = 921600;

extern UART_HandleTypeDef UartHandle; /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
UART_HandleTypeDef UartHandle;
TUart_Engine UartEngine;

/* Private variables ---------------------------------------------------------*/
static DMA_HandleTypeDef HdmaRx;
static volatile uint8_t UartTxBuffer[TMsg_MaxLen * 2];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Check if a message is received via UART
 * @param  Msg the pointer to the message to be received
 * @retval 1 if a complete message is found, 0 otherwise
 */
int UART_ReceivedMSG(TMsg *Msg)
{
  uint16_t i, j, k, j2;
  uint16_t dma_counter, length;
  uint8_t data;
  uint16_t source = 0;
  uint8_t inc;

  if (Get_DMA_Flag_Status(&HdmaRx) == (uint32_t)RESET)
  {
    dma_counter = (uint16_t)UART_RxBufferSize - (uint16_t)Get_DMA_Counter(&HdmaRx);

    if (dma_counter >= UartEngine.StartOfMsg)
    {
      length = dma_counter - UartEngine.StartOfMsg;
    }
    else
    {
      length = (uint16_t)UART_RxBufferSize + dma_counter - UartEngine.StartOfMsg;
    }

    j = UartEngine.StartOfMsg;

    for (k = 0; k < length; k++)
    {
      data = UartRxBuffer[j];
      j++;
      if (j >= (uint16_t)UART_RxBufferSize)
      {
        j = 0;
      }

      if (data == (uint8_t)TMsg_EOF)
      {
        j = UartEngine.StartOfMsg;
        for (i = 0; i < k; i += inc)
        {
          uint8_t  Source0;
          uint8_t  Source1;
          uint8_t *Dest;

          j2 = (j + 1U) % (uint16_t)UART_RxBufferSize;

          Source0 = UartRxBuffer[j];
          Source1 = UartRxBuffer[j2];
          Dest    = &Msg->Data[source];

          inc = (uint8_t)ReverseByteStuffCopyByte2(Source0, Source1, Dest);

          if (inc == 0U)
          {
            UartEngine.StartOfMsg = j2;
            return 0;
          }

          j = (j + inc) % (uint16_t)UART_RxBufferSize;
          source++;
        }

        Msg->Len = source;
        j = (j + 1U) % (uint16_t)UART_RxBufferSize; /* skip TMsg_EOF */
        UartEngine.StartOfMsg = j;

        if (CHK_CheckAndRemove(Msg) != 0) /* check message integrity */
        {
          return 1;
        }
      }
    }

    if (length > (uint16_t)Uart_Msg_Max_Size)
    {
      UartEngine.StartOfMsg = dma_counter;
    }
  }

  return 0;
}

/**
 * @brief  Send a message via UART
 * @param  Msg the pointer to the message to be sent
 * @retval None
 */
void UART_SendMsg(TMsg *Msg)
{
  uint16_t count_out;

  CHK_ComputeAndAdd(Msg);

  /* MISRA C-2012 rule 11.8 violation for purpose */
  count_out = (uint16_t)ByteStuffCopy((uint8_t *)UartTxBuffer, Msg);

  /* MISRA C-2012 rule 11.8 violation for purpose */
  (void)HAL_UART_Transmit(&UartHandle, (uint8_t *)UartTxBuffer, count_out, 5000);
}

/**
 * @brief  Configure DMA for the reception via USART
 * @param  None
 * @retval None
 */
void USART_DMA_Configuration(void)
{
  /* Configure the DMA handler for Transmission process */
  Config_DMA_Handler(&HdmaRx);
  (void)HAL_DMA_Init(&HdmaRx);

  /* Associate the initialized DMA handle to the the UART handle */
  __HAL_LINKDMA(&UartHandle, hdmarx, HdmaRx);
}

/**
 * @brief  Configure the USART
 * @param  None
 * @retval None
 */
void USARTConfig(void)
{
  GPIO_InitTypeDef gpio_init_struct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();

  /* Enable USART2 clock */
  USARTx_CLK_ENABLE();

  /* Enable DMA1 clock */
  DMAx_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* UART TX GPIO pin configuration  */
  gpio_init_struct.Pin       = USARTx_TX_PIN;
  gpio_init_struct.Mode      = GPIO_MODE_AF_PP;
  gpio_init_struct.Pull      = GPIO_NOPULL;
  gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio_init_struct.Alternate = USARTx_TX_AF;

  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &gpio_init_struct);

  /* UART RX GPIO pin configuration */
  gpio_init_struct.Pin = USARTx_RX_PIN;
  gpio_init_struct.Alternate = USARTx_RX_AF;

  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &gpio_init_struct);

  /*##-3- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  UartHandle.Instance        = USARTx;
  UartHandle.Init.BaudRate   = UsartBaudRate;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;

  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    for (;;)
    {}
  }

  USART_DMA_Configuration();

  UartHandle.pRxBuffPtr = (uint8_t *)UartRxBuffer; /* MISRA C-2012 rule 11.8 violation for purpose */
  UartHandle.RxXferSize = UART_RxBufferSize;
  UartHandle.ErrorCode = (uint32_t)HAL_UART_ERROR_NONE;

  /* Enable the DMA transfer for the receiver request by setting the DMAR bit
  in the UART CR3 register */
  /* MISRA C-2012 rule 11.8 violation for purpose */
  (void)HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)UartRxBuffer, UART_RxBufferSize);
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
