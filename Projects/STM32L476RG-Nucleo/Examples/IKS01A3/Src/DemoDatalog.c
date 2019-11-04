/**
 ******************************************************************************
 * @file    DemoDatalog.c
 * @author  MEMS Software Solutions Team
 * @brief   Handle the data logging to flash memory
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
#include "main.h"
#include "DemoDatalog.h"

/** @addtogroup MOTION_APPLICATIONS MOTION APPLICATIONS
 * @{
 */

/** @addtogroup TILT_SENSING TILT SENSING
 * @{
 */

/* Private defines -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#if (defined (USE_STM32L4XX_NUCLEO))
static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);
#endif

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Save the calibration values to memory
 * @param  None
 * @retval None
 */
void SaveCalibrationToMemory(uint16_t DataSize, uint32_t *Data)
{
  uint32_t address = FLASH_ADDRESS;
  uint32_t nword;

  /* Reset Before The data in Memory */
  ResetCalibrationInMemory();

  /* Unlock the Flash to enable the flash control register access */
  (void)HAL_FLASH_Unlock();

  /* Clear pending flags (if any) */
#if (defined (USE_STM32F4XX_NUCLEO))
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                         FLASH_FLAG_PGSERR);

#elif (defined (USE_STM32L0XX_NUCLEO))
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_RDERR |
                         FLASH_FLAG_WRPERR | FLASH_FLAG_FWWERR | FLASH_FLAG_NOTZEROERR);

#elif (defined (USE_STM32L1XX_NUCLEO))
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR |
                         FLASH_FLAG_OPTVERRUSR | FLASH_FLAG_WRPERR);

#elif (defined (USE_STM32L4XX_NUCLEO))
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

#else
#error Not supported platform
#endif

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
  uint32_t word;

  for (nword = 0; nword < DataSize; nword++)
  {
    word = *(Data + nword);

    if (HAL_FLASH_Program(TYPEPROGRAM_WORD, address, word) == HAL_OK)
    {
      address += 4U;
    }
    else
    {
      /* Error occurred while writing data in Flash memory */
      Error_Handler();
    }
  }

#elif (defined (USE_STM32L4XX_NUCLEO))
  uint64_t lword, hword;

  for (nword = 0; nword < DataSize; nword += 2U)
  {
    lword = *(Data + nword); /* MISRA C-2012 rule 18.4 violation for purpose */

    if ((nword + 1U) < DataSize)
    {
      hword = (uint64_t)(*(Data + nword + 1)) << 32; /* MISRA C-2012 rule 18.4 violation for purpose */
    }
    else
    {
      hword = 0;
    }

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, hword | lword) == HAL_OK)
    {
      address += 8U;
    }
    else
    {
      /* Error occurred while writing data in Flash memory */
      Error_Handler();
    }
  }

#else
#error Not supported platform
#endif

  /* Lock the Flash to disable the flash control register access */
  (void)HAL_FLASH_Lock();
}

/**
 * @brief  Check if there are valid calibration values in memory and read them
 * @param  None
 * @retval None
 */
void RecallCalibrationFromMemory(uint16_t DataSize, uint32_t *Data)
{
  uint32_t address = FLASH_ADDRESS;

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
  while (address < (FLASH_ADDRESS + DataSize * 4))
  {
    *(Data) = *(__IO uint32_t *)address;
    Data += 1;
    address += 4;
  }

#elif (defined (USE_STM32L4XX_NUCLEO))
  uint32_t *dataMem = Data;
  uint64_t dword;

  while (address < (FLASH_ADDRESS + (uint32_t)DataSize * 4U))
  {
    dword = (*(__IO uint64_t *)address);
    *(Data) = (uint32_t)(dword & 0x00000000FFFFFFFFU);
    Data += 1; /* MISRA C-2012 rule 18.4 violation for purpose */

    if (Data < (dataMem + DataSize)) /* MISRA C-2012 rule 18.4 violation for purpose */
    {
      *(Data) = (uint32_t)((dword & 0xFFFFFFFF00000000U) >> 32);
      Data += 1; /* MISRA C-2012 rule 18.4 violation for purpose */
    }

    address += 8U;
  }

#else
#error Not supported platform
#endif
}

/**
 * @brief  Reset the calibration values in memory
 * @param  None
 * @retval None
 */
void ResetCalibrationInMemory(void)
{
  FLASH_EraseInitTypeDef erase_init_struct;
  uint32_t sector_error = 0;

#if (defined (USE_STM32F4XX_NUCLEO))
  erase_init_struct.TypeErase    = TYPEERASE_SECTORS;
  erase_init_struct.VoltageRange = VOLTAGE_RANGE_3;
  erase_init_struct.Sector       = FLASH_SECTOR;
  erase_init_struct.NbSectors    = 1;

#elif (defined (USE_STM32L0XX_NUCLEO))
  erase_init_struct.TypeErase   = FLASH_TYPEERASE_PAGES;
  erase_init_struct.PageAddress = FLASH_ADDRESS;
  erase_init_struct.NbPages     = 32; /* page 480 .. 511 */

#elif (defined (USE_STM32L1XX_NUCLEO))
  erase_init_struct.TypeErase   = FLASH_TYPEERASE_PAGES;
  erase_init_struct.PageAddress = FLASH_ADDRESS;
  erase_init_struct.NbPages     = 512; /* page 1024 .. 1535 */

#elif (defined (USE_STM32L4XX_NUCLEO))
  erase_init_struct.TypeErase   = FLASH_TYPEERASE_PAGES;
  erase_init_struct.Banks       = GetBank(FLASH_ADDRESS);
  erase_init_struct.Page        = GetPage(FLASH_ADDRESS);
  erase_init_struct.NbPages     = 1;

#else
#error Not supported platform
#endif

  /* Unlock the Flash to enable the flash control register access */
  (void)HAL_FLASH_Unlock();

  /* Clear pending flags (if any) */
#if (defined (USE_STM32F4XX_NUCLEO))
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                         FLASH_FLAG_PGSERR);

#elif (defined (USE_STM32L0XX_NUCLEO))
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_RDERR |
                         FLASH_FLAG_WRPERR | FLASH_FLAG_FWWERR | FLASH_FLAG_NOTZEROERR);

#elif (defined (USE_STM32L1XX_NUCLEO))
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR |
                         FLASH_FLAG_OPTVERRUSR | FLASH_FLAG_WRPERR);

#elif (defined (USE_STM32L4XX_NUCLEO))
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

#else
#error Not supported platform
#endif

  if (HAL_FLASHEx_Erase(&erase_init_struct, &sector_error) != HAL_OK)
  {
    Error_Handler();
  }

  /* Lock the Flash to disable the flash control register access */
  (void)HAL_FLASH_Lock();
}

#if (defined (USE_STM32L4XX_NUCLEO))
/**
 * @brief  Gets the page of a given address
 * @param  Addr: Address of the FLASH Memory
 * @retval The page of a given address
 */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

/**
 * @brief  Gets the bank of a given address
 * @param  Addr: Address of the FLASH Memory
 * @retval The bank of a given address
 */
static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank;

  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
    /* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
    /* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }

  return bank;
}
#endif /* USE_STM32L4XX_NUCLEO */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
