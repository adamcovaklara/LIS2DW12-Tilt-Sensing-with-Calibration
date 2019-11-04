/**
 *******************************************************************************
 * @file    DemoDatalog.h
 * @author  MEMS Software Solutions Team
 * @brief   Header for DemoDatalog.c
 *******************************************************************************
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
 *******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEMO_DATALOG_H
#define DEMO_DATALOG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Public types --------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define DATABYTE_LEN                  ((uint8_t)600)

#if (defined (USE_STM32F4XX_NUCLEO))
#define FLASH_SECTOR                  FLASH_SECTOR_6
#define FLASH_SECTOR_SIZE             (SIZE_FLASH_SECTOR_6)
#define SIZE_FLASH_SECTOR_6           ((uint32_t)0x00020000)

#elif (defined (USE_STM32L0XX_NUCLEO))
#define FLASH_SECTOR_SIZE             ((uint32_t)0x00001000)

#elif (defined (USE_STM32L1XX_NUCLEO))
#define FLASH_SECTOR_SIZE             ((uint32_t)0x00020000) /* Size of sector 64 .. 95 */

#elif (defined (USE_STM32L4XX_NUCLEO))
#define FLASH_SECTOR_SIZE             ((uint32_t)0x00020000)

#else
#error Not supported platform
#endif

#define FLASH_ITEM_SIZE               8U

/* Exported defines ----------------------------------------------------------*/
#if (defined (USE_STM32F4XX_NUCLEO))
#define FLASH_ADDRESS                 ((uint32_t)0x08040000)

#elif (defined (USE_STM32L0XX_NUCLEO))
#define FLASH_ADDRESS                 ((uint32_t)0x0800F000) /* page 480 */

#elif (defined (USE_STM32L1XX_NUCLEO))
#define FLASH_ADDRESS                 ((uint32_t)0x08040000) /* page 1024 */

#elif (defined (USE_STM32L4XX_NUCLEO))
#define FLASH_ADDRESS                 ((uint32_t)0x080DF800) /* page 447 */

#else
#error Not supported platform
#endif

/* Public variables ----------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SaveCalibrationToMemory(uint16_t DataSize, uint32_t *Data);
void RecallCalibrationFromMemory(uint16_t DataSize, uint32_t *Data);
void ResetCalibrationInMemory(void);

#ifdef __cplusplus
}
#endif

#endif /* DEMO_DATALOG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

