/**
 ******************************************************************************
 * @file    lps33hw.h
 * @author  MEMS Software Solutions Team
 * @brief   LPS33HW header driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LPS33HW_H
#define LPS33HW_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "lps33hw_reg.h"
#include <string.h> // TODO: Is needed?

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup Component Component
 * @{
 */

/** @addtogroup LPS33HW LPS33HW
 * @{
 */

/** @defgroup LPS33HW_Exported_Types LPS33HW Exported Types
 * @{
 */

typedef int32_t (*LPS33HW_Init_Func)(void);
typedef int32_t (*LPS33HW_DeInit_Func)(void);
typedef int32_t (*LPS33HW_GetTick_Func)(void);
typedef int32_t (*LPS33HW_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*LPS33HW_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef struct
{
  LPS33HW_Init_Func          Init;
  LPS33HW_DeInit_Func        DeInit;
  uint32_t                   BusType; /*0 means I2C, 1 means SPI 4-Wires, 2 means SPI-3-Wires */
  uint8_t                    Address;
  LPS33HW_WriteReg_Func      WriteReg;
  LPS33HW_ReadReg_Func       ReadReg;
  LPS33HW_GetTick_Func       GetTick;
} LPS33HW_IO_t;

typedef struct
{
  LPS33HW_IO_t        IO;
  lps33hw_ctx_t       Ctx;
  uint8_t             is_initialized;
  uint8_t             press_is_enabled;
  uint8_t             temp_is_enabled;
  lps33hw_odr_t       last_odr;
} LPS33HW_Object_t;

typedef struct
{
  uint8_t Temperature;
  uint8_t Pressure;
  uint8_t Humidity;
  uint8_t LowPower;
  float   HumMaxOdr;
  float   TempMaxOdr;
  float   PressMaxOdr;
} LPS33HW_Capabilities_t;

typedef struct
{
  int32_t (*Init)(LPS33HW_Object_t *);
  int32_t (*DeInit)(LPS33HW_Object_t *);
  int32_t (*ReadID)(LPS33HW_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(LPS33HW_Object_t *, LPS33HW_Capabilities_t *);
} LPS33HW_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(LPS33HW_Object_t *);
  int32_t (*Disable)(LPS33HW_Object_t *);
  int32_t (*GetOutputDataRate)(LPS33HW_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LPS33HW_Object_t *, float);
  int32_t (*GetTemperature)(LPS33HW_Object_t *, float *);
} LPS33HW_TEMP_Drv_t;

typedef struct
{
  int32_t (*Enable)(LPS33HW_Object_t *);
  int32_t (*Disable)(LPS33HW_Object_t *);
  int32_t (*GetOutputDataRate)(LPS33HW_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LPS33HW_Object_t *, float);
  int32_t (*GetPressure)(LPS33HW_Object_t *, float *);
} LPS33HW_PRESS_Drv_t;

typedef enum
{
  LPS33HW_FIFO_BYPASS_MODE                    = (uint8_t)0x00,    /*!< The FIFO is disabled and empty. The pressure is read directly*/
  LPS33HW_FIFO_FIFO_MODE                      = (uint8_t)0x20,    /*!< Stops collecting data when full */
  LPS33HW_FIFO_STREAM_MODE                    = (uint8_t)0x40,    /*!< Keep the newest measurements in the FIFO*/
  LPS33HW_FIFO_TRIGGER_STREAMTOFIFO_MODE      = (uint8_t)0x60,    /*!< STREAM MODE until trigger deasserted, then change to FIFO MODE*/
  LPS33HW_FIFO_TRIGGER_BYPASSTOSTREAM_MODE    = (uint8_t)0x80,    /*!< BYPASS MODE until trigger deasserted, then STREAM MODE*/
  LPS33HW_FIFO_TRIGGER_DYNAMICSTREAM_MODE     = (uint8_t)0xC0,    /*!< DYNAMIC STREAM MODE*/
  LPS33HW_FIFO_TRIGGER_BYPASSTOFIFO_MODE      = (uint8_t)0xE0     /*!< BYPASS mode until trigger deasserted, then FIFO MODE*/
} LPS33HW_FifoMode;

/**
 * @}
 */

/** @defgroup LPS33HW_Exported_Constants LPS33HW Exported Constants
 * @{
 */

#define LPS33HW_OK                0
#define LPS33HW_ERROR            -1

#define LPS33HW_I2C_BUS          0U
#define LPS33HW_SPI_4WIRES_BUS   1U
#define LPS33HW_SPI_3WIRES_BUS   2U

#define LPS33HW_FIFO_FULL        (uint8_t)0x20

/**
 * @}
 */

/** @addtogroup LPS33HW_Exported_Functions LPS33HW Exported Functions
 * @{
 */

int32_t LPS33HW_RegisterBusIO(LPS33HW_Object_t *pObj, LPS33HW_IO_t *pIO);
int32_t LPS33HW_Init(LPS33HW_Object_t *pObj);
int32_t LPS33HW_DeInit(LPS33HW_Object_t *pObj);
int32_t LPS33HW_ReadID(LPS33HW_Object_t *pObj, uint8_t *Id);
int32_t LPS33HW_GetCapabilities(LPS33HW_Object_t *pObj, LPS33HW_Capabilities_t *Capabilities);
int32_t LPS33HW_Get_Init_Status(LPS33HW_Object_t *pObj, uint8_t *Status);

int32_t LPS33HW_PRESS_Enable(LPS33HW_Object_t *pObj);
int32_t LPS33HW_PRESS_Disable(LPS33HW_Object_t *pObj);
int32_t LPS33HW_PRESS_GetOutputDataRate(LPS33HW_Object_t *pObj, float *Odr);
int32_t LPS33HW_PRESS_SetOutputDataRate(LPS33HW_Object_t *pObj, float Odr);
int32_t LPS33HW_PRESS_GetPressure(LPS33HW_Object_t *pObj, float *Value);
int32_t LPS33HW_PRESS_Get_DRDY_Status(LPS33HW_Object_t *pObj, uint8_t *Status);

int32_t LPS33HW_TEMP_Enable(LPS33HW_Object_t *pObj);
int32_t LPS33HW_TEMP_Disable(LPS33HW_Object_t *pObj);
int32_t LPS33HW_TEMP_GetOutputDataRate(LPS33HW_Object_t *pObj, float *Odr);
int32_t LPS33HW_TEMP_SetOutputDataRate(LPS33HW_Object_t *pObj, float Odr);
int32_t LPS33HW_TEMP_GetTemperature(LPS33HW_Object_t *pObj, float *Value);
int32_t LPS33HW_TEMP_Get_DRDY_Status(LPS33HW_Object_t *pObj, uint8_t *Status);

int32_t LPS33HW_Read_Reg(LPS33HW_Object_t *pObj, uint8_t reg, uint8_t *Data);
int32_t LPS33HW_Write_Reg(LPS33HW_Object_t *pObj, uint8_t reg, uint8_t Data);

int32_t LPS33HW_Get_Press(LPS33HW_Object_t *pObj, float *Data);
int32_t LPS33HW_Get_Temp(LPS33HW_Object_t *pObj, float *Data);

int32_t LPS33HW_FIFO_Get_Data(LPS33HW_Object_t *pObj, float *Press, float *Temp);
int32_t LPS33HW_FIFO_Get_FTh_Status(LPS33HW_Object_t *pObj, uint8_t *Status);
int32_t LPS33HW_FIFO_Get_Full_Status(LPS33HW_Object_t *pObj, uint8_t *Status);
int32_t LPS33HW_FIFO_Get_Ovr_Status(LPS33HW_Object_t *pObj, uint8_t *Status);
int32_t LPS33HW_FIFO_Get_Level(LPS33HW_Object_t *pObj, uint8_t *Status);
int32_t LPS33HW_FIFO_Reset_Interrupt(LPS33HW_Object_t *pObj, uint8_t interrupt);
int32_t LPS33HW_FIFO_Set_Interrupt(LPS33HW_Object_t *pObj, uint8_t interrupt);
int32_t LPS33HW_FIFO_Set_Mode(LPS33HW_Object_t *pObj, uint8_t Mode);
int32_t LPS33HW_FIFO_Set_Watermark_Level(LPS33HW_Object_t *pObj, uint8_t Watermark);
int32_t LPS33HW_FIFO_Stop_On_Watermark(LPS33HW_Object_t *pObj, uint8_t Stop);
int32_t LPS33HW_FIFO_Usage(LPS33HW_Object_t *pObj, uint8_t Status);

int32_t LPS33HW_Set_One_Shot(LPS33HW_Object_t *pObj);
int32_t LPS33HW_Get_One_Shot_Status(LPS33HW_Object_t *pObj, uint8_t *Status);

/**
 * @}
 */

/** @addtogroup LPS33HW_Exported_Variables LPS33HW Exported Variables
 * @{
 */
extern LPS33HW_CommonDrv_t LPS33HW_COMMON_Driver;
extern LPS33HW_PRESS_Drv_t LPS33HW_PRESS_Driver;
extern LPS33HW_TEMP_Drv_t LPS33HW_TEMP_Driver;

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/