/**
 ******************************************************************************
 * @file    lis3mdl.h
 * @author  MCD Application Team
 * @brief   LIS3MDL header driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#ifndef LIS3MDL_H
#define LIS3MDL_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "lis3mdl_reg.h"
#include <stddef.h>
#include <string.h>
/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @addtogroup LIS3MDL
  * @{
  */

/** @defgroup LIS3MDL_Exported_Types LIS3MDL Exported Types
 * @{
 */
typedef int32_t (*LIS3MDL_Init_Func)(void);
typedef int32_t (*LIS3MDL_DeInit_Func)(void);
typedef int32_t (*LIS3MDL_GetTick_Func)(void);
typedef int32_t (*LIS3MDL_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*LIS3MDL_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef struct
{
  LIS3MDL_Init_Func          Init;
  LIS3MDL_DeInit_Func        DeInit;
  uint32_t                   BusType; /*0 means I2C, 1 means SPI 4-Wires, 2 means SPI-3-Wires */
  uint8_t                    Address;
  LIS3MDL_WriteReg_Func      WriteReg;
  LIS3MDL_ReadReg_Func       ReadReg;
  LIS3MDL_GetTick_Func       GetTick;
} LIS3MDL_IO_t;


typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} LIS3MDL_AxesRaw_t;

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} LIS3MDL_Axes_t;

typedef struct
{
  LIS3MDL_IO_t        IO;
  lis3mdl_ctx_t       Ctx;
  uint8_t             is_initialized;
  uint8_t             mag_is_enabled;
} LIS3MDL_Object_t;

typedef struct
{
  uint8_t   Acc;
  uint8_t   Gyro;
  uint8_t   Magneto;
  uint8_t   LowPower;
  uint32_t  GyroMaxFS;
  uint32_t  AccMaxFS;
  uint32_t  MagMaxFS;
  float     GyroMaxOdr;
  float     AccMaxOdr;
  float     MagMaxOdr;
} LIS3MDL_Capabilities_t;

typedef struct
{
  int32_t (*Init)(LIS3MDL_Object_t *);
  int32_t (*DeInit)(LIS3MDL_Object_t *);
  int32_t (*ReadID)(LIS3MDL_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(LIS3MDL_Object_t *, LIS3MDL_Capabilities_t *);
} LIS3MDL_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(LIS3MDL_Object_t *);
  int32_t (*Disable)(LIS3MDL_Object_t *);
  int32_t (*GetSensitivity)(LIS3MDL_Object_t *, float *);
  int32_t (*GetOutputDataRate)(LIS3MDL_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LIS3MDL_Object_t *, float);
  int32_t (*GetFullScale)(LIS3MDL_Object_t *, int32_t *);
  int32_t (*SetFullScale)(LIS3MDL_Object_t *, int32_t);
  int32_t (*GetAxes)(LIS3MDL_Object_t *, LIS3MDL_Axes_t *);
  int32_t (*GetAxesRaw)(LIS3MDL_Object_t *, LIS3MDL_AxesRaw_t *);
} LIS3MDL_MAG_Drv_t;

/**
  * @}
  */

/** @defgroup LIS3MDL_Exported_Constants LIS3MDL Exported Constants
 * @{
 */
#define LIS3MDL_OK            0
#define LIS3MDL_ERROR        -1

#define LIS3MDL_I2C_BUS         0U
#define LIS3MDL_SPI_4WIRES_BUS  1U
#define LIS3MDL_SPI_3WIRES_BUS  2U

#define LIS3MDL_MAG_SENSITIVITY_FS_4GAUSS   0.146f /**< Sensitivity value for 4 gauss full scale [mgauss/LSB] */
#define LIS3MDL_MAG_SENSITIVITY_FS_8GAUSS   0.292f /**< Sensitivity value for 8 gauss full scale [mgauss/LSB] */
#define LIS3MDL_MAG_SENSITIVITY_FS_12GAUSS  0.438f /**< Sensitivity value for 12 gauss full scale [mgauss/LSB] */
#define LIS3MDL_MAG_SENSITIVITY_FS_16GAUSS  0.584f /**< Sensitivity value for 16 gauss full scale [mgauss/LSB] */

/**
  * @}
  */

/** @addtogroup LIS3MDL_Exported_Functions
 * @{
 */
int32_t LIS3MDL_RegisterBusIO(LIS3MDL_Object_t *pObj, LIS3MDL_IO_t *pIO);
int32_t LIS3MDL_Init(LIS3MDL_Object_t *pObj);
int32_t LIS3MDL_DeInit(LIS3MDL_Object_t *pObj);
int32_t LIS3MDL_ReadID(LIS3MDL_Object_t *pObj, uint8_t *Id);
int32_t LIS3MDL_GetCapabilities(LIS3MDL_Object_t *pObj, LIS3MDL_Capabilities_t *Capabilities);

int32_t LIS3MDL_MAG_Enable(LIS3MDL_Object_t *pObj);
int32_t LIS3MDL_MAG_Disable(LIS3MDL_Object_t *pObj);
int32_t LIS3MDL_MAG_GetSensitivity(LIS3MDL_Object_t *pObj, float *Sensitivity);
int32_t LIS3MDL_MAG_GetOutputDataRate(LIS3MDL_Object_t *pObj, float  *Odr);
int32_t LIS3MDL_MAG_SetOutputDataRate(LIS3MDL_Object_t *pObj, float  Odr);
int32_t LIS3MDL_MAG_GetFullScale(LIS3MDL_Object_t *pObj, int32_t  *FullScale);
int32_t LIS3MDL_MAG_SetFullScale(LIS3MDL_Object_t *pObj, int32_t FullScale);
int32_t LIS3MDL_MAG_GetAxes(LIS3MDL_Object_t *pObj, LIS3MDL_Axes_t *MagneticField);
int32_t LIS3MDL_MAG_GetAxesRaw(LIS3MDL_Object_t *pObj, LIS3MDL_AxesRaw_t *Value);

int32_t LIS3MDL_Read_Reg(LIS3MDL_Object_t *pObj, uint8_t Reg, uint8_t *pData);
int32_t LIS3MDL_Write_Reg(LIS3MDL_Object_t *pObj, uint8_t Reg, uint8_t Data);
int32_t LIS3MDL_MAG_Get_DRDY_Status(LIS3MDL_Object_t *pObj, uint8_t *Status);
/**
  * @}
  */

/** @addtogroup LIS3MDL_Exported_Variables
  * @{
  */
extern LIS3MDL_CommonDrv_t LIS3MDL_COMMON_Driver;
extern LIS3MDL_MAG_Drv_t LIS3MDL_MAG_Driver;
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
