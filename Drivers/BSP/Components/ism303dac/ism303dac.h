/**
 ******************************************************************************
 * @file    ism303dac.h
 * @author  MEMS Software Solutions Team
 * @brief   ISM303DAC header driver file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ISM303DAC_H
#define ISM303DAC_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "ism303dac_reg.h"
#include <string.h>

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup Component Component
 * @{
 */

/** @addtogroup ISM303DAC ISM303DAC
 * @{
 */

/** @defgroup ISM303DAC_Exported_Types ISM303DAC Exported Types
 * @{
 */

typedef int32_t (*ISM303DAC_Init_Func)(void);
typedef int32_t (*ISM303DAC_DeInit_Func)(void);
typedef int32_t (*ISM303DAC_GetTick_Func)(void);
typedef int32_t (*ISM303DAC_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*ISM303DAC_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef struct
{
  ISM303DAC_Init_Func          Init;
  ISM303DAC_DeInit_Func        DeInit;
  uint32_t                     BusType; /*0 means I2C, 1 means SPI-3-Wires */
  uint8_t                      Address;
  ISM303DAC_WriteReg_Func      WriteReg;
  ISM303DAC_ReadReg_Func       ReadReg;
  ISM303DAC_GetTick_Func       GetTick;
} ISM303DAC_IO_t;


typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} ISM303DAC_AxesRaw_t;

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} ISM303DAC_Axes_t;

typedef struct
{
  ISM303DAC_IO_t        IO;
  ism303dac_ctx_t       Ctx;
  uint8_t               is_initialized;
  uint8_t               acc_is_enabled;
  ism303dac_xl_odr_t    acc_odr;
} ISM303DAC_ACC_Object_t;

typedef struct
{
  ISM303DAC_IO_t        IO;
  ism303dac_ctx_t       Ctx;
  uint8_t               is_initialized;
  uint8_t               mag_is_enabled;
} ISM303DAC_MAG_Object_t;

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
} ISM303DAC_Capabilities_t;

typedef struct
{
  int32_t (*Init)(ISM303DAC_ACC_Object_t *);
  int32_t (*DeInit)(ISM303DAC_ACC_Object_t *);
  int32_t (*ReadID)(ISM303DAC_ACC_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(ISM303DAC_ACC_Object_t *, ISM303DAC_Capabilities_t *);
} ISM303DAC_ACC_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(ISM303DAC_ACC_Object_t *);
  int32_t (*Disable)(ISM303DAC_ACC_Object_t *);
  int32_t (*GetSensitivity)(ISM303DAC_ACC_Object_t *, float *);
  int32_t (*GetOutputDataRate)(ISM303DAC_ACC_Object_t *, float *);
  int32_t (*SetOutputDataRate)(ISM303DAC_ACC_Object_t *, float);
  int32_t (*GetFullScale)(ISM303DAC_ACC_Object_t *, int32_t *);
  int32_t (*SetFullScale)(ISM303DAC_ACC_Object_t *, int32_t);
  int32_t (*GetAxes)(ISM303DAC_ACC_Object_t *, ISM303DAC_Axes_t *);
  int32_t (*GetAxesRaw)(ISM303DAC_ACC_Object_t *, ISM303DAC_AxesRaw_t *);
} ISM303DAC_ACC_Drv_t;

typedef struct
{
  int32_t (*Init)(ISM303DAC_MAG_Object_t *);
  int32_t (*DeInit)(ISM303DAC_MAG_Object_t *);
  int32_t (*ReadID)(ISM303DAC_MAG_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(ISM303DAC_MAG_Object_t *, ISM303DAC_Capabilities_t *);
} ISM303DAC_MAG_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(ISM303DAC_MAG_Object_t *);
  int32_t (*Disable)(ISM303DAC_MAG_Object_t *);
  int32_t (*GetSensitivity)(ISM303DAC_MAG_Object_t *, float *);
  int32_t (*GetOutputDataRate)(ISM303DAC_MAG_Object_t *, float *);
  int32_t (*SetOutputDataRate)(ISM303DAC_MAG_Object_t *, float);
  int32_t (*GetFullScale)(ISM303DAC_MAG_Object_t *, int32_t *);
  int32_t (*SetFullScale)(ISM303DAC_MAG_Object_t *, int32_t);
  int32_t (*GetAxes)(ISM303DAC_MAG_Object_t *, ISM303DAC_Axes_t *);
  int32_t (*GetAxesRaw)(ISM303DAC_MAG_Object_t *, ISM303DAC_AxesRaw_t *);
} ISM303DAC_MAG_Drv_t;

/**
 * @}
 */

/** @defgroup ISM303DAC_Exported_Constants ISM303DAC Exported Constants
 * @{
 */

#define ISM303DAC_OK                     0
#define ISM303DAC_ERROR                 -1

#define ISM303DAC_I2C_BUS               0U
#define ISM303DAC_SPI_3WIRES_BUS        1U

#define ISM303DAC_ACC_SENSITIVITY_FS_2G       0.061f  /**< Sensitivity value for 2 g full scale [mg/LSB] */
#define ISM303DAC_ACC_SENSITIVITY_FS_4G       0.122f  /**< Sensitivity value for 4 g full scale [mg/LSB] */
#define ISM303DAC_ACC_SENSITIVITY_FS_8G       0.244f  /**< Sensitivity value for 8 g full scale [mg/LSB] */
#define ISM303DAC_ACC_SENSITIVITY_FS_16G      0.488f  /**< Sensitivity value for 16 g full scale [mg/LSB] */

#define ISM303DAC_MAG_SENSITIVITY_FS_50GAUSS  1.500f  /**< Sensitivity value for 50 gauss full scale [mgauss/LSB] */

/**
 * @}
 */

/** @defgroup ISM303DAC_Exported_Functions ISM303DAC Exported Functions
 * @{
 */

int32_t ISM303DAC_ACC_RegisterBusIO(ISM303DAC_ACC_Object_t *pObj, ISM303DAC_IO_t *pIO);
int32_t ISM303DAC_ACC_Init(ISM303DAC_ACC_Object_t *pObj);
int32_t ISM303DAC_ACC_DeInit(ISM303DAC_ACC_Object_t *pObj);
int32_t ISM303DAC_ACC_ReadID(ISM303DAC_ACC_Object_t *pObj, uint8_t *Id);
int32_t ISM303DAC_ACC_GetCapabilities(ISM303DAC_ACC_Object_t *pObj, ISM303DAC_Capabilities_t *Capabilities);

int32_t ISM303DAC_ACC_Enable(ISM303DAC_ACC_Object_t *pObj);
int32_t ISM303DAC_ACC_Disable(ISM303DAC_ACC_Object_t *pObj);
int32_t ISM303DAC_ACC_GetSensitivity(ISM303DAC_ACC_Object_t *pObj, float *sensitivity);
int32_t ISM303DAC_ACC_GetOutputDataRate(ISM303DAC_ACC_Object_t *pObj, float *odr);
int32_t ISM303DAC_ACC_SetOutputDataRate(ISM303DAC_ACC_Object_t *pObj, float odr);
int32_t ISM303DAC_ACC_GetFullScale(ISM303DAC_ACC_Object_t *pObj, int32_t *fullscale);
int32_t ISM303DAC_ACC_SetFullScale(ISM303DAC_ACC_Object_t *pObj, int32_t fullscale);
int32_t ISM303DAC_ACC_GetAxes(ISM303DAC_ACC_Object_t *pObj, ISM303DAC_Axes_t *acceleration);
int32_t ISM303DAC_ACC_GetAxesRaw(ISM303DAC_ACC_Object_t *pObj, ISM303DAC_AxesRaw_t *value);

int32_t ISM303DAC_ACC_Read_Reg(ISM303DAC_ACC_Object_t *pObj, uint8_t reg, uint8_t *data);
int32_t ISM303DAC_ACC_Write_Reg(ISM303DAC_ACC_Object_t *pObj, uint8_t reg, uint8_t data);

int32_t ISM303DAC_ACC_Get_DRDY_Status(ISM303DAC_ACC_Object_t *pObj, uint8_t *status);
int32_t ISM303DAC_ACC_Get_Init_Status(ISM303DAC_ACC_Object_t *pObj, uint8_t *status);

int32_t ISM303DAC_MAG_RegisterBusIO(ISM303DAC_MAG_Object_t *pObj, ISM303DAC_IO_t *pIO);
int32_t ISM303DAC_MAG_Init(ISM303DAC_MAG_Object_t *pObj);
int32_t ISM303DAC_MAG_DeInit(ISM303DAC_MAG_Object_t *pObj);
int32_t ISM303DAC_MAG_ReadID(ISM303DAC_MAG_Object_t *pObj, uint8_t *id);
int32_t ISM303DAC_MAG_GetCapabilities(ISM303DAC_MAG_Object_t *pObj, ISM303DAC_Capabilities_t *Capabilities);

int32_t ISM303DAC_MAG_Enable(ISM303DAC_MAG_Object_t *pObj);
int32_t ISM303DAC_MAG_Disable(ISM303DAC_MAG_Object_t *pObj);
int32_t ISM303DAC_MAG_GetSensitivity(ISM303DAC_MAG_Object_t *pObj, float *sensitivity);
int32_t ISM303DAC_MAG_GetOutputDataRate(ISM303DAC_MAG_Object_t *pObj, float *odr);
int32_t ISM303DAC_MAG_SetOutputDataRate(ISM303DAC_MAG_Object_t *pObj, float odr);
int32_t ISM303DAC_MAG_GetFullScale(ISM303DAC_MAG_Object_t *pObj, int32_t *fullscale);
int32_t ISM303DAC_MAG_SetFullScale(ISM303DAC_MAG_Object_t *pObj, int32_t fullscale);
int32_t ISM303DAC_MAG_GetAxes(ISM303DAC_MAG_Object_t *pObj, ISM303DAC_Axes_t *magnetic_field);
int32_t ISM303DAC_MAG_GetAxesRaw(ISM303DAC_MAG_Object_t *pObj, ISM303DAC_AxesRaw_t *value);

int32_t ISM303DAC_MAG_Read_Reg(ISM303DAC_MAG_Object_t *pObj, uint8_t reg, uint8_t *data);
int32_t ISM303DAC_MAG_Write_Reg(ISM303DAC_MAG_Object_t *pObj, uint8_t reg, uint8_t data);

int32_t ISM303DAC_MAG_Get_DRDY_Status(ISM303DAC_MAG_Object_t *pObj, uint8_t *status);
int32_t ISM303DAC_MAG_Get_Init_Status(ISM303DAC_MAG_Object_t *pObj, uint8_t *status);

/**
 * @}
 */

/** @addtogroup ISM303DAC_Exported_Variables ISM303DAC Exported Variables
 * @{
 */

extern ISM303DAC_ACC_CommonDrv_t ISM303DAC_ACC_COMMON_Driver;
extern ISM303DAC_ACC_Drv_t ISM303DAC_ACC_Driver;
extern ISM303DAC_MAG_CommonDrv_t ISM303DAC_MAG_COMMON_Driver;
extern ISM303DAC_MAG_Drv_t ISM303DAC_MAG_Driver;

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
