/**
 ******************************************************************************
 * @file    lsm6dso.h
 * @author  MEMS Software Solutions Team
 * @brief   LSM6DSO header driver file
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
#ifndef LSM6DSO_H
#define LSM6DSO_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "lsm6dso_reg.h"
#include <string.h>

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup Component Component
 * @{
 */

/** @addtogroup LSM6DSO LSM6DSO
 * @{
 */

/** @defgroup LSM6DSO_Exported_Types LSM6DSO Exported Types
 * @{
 */

typedef int32_t (*LSM6DSO_Init_Func)(void);
typedef int32_t (*LSM6DSO_DeInit_Func)(void);
typedef int32_t (*LSM6DSO_GetTick_Func)(void);
typedef int32_t (*LSM6DSO_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*LSM6DSO_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef enum
{
  LSM6DSO_INT1_PIN,
  LSM6DSO_INT2_PIN,
} LSM6DSO_SensorIntPin_t;

typedef struct
{
  LSM6DSO_Init_Func          Init;
  LSM6DSO_DeInit_Func        DeInit;
  uint32_t                   BusType; /*0 means I2C, 1 means SPI 4-Wires, 2 means SPI-3-Wires */
  uint8_t                    Address;
  LSM6DSO_WriteReg_Func      WriteReg;
  LSM6DSO_ReadReg_Func       ReadReg;
  LSM6DSO_GetTick_Func       GetTick;
} LSM6DSO_IO_t;


typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} LSM6DSO_AxesRaw_t;

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} LSM6DSO_Axes_t;

typedef struct
{
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
  unsigned int SleepStatus : 1;
} LSM6DSO_Event_Status_t;

typedef struct
{
  LSM6DSO_IO_t        IO;
  lsm6dso_ctx_t       Ctx;
  uint8_t             is_initialized;
  uint8_t             acc_is_enabled;
  uint8_t             gyro_is_enabled;
  lsm6dso_odr_xl_t    acc_odr;
  lsm6dso_odr_g_t     gyro_odr;
} LSM6DSO_Object_t;

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
} LSM6DSO_Capabilities_t;

typedef struct
{
  int32_t (*Init)(LSM6DSO_Object_t *);
  int32_t (*DeInit)(LSM6DSO_Object_t *);
  int32_t (*ReadID)(LSM6DSO_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(LSM6DSO_Object_t *, LSM6DSO_Capabilities_t *);
} LSM6DSO_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(LSM6DSO_Object_t *);
  int32_t (*Disable)(LSM6DSO_Object_t *);
  int32_t (*GetSensitivity)(LSM6DSO_Object_t *, float *);
  int32_t (*GetOutputDataRate)(LSM6DSO_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LSM6DSO_Object_t *, float);
  int32_t (*GetFullScale)(LSM6DSO_Object_t *, int32_t *);
  int32_t (*SetFullScale)(LSM6DSO_Object_t *, int32_t);
  int32_t (*GetAxes)(LSM6DSO_Object_t *, LSM6DSO_Axes_t *);
  int32_t (*GetAxesRaw)(LSM6DSO_Object_t *, LSM6DSO_AxesRaw_t *);
} LSM6DSO_ACC_Drv_t;

typedef struct
{
  int32_t (*Enable)(LSM6DSO_Object_t *);
  int32_t (*Disable)(LSM6DSO_Object_t *);
  int32_t (*GetSensitivity)(LSM6DSO_Object_t *, float *);
  int32_t (*GetOutputDataRate)(LSM6DSO_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LSM6DSO_Object_t *, float);
  int32_t (*GetFullScale)(LSM6DSO_Object_t *, int32_t *);
  int32_t (*SetFullScale)(LSM6DSO_Object_t *, int32_t);
  int32_t (*GetAxes)(LSM6DSO_Object_t *, LSM6DSO_Axes_t *);
  int32_t (*GetAxesRaw)(LSM6DSO_Object_t *, LSM6DSO_AxesRaw_t *);
} LSM6DSO_GYRO_Drv_t;

/**
 * @}
 */

/** @defgroup LSM6DSO_Exported_Constants LSM6DSO Exported Constants
 * @{
 */

#define LSM6DSO_OK                       0
#define LSM6DSO_ERROR                   -1

#define LSM6DSO_I2C_BUS                 0U
#define LSM6DSO_SPI_4WIRES_BUS          1U
#define LSM6DSO_SPI_3WIRES_BUS          2U

#define LSM6DSO_ACC_SENSITIVITY_FS_2G   0.061f
#define LSM6DSO_ACC_SENSITIVITY_FS_4G   0.122f
#define LSM6DSO_ACC_SENSITIVITY_FS_8G   0.244f
#define LSM6DSO_ACC_SENSITIVITY_FS_16G  0.488f

#define LSM6DSO_GYRO_SENSITIVITY_FS_125DPS    4.375f
#define LSM6DSO_GYRO_SENSITIVITY_FS_250DPS    8.750f
#define LSM6DSO_GYRO_SENSITIVITY_FS_500DPS   17.500f
#define LSM6DSO_GYRO_SENSITIVITY_FS_1000DPS  35.000f
#define LSM6DSO_GYRO_SENSITIVITY_FS_2000DPS  70.000f

/**
 * @}
 */

/** @addtogroup LSM6DSO_Exported_Functions LSM6DSO Exported Functions
 * @{
 */

int32_t LSM6DSO_RegisterBusIO(LSM6DSO_Object_t *pObj, LSM6DSO_IO_t *pIO);
int32_t LSM6DSO_Init(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_DeInit(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_ReadID(LSM6DSO_Object_t *pObj, uint8_t *Id);
int32_t LSM6DSO_GetCapabilities(LSM6DSO_Object_t *pObj, LSM6DSO_Capabilities_t *Capabilities);

int32_t LSM6DSO_ACC_Enable(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_ACC_Disable(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_ACC_GetSensitivity(LSM6DSO_Object_t *pObj, float *Sensitivity);
int32_t LSM6DSO_ACC_GetOutputDataRate(LSM6DSO_Object_t *pObj, float *Odr);
int32_t LSM6DSO_ACC_SetOutputDataRate(LSM6DSO_Object_t *pObj, float Odr);
int32_t LSM6DSO_ACC_GetFullScale(LSM6DSO_Object_t *pObj, int32_t *FullScale);
int32_t LSM6DSO_ACC_SetFullScale(LSM6DSO_Object_t *pObj, int32_t FullScale);
int32_t LSM6DSO_ACC_GetAxesRaw(LSM6DSO_Object_t *pObj, LSM6DSO_AxesRaw_t *Value);
int32_t LSM6DSO_ACC_GetAxes(LSM6DSO_Object_t *pObj, LSM6DSO_Axes_t *Acceleration);

int32_t LSM6DSO_GYRO_Enable(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_GYRO_Disable(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_GYRO_GetSensitivity(LSM6DSO_Object_t *pObj, float *Sensitivity);
int32_t LSM6DSO_GYRO_GetOutputDataRate(LSM6DSO_Object_t *pObj, float *Odr);
int32_t LSM6DSO_GYRO_SetOutputDataRate(LSM6DSO_Object_t *pObj, float Odr);
int32_t LSM6DSO_GYRO_GetFullScale(LSM6DSO_Object_t *pObj, int32_t *FullScale);
int32_t LSM6DSO_GYRO_SetFullScale(LSM6DSO_Object_t *pObj, int32_t FullScale);
int32_t LSM6DSO_GYRO_GetAxesRaw(LSM6DSO_Object_t *pObj, LSM6DSO_AxesRaw_t *Value);
int32_t LSM6DSO_GYRO_GetAxes(LSM6DSO_Object_t *pObj, LSM6DSO_Axes_t *AngularRate);

int32_t LSM6DSO_Read_Reg(LSM6DSO_Object_t *pObj, uint8_t reg, uint8_t *Data);
int32_t LSM6DSO_Write_Reg(LSM6DSO_Object_t *pObj, uint8_t reg, uint8_t Data);
int32_t LSM6DSO_Set_Interrupt_Latch(LSM6DSO_Object_t *pObj, uint8_t Status);

int32_t LSM6DSO_ACC_Enable_Free_Fall_Detection(LSM6DSO_Object_t *pObj, LSM6DSO_SensorIntPin_t IntPin);
int32_t LSM6DSO_ACC_Disable_Free_Fall_Detection(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_ACC_Set_Free_Fall_Threshold(LSM6DSO_Object_t *pObj, uint8_t Threshold);
int32_t LSM6DSO_ACC_Set_Free_Fall_Duration(LSM6DSO_Object_t *pObj, uint8_t Duration);

int32_t LSM6DSO_ACC_Enable_Pedometer(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_ACC_Disable_Pedometer(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_ACC_Get_Step_Count(LSM6DSO_Object_t *pObj, uint16_t *StepCount);
int32_t LSM6DSO_ACC_Step_Counter_Reset(LSM6DSO_Object_t *pObj);

int32_t LSM6DSO_ACC_Enable_Tilt_Detection(LSM6DSO_Object_t *pObj, LSM6DSO_SensorIntPin_t IntPin);
int32_t LSM6DSO_ACC_Disable_Tilt_Detection(LSM6DSO_Object_t *pObj);

int32_t LSM6DSO_ACC_Enable_Wake_Up_Detection(LSM6DSO_Object_t *pObj, LSM6DSO_SensorIntPin_t IntPin);
int32_t LSM6DSO_ACC_Disable_Wake_Up_Detection(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_ACC_Set_Wake_Up_Threshold(LSM6DSO_Object_t *pObj, uint8_t Threshold);
int32_t LSM6DSO_ACC_Set_Wake_Up_Duration(LSM6DSO_Object_t *pObj, uint8_t Duration);

int32_t LSM6DSO_ACC_Enable_Single_Tap_Detection(LSM6DSO_Object_t *pObj, LSM6DSO_SensorIntPin_t IntPin);
int32_t LSM6DSO_ACC_Disable_Single_Tap_Detection(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_ACC_Enable_Double_Tap_Detection(LSM6DSO_Object_t *pObj, LSM6DSO_SensorIntPin_t IntPin);
int32_t LSM6DSO_ACC_Disable_Double_Tap_Detection(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_ACC_Set_Tap_Threshold(LSM6DSO_Object_t *pObj, uint8_t Threshold);
int32_t LSM6DSO_ACC_Set_Tap_Shock_Time(LSM6DSO_Object_t *pObj, uint8_t Time);
int32_t LSM6DSO_ACC_Set_Tap_Quiet_Time(LSM6DSO_Object_t *pObj, uint8_t Time);
int32_t LSM6DSO_ACC_Set_Tap_Duration_Time(LSM6DSO_Object_t *pObj, uint8_t Time);

int32_t LSM6DSO_ACC_Enable_6D_Orientation(LSM6DSO_Object_t *pObj, LSM6DSO_SensorIntPin_t IntPin);
int32_t LSM6DSO_ACC_Disable_6D_Orientation(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_ACC_Set_6D_Orientation_Threshold(LSM6DSO_Object_t *pObj, uint8_t Threshold);
int32_t LSM6DSO_ACC_Get_6D_Orientation_XL(LSM6DSO_Object_t *pObj, uint8_t *XLow);
int32_t LSM6DSO_ACC_Get_6D_Orientation_XH(LSM6DSO_Object_t *pObj, uint8_t *XHigh);
int32_t LSM6DSO_ACC_Get_6D_Orientation_YL(LSM6DSO_Object_t *pObj, uint8_t *YLow);
int32_t LSM6DSO_ACC_Get_6D_Orientation_YH(LSM6DSO_Object_t *pObj, uint8_t *YHigh);
int32_t LSM6DSO_ACC_Get_6D_Orientation_ZL(LSM6DSO_Object_t *pObj, uint8_t *ZLow);
int32_t LSM6DSO_ACC_Get_6D_Orientation_ZH(LSM6DSO_Object_t *pObj, uint8_t *ZHigh);

int32_t LSM6DSO_ACC_Get_DRDY_Status(LSM6DSO_Object_t *pObj, uint8_t *Status);
int32_t LSM6DSO_ACC_Get_Event_Status(LSM6DSO_Object_t *pObj, LSM6DSO_Event_Status_t *Status);
int32_t LSM6DSO_ACC_Set_SelfTest(LSM6DSO_Object_t *pObj, uint8_t Status);

int32_t LSM6DSO_GYRO_Get_DRDY_Status(LSM6DSO_Object_t *pObj, uint8_t *Status);
int32_t LSM6DSO_GYRO_Set_SelfTest(LSM6DSO_Object_t *pObj, uint8_t Status);

int32_t LSM6DSO_FIFO_Get_Num_Samples(LSM6DSO_Object_t *pObj, uint16_t *NumSamples);
int32_t LSM6DSO_FIFO_Get_Full_Status(LSM6DSO_Object_t *pObj, uint8_t *Status);
int32_t LSM6DSO_FIFO_Set_INT1_FIFO_Full(LSM6DSO_Object_t *pObj, uint8_t Status);
int32_t LSM6DSO_FIFO_Set_Watermark_Level(LSM6DSO_Object_t *pObj, uint16_t Watermark);
int32_t LSM6DSO_FIFO_Set_Stop_On_Fth(LSM6DSO_Object_t *pObj, uint8_t Status);
int32_t LSM6DSO_FIFO_Set_Mode(LSM6DSO_Object_t *pObj, uint8_t Mode);
int32_t LSM6DSO_FIFO_Get_Tag(LSM6DSO_Object_t *pObj, uint8_t *Tag);
int32_t LSM6DSO_FIFO_Get_Data(LSM6DSO_Object_t *pObj, uint8_t *Data);
int32_t LSM6DSO_FIFO_Get_Empty_Status(LSM6DSO_Object_t *pObj, uint8_t *Status);
int32_t LSM6DSO_FIFO_Get_Overrun_Status(LSM6DSO_Object_t *pObj, uint8_t *Status);
int32_t LSM6DSO_FIFO_ACC_Get_Axes(LSM6DSO_Object_t *pObj, LSM6DSO_Axes_t *Acceleration);
int32_t LSM6DSO_FIFO_ACC_Set_BDR(LSM6DSO_Object_t *pObj, float Bdr);
int32_t LSM6DSO_FIFO_GYRO_Get_Axes(LSM6DSO_Object_t *pObj, LSM6DSO_Axes_t *AngularVelocity);
int32_t LSM6DSO_FIFO_GYRO_Set_BDR(LSM6DSO_Object_t *pObj, float Bdr);

int32_t LSM6DSO_ACC_Enable_DRDY_On_INT1(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_ACC_Disable_DRDY_On_INT1(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_ACC_Set_Power_Mode(LSM6DSO_Object_t *pObj, uint8_t PowerMode);
int32_t LSM6DSO_ACC_Set_Filter_Mode(LSM6DSO_Object_t *pObj, uint8_t LowHighPassFlag, uint8_t FilterMode);
int32_t LSM6DSO_ACC_Enable_Inactivity_Detection(LSM6DSO_Object_t *pObj, lsm6dso_inact_en_t InactMode, LSM6DSO_SensorIntPin_t IntPin);
int32_t LSM6DSO_ACC_Disable_Inactivity_Detection(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_ACC_Set_Sleep_Duration(LSM6DSO_Object_t *pObj, uint8_t Duration);

int32_t LSM6DSO_GYRO_Enable_DRDY_On_INT2(LSM6DSO_Object_t *pObj);
int32_t LSM6DSO_GYRO_Set_Power_Mode(LSM6DSO_Object_t *pObj, uint8_t PowerMode);
int32_t LSM6DSO_GYRO_Set_Filter_Mode(LSM6DSO_Object_t *pObj, uint8_t LowHighPassFlag, uint8_t FilterMode);

int32_t LSM6DSO_DRDY_Set_Mode(LSM6DSO_Object_t *pObj, uint8_t Mode);

/**
 * @}
 */

/** @addtogroup LSM6DSO_Exported_Variables LSM6DSO Exported Variables
 * @{
 */

extern LSM6DSO_CommonDrv_t LSM6DSO_COMMON_Driver;
extern LSM6DSO_ACC_Drv_t LSM6DSO_ACC_Driver;
extern LSM6DSO_GYRO_Drv_t LSM6DSO_GYRO_Driver;

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
