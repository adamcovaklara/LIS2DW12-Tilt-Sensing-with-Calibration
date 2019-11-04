/**
 ******************************************************************************
 * @file    MotionTL_Manager.h
 * @author  MEMS Software Solutions Team
 * @brief   This file contains definitions for the MotionTL_Manager.c file
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
#ifndef MOTIONTL_MANAGER_H
#define MOTIONTL_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "motion_tl.h"
#include "main.h"
#include "DemoDatalog.h"

/* Extern variables ----------------------------------------------------------*/
/* Exported Macros -----------------------------------------------------------*/
/* Exported Types ------------------------------------------------------------*/
/* Imported Variables --------------------------------------------------------*/
/* Exported Functions Prototypes ---------------------------------------------*/
void MotionTL_manager_init(void);
void MotionTL_manager_run(MTL_input_t *data_in);
void MotionTL_manager_getAngles(MTL_output_t *data_out, MTL_angle_mode_t angle_mode);
void MotionTL_manager_get_version(char *version, int *length);
void MotionTL_manager_calibratePosition(MTL_cal_position_t cal_position);
MTL_cal_result_t MotionTL_manager_getCalibrationValues(MTL_acc_cal_t *acc_cal);
void MotionTL_manager_setCalibrationValues(MTL_acc_cal_t *acc_cal);
//void MotionTL_manager_getEstimatedMeasTime(float *time_s);
uint8_t MotionTL_manager_LoadCalValuesFromNVM(MTL_acc_cal_t *acc_cal);
uint8_t MotionTL_manager_SaveCalValuesInNVM(MTL_acc_cal_t *acc_cal);

/* Imported Functions Prototypes ---------------------------------------------*/
uint8_t CollectData(float cal_data[][3], uint32_t num_records);
//void GetEstimatedMeasTime(float *time_s, uint32_t num_records);

#ifdef __cplusplus
}
#endif

#endif /* MOTIONTL_MANAGER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
