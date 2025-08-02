/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GRIMSON_L_PWM_Pin GPIO_PIN_5
#define GRIMSON_L_PWM_GPIO_Port GPIOE
#define GRIMSON_R_PWM_Pin GPIO_PIN_6
#define GRIMSON_R_PWM_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define BASE_HOMING_GPIO_INP_Pin GPIO_PIN_0
#define BASE_HOMING_GPIO_INP_GPIO_Port GPIOC
#define SHOULDER_HOMING_GPIO_INP_Pin GPIO_PIN_1
#define SHOULDER_HOMING_GPIO_INP_GPIO_Port GPIOC
#define ELBOW_HOMING_GPIO_INP_Pin GPIO_PIN_2
#define ELBOW_HOMING_GPIO_INP_GPIO_Port GPIOC
#define BASE_PWM_Pin GPIO_PIN_0
#define BASE_PWM_GPIO_Port GPIOA
#define ENC_B_SHOULDER_NEO_Pin GPIO_PIN_1
#define ENC_B_SHOULDER_NEO_GPIO_Port GPIOA
#define SHOULDER_PWM_Pin GPIO_PIN_2
#define SHOULDER_PWM_GPIO_Port GPIOA
#define ELBOW_PWM_Pin GPIO_PIN_3
#define ELBOW_PWM_GPIO_Port GPIOA
#define ENC_A_SHOULDER_NEO_Pin GPIO_PIN_5
#define ENC_A_SHOULDER_NEO_GPIO_Port GPIOA
#define ENC_A_ELBOW_NEO_Pin GPIO_PIN_6
#define ENC_A_ELBOW_NEO_GPIO_Port GPIOA
#define ENC_B_ELBOW_NEO_Pin GPIO_PIN_7
#define ENC_B_ELBOW_NEO_GPIO_Port GPIOA
#define GRIMSON_LEFT_DIR_GPIO_O_Pin GPIO_PIN_5
#define GRIMSON_LEFT_DIR_GPIO_O_GPIO_Port GPIOC
#define GRIMSON_RIGHT_DIR_GPIO_O_Pin GPIO_PIN_0
#define GRIMSON_RIGHT_DIR_GPIO_O_GPIO_Port GPIOB
#define NEMA_ENABLE_GPIO_O_Pin GPIO_PIN_1
#define NEMA_ENABLE_GPIO_O_GPIO_Port GPIOB
#define NEMA_DIR_GPIO_O_Pin GPIO_PIN_2
#define NEMA_DIR_GPIO_O_GPIO_Port GPIOB
#define ENC_A_BASE_NEO_Pin GPIO_PIN_9
#define ENC_A_BASE_NEO_GPIO_Port GPIOE
#define ENC_B_BASE_NEO_Pin GPIO_PIN_11
#define ENC_B_BASE_NEO_GPIO_Port GPIOE
#define ENC_A_GRIMSON_L_Pin GPIO_PIN_12
#define ENC_A_GRIMSON_L_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define ENC_A_GRIMSON_R_Pin GPIO_PIN_6
#define ENC_A_GRIMSON_R_GPIO_Port GPIOC
#define ENC_B_GRIMSON_R_Pin GPIO_PIN_7
#define ENC_B_GRIMSON_R_GPIO_Port GPIOC
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define ENC_B_GRIMSON_L_Pin GPIO_PIN_7
#define ENC_B_GRIMSON_L_GPIO_Port GPIOB
#define NEMA_PWM_Pin GPIO_PIN_9
#define NEMA_PWM_GPIO_Port GPIOB
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
