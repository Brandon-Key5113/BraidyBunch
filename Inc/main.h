/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32h7xx_hal.h"

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
#define SOL0_DIR_Pin GPIO_PIN_6
#define SOL0_DIR_GPIO_Port GPIOE
#define User_Button_Pin GPIO_PIN_13
#define User_Button_GPIO_Port GPIOC
#define User_Button_EXTI_IRQn EXTI15_10_IRQn
#define RST_Pin GPIO_PIN_3
#define RST_GPIO_Port GPIOF
#define IntTest_Pin GPIO_PIN_0
#define IntTest_GPIO_Port GPIOG
#define IntTest_EXTI_IRQn EXTI0_IRQn
#define DIR3_Pin GPIO_PIN_11
#define DIR3_GPIO_Port GPIOE
#define SOL0_EN_Pin GPIO_PIN_12
#define SOL0_EN_GPIO_Port GPIOE
#define DIR2_Pin GPIO_PIN_14
#define DIR2_GPIO_Port GPIOE
#define SOL1_EN_Pin GPIO_PIN_15
#define SOL1_EN_GPIO_Port GPIOE
#define SOL1_DIR_Pin GPIO_PIN_10
#define SOL1_DIR_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOD
#define ChipSel_Pin GPIO_PIN_14
#define ChipSel_GPIO_Port GPIOD
#define PWM1_Pin GPIO_PIN_15
#define PWM1_GPIO_Port GPIOD
#define USB_OTG_FS_OVCR_Pin GPIO_PIN_7
#define USB_OTG_FS_OVCR_GPIO_Port GPIOG
#define USB_OTG_FS_OVCR_EXTI_IRQn EXTI9_5_IRQn
#define DIR1_Pin GPIO_PIN_12
#define DIR1_GPIO_Port GPIOG
#define Flag_Pin GPIO_PIN_14
#define Flag_GPIO_Port GPIOG
#define Flag_EXTI_IRQn EXTI15_10_IRQn
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
