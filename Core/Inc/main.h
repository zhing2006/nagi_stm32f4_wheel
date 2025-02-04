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
#include <stdbool.h>

#include <arm_math.h>

#include "Devices/led_blink.h"
#include "Devices/mpu6050.h"
#include "Devices/pwm_motor.h"
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_CLK_Pin GPIO_PIN_13
#define OLED_CLK_GPIO_Port GPIOC
#define OLED_DIN_Pin GPIO_PIN_14
#define OLED_DIN_GPIO_Port GPIOC
#define OLED_RST_Pin GPIO_PIN_15
#define OLED_RST_GPIO_Port GPIOC
#define OLED_DC_Pin GPIO_PIN_0
#define OLED_DC_GPIO_Port GPIOC
#define DISTANCE_TRIG_Pin GPIO_PIN_2
#define DISTANCE_TRIG_GPIO_Port GPIOC
#define M1_ENCODER_A_Pin GPIO_PIN_0
#define M1_ENCODER_A_GPIO_Port GPIOA
#define M1_ENCODER_B_Pin GPIO_PIN_1
#define M1_ENCODER_B_GPIO_Port GPIOA
#define DISTANCE_CAP_Pin GPIO_PIN_0
#define DISTANCE_CAP_GPIO_Port GPIOB
#define USER_LED_Pin GPIO_PIN_13
#define USER_LED_GPIO_Port GPIOB
#define USER_KEY_Pin GPIO_PIN_14
#define USER_KEY_GPIO_Port GPIOB
#define USER_KEY_EXTI_IRQn EXTI15_10_IRQn
#define MPU6050_INT_Pin GPIO_PIN_15
#define MPU6050_INT_GPIO_Port GPIOB
#define MPU6050_INT_EXTI_IRQn EXTI15_10_IRQn
#define PWMB_Pin GPIO_PIN_8
#define PWMB_GPIO_Port GPIOA
#define PWMA_Pin GPIO_PIN_11
#define PWMA_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_12
#define BIN2_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_15
#define AIN2_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_5
#define AIN1_GPIO_Port GPIOB
#define M0_ENCODER_A_Pin GPIO_PIN_6
#define M0_ENCODER_A_GPIO_Port GPIOB
#define M0_ENCODER_B_Pin GPIO_PIN_7
#define M0_ENCODER_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define OLED_INIT_FLAG_BIT (1 << 0)
#define LED_BLINK_INIT_FLAG_BIT (1 << 1)
#define BAT_READ_INIT_FLAG_BIT (1 << 2)
#define MPU6050_INIT_FLAG_BIT (1 << 3)

#define USER_KEY_FLAG_BIT (1 << 0)

typedef void *osEventFlagsId_t;

extern osEventFlagsId_t g_initEventHandle;
extern osEventFlagsId_t g_keyEventHandle;
extern __attribute__((section(".ccmram"))) char g_fmt_string[4 * 1024];
extern uint32_t g_battery_voltage;
extern bool g_is_calibrated;
extern float g_gyro_bias[3];
extern float g_temperature;
extern float g_pitch;
extern float g_gyro_pitch_rate;
extern float g_angular_velocity[2];
extern float g_velocity[2];
extern bool g_is_balancing;
extern arm_pid_instance_f32 g_pitch_angle_pid;
extern arm_pid_instance_f32 g_pitch_rate_pid;
extern arm_pid_instance_f32 g_velocity_pid;
extern arm_pid_instance_f32 g_turn_pid;
extern int32_t g_motor_pwm[2];
extern LEDBlink_t g_led_blink;
extern MPU6050_t g_mpu6050;
extern PWMMotor_t g_motor[2];
extern uint8_t g_uart2_command;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
