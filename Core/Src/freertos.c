/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <atomic.h>

#include "adc.h"
#include "usart.h"
#include "i2c.h"
#include "tim.h"

#include "Devices/oled.h"
#include "Devices/led_blink.h"
#include "Devices/mpu6050.h"

#include "Tasks/screen.h"
#include "Tasks/control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/// @brief Format string buffer for printf.
__attribute__((section(".ccmram"))) char g_fmt_string[4 * 1024];

/// @brief LED blink structure.
LEDBlink_t g_led_blink;

/// @brief Global battery voltage.
uint32_t g_battery_voltage = 0;

/// @brief MPU6050 structure.
MPU6050_t g_mpu6050;

/// @brief PWMMotor structure.
PWMMotor_t g_motor[2];

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ledBlinkTask */
osThreadId_t ledBlinkTaskHandle;
const osThreadAttr_t ledBlinkTask_attributes = {
  .name = "ledBlinkTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for batReadTask */
osThreadId_t batReadTaskHandle;
const osThreadAttr_t batReadTask_attributes = {
  .name = "batReadTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for screenTask */
osThreadId_t screenTaskHandle;
const osThreadAttr_t screenTask_attributes = {
  .name = "screenTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for mpu6050Task */
osThreadId_t mpu6050TaskHandle;
const osThreadAttr_t mpu6050Task_attributes = {
  .name = "mpu6050Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for g_ledMutex */
osMutexId_t g_ledMutexHandle;
const osMutexAttr_t g_ledMutex_attributes = {
  .name = "g_ledMutex"
};
/* Definitions for i2cReadySem */
osSemaphoreId_t i2cReadySemHandle;
const osSemaphoreAttr_t i2cReadySem_attributes = {
  .name = "i2cReadySem"
};
/* Definitions for g_initEvent */
osEventFlagsId_t g_initEventHandle;
const osEventFlagsAttr_t g_initEvent_attributes = {
  .name = "g_initEvent"
};
/* Definitions for g_keyEvent */
osEventFlagsId_t g_keyEventHandle;
const osEventFlagsAttr_t g_keyEvent_attributes = {
  .name = "g_keyEvent"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void led_write(uint8_t state);
void led_delay(uint32_t delay_time);
void led_lock(void);
void led_unlock(void);
int32_t mpu6050_read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t size);
int32_t mpu6050_write(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t size);
void set_m0_dir(int8_t dir);
void set_m0_pwm(uint32_t pwm);
void set_m1_dir(int8_t dir);
void set_m1_pwm(uint32_t pwm);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void LEDBlinkTask(void *argument);
void BatReadTask(void *argument);
void ScreenTask(void *argument);
void MPU6050ReadTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of g_ledMutex */
  g_ledMutexHandle = osMutexNew(&g_ledMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of i2cReadySem */
  i2cReadySemHandle = osSemaphoreNew(1, 0, &i2cReadySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ledBlinkTask */
  ledBlinkTaskHandle = osThreadNew(LEDBlinkTask, NULL, &ledBlinkTask_attributes);

  /* creation of batReadTask */
  batReadTaskHandle = osThreadNew(BatReadTask, NULL, &batReadTask_attributes);

  /* creation of screenTask */
  screenTaskHandle = osThreadNew(ScreenTask, NULL, &screenTask_attributes);

  /* creation of mpu6050Task */
  mpu6050TaskHandle = osThreadNew(MPU6050ReadTask, NULL, &mpu6050Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  // Pause the LED blink task.
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of g_initEvent */
  g_initEventHandle = osEventFlagsNew(&g_initEvent_attributes);

  /* creation of g_keyEvent */
  g_keyEventHandle = osEventFlagsNew(&g_keyEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  UNUSED(argument);

  // Initialize OLED.
  OLED_Init();
  OLED_Display_On();
  osEventFlagsSet(g_initEventHandle, OLED_INIT_FLAG_BIT);

  // Initialize LED blink.
  assert(LEDBlink_Init(&g_led_blink, led_write, led_delay, led_lock, led_unlock, 0) == LED_BLINK_ERROR_OK);
  assert(LEDBlink_SetDelayTime(&g_led_blink, 0, 500) == LED_BLINK_ERROR_OK);
  osEventFlagsSet(g_initEventHandle, LED_BLINK_INIT_FLAG_BIT);

  // Initialize battery read.
  osEventFlagsSet(g_initEventHandle, BAT_READ_INIT_FLAG_BIT);

  // Initialize MPU6050.
  int ret = MPU6050_Init(&g_mpu6050, mpu6050_read, mpu6050_write, MPU6050_DEFAULT_ADDRESS);
  if (ret != MPU6050_ERROR_OK) {
    printf("MPU6050_Init failed: %d\n", ret);
  } else {
    uint8_t device_id;
    ret = MPU6050_GetDeviceID(&g_mpu6050, &device_id);
    if (ret == MPU6050_ERROR_OK && device_id == 0x68) {
      MPU6050_SetResetBit(&g_mpu6050, 1);
      osDelay(pdMS_TO_TICKS(100));
      MPU6050_SetResetBit(&g_mpu6050, 0);
      MPU6050_SetClockSource(&g_mpu6050, MPU6050_CLOCK_PLL_YGYRO);
      MPU6050_SetFullScaleGyroRange(&g_mpu6050, MPU6050_GYRO_FS_2000);
      MPU6050_SetFullScaleAccelRange(&g_mpu6050, MPU6050_ACCEL_FS_2);
      MPU6050_SetSleepEnabled(&g_mpu6050, 0);
      MPU6050_SetI2CMasterModeEnabled(&g_mpu6050, 0);
      MPU6050_SetI2CByPassEnabled(&g_mpu6050, 0);
      MPU6050_SetDLPF(&g_mpu6050, MPU6050_DLPF_BW_42);
      MPU6050_SetSampleRateDivider(&g_mpu6050, 0);
      MPU6050_SetTemperatureFIFOEnable(&g_mpu6050, 0);
      MPU6050_SetTemperatureEnable(&g_mpu6050, 1);
      MPU6050_SetAccelAndGyroStandby(&g_mpu6050, 0);
      MPU6050_SetAccelAndGyroFIFOEnable(&g_mpu6050, 0);
      MPU6050_SetInterruptLevel(&g_mpu6050, 0);
      MPU6050_SetInterruptOpenDrain(&g_mpu6050, 1);
      MPU6050_SetInterruptLatchEnable(&g_mpu6050, 0);
      MPU6050_SetInterruptClearOnRead(&g_mpu6050, 1);
      MPU6050_SetFSyncInterruptEnabled(&g_mpu6050, 0);
      MPU6050_ResetFIFOBuffer(&g_mpu6050);
      MPU6050_EnableFIFOBuffer(&g_mpu6050, 0);
      MPU6050_SetInterruptEnable(&g_mpu6050, 1);

      osEventFlagsSet(g_initEventHandle, MPU6050_INIT_FLAG_BIT);
    } else {
      printf("MPU6050_GetDeviceID failed: %d\n", ret);
    }
  }

  // Initialize PWM motors.
  assert(PWMMotor_Init(&g_motor[0], set_m0_dir, set_m0_pwm) == PWM_MOTOR_ERROR_OK);
  assert(PWMMotor_Init(&g_motor[1], set_m1_dir, set_m1_pwm) == PWM_MOTOR_ERROR_OK);
  PWMMotor_Run(&g_motor[0], 0, 0);
  PWMMotor_Run(&g_motor[1], 0, 0);

  /* Infinite loop */
  for(;;)
  {
    osDelay(pdMS_TO_TICKS(1000));
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_LEDBlinkTask */
/**
* @brief Function implementing the ledBlinkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEDBlinkTask */
void LEDBlinkTask(void *argument)
{
  /* USER CODE BEGIN LEDBlinkTask */
  UNUSED(argument);
  osEventFlagsWait(g_initEventHandle, LED_BLINK_INIT_FLAG_BIT, osFlagsWaitAny | osFlagsNoClear, osWaitForever);

  /* Infinite loop */
  LEDBlink_Loop(&g_led_blink);
  /* USER CODE END LEDBlinkTask */
}

/* USER CODE BEGIN Header_BatReadTask */
/**
* @brief Function implementing the batReadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BatReadTask */
void BatReadTask(void *argument)
{
  /* USER CODE BEGIN BatReadTask */
  UNUSED(argument);
  #define WINDOW_SIZE 20
  static __attribute__((section(".ccmram"))) float buffer[WINDOW_SIZE];
  static __attribute__((section(".ccmram"))) uint8_t index;
  static __attribute__((section(".ccmram"))) float sum;
  static __attribute__((section(".ccmram"))) float battery_voltage;

  osEventFlagsWait(g_initEventHandle, BAT_READ_INIT_FLAG_BIT, osFlagsWaitAny | osFlagsNoClear, osWaitForever);

  index = 0;
  sum = 0;

  // Fill the buffer with initial values.
  for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint32_t raw_value = HAL_ADC_GetValue(&hadc1);
    float b = (float)(raw_value * 330 * 11) / 409.6f; // mV
    buffer[i] = b;
    sum += b;
    osDelay(pdMS_TO_TICKS(50));
  }

  battery_voltage = sum / WINDOW_SIZE;

  /* Infinite loop */
  for (;;) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint32_t raw_value = HAL_ADC_GetValue(&hadc1);
    float b = (float)(raw_value * 330 * 11) / 409.6f; // mV

    sum -= buffer[index];
    buffer[index] = b;
    sum += b;

    index = (index + 1) % WINDOW_SIZE;

    battery_voltage = sum / WINDOW_SIZE;

    Atomic_CompareAndSwap_u32(&g_battery_voltage, (uint32_t)battery_voltage, g_battery_voltage);

    osDelay(pdMS_TO_TICKS(50));
  }
  /* USER CODE END BatReadTask */
}

/* USER CODE BEGIN Header_ScreenTask */
/**
* @brief Function implementing the screenTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ScreenTask */
void ScreenTask(void *argument)
{
  /* USER CODE BEGIN ScreenTask */
  UNUSED(argument);

  osEventFlagsWait(g_initEventHandle, OLED_INIT_FLAG_BIT, osFlagsWaitAny | osFlagsNoClear, osWaitForever);

  /* Infinite loop */
  for (;;) {
    Screen_Update();
    osDelay(pdMS_TO_TICKS(100));
  }
  /* USER CODE END ScreenTask */
}

/* USER CODE BEGIN Header_MPU6050ReadTask */
/**
* @brief Function implementing the mpu6050Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MPU6050ReadTask */
void MPU6050ReadTask(void *argument)
{
  /* USER CODE BEGIN MPU6050ReadTask */
  UNUSED(argument);

  g_velocity_pid.Kp = 0.05f;
  g_velocity_pid.Ki = 0.00002f;
  g_velocity_pid.Kd = 0.0045f;
  arm_pid_init_f32(&g_velocity_pid, 1);
  g_pitch_angle_pid.Kp = 120.f;
  g_pitch_angle_pid.Ki = 4.f;
  g_pitch_angle_pid.Kd = 50.f;
  arm_pid_init_f32(&g_pitch_angle_pid, 1);
  g_pitch_rate_pid.Kp = 1.f;
  g_pitch_rate_pid.Ki = 0.f;
  g_pitch_rate_pid.Kd = 8.f;
  arm_pid_init_f32(&g_pitch_rate_pid, 1);
  g_turn_pid.Kp = 3.f;
  g_turn_pid.Ki = 0.f;
  g_turn_pid.Kd = 1.f;
  arm_pid_init_f32(&g_turn_pid, 1);

  osEventFlagsWait(g_initEventHandle, MPU6050_INIT_FLAG_BIT, osFlagsWaitAny | osFlagsNoClear, osWaitForever);

  /* Infinite loop */
  for(;;)
  {
    uint32_t flags = osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
    if ((flags & 0x80000000) == 0) {
      Control_Update();
    }
  }
  /* USER CODE END MPU6050ReadTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void led_write(uint8_t state) {
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, state);
}

void led_delay(uint32_t delay_time) {
  osDelay(pdMS_TO_TICKS(delay_time));
}

void led_lock(void) {
  osMutexAcquire(g_ledMutexHandle, osWaitForever);
}

void led_unlock(void) {
  osMutexRelease(g_ledMutexHandle);
}

int32_t mpu6050_read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t size) {
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read_IT(&hi2c2, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, size);
  if (status != HAL_OK) {
    printf("mpu6050_read failed: %d\n", status);
  }
  osStatus_t wait_status = osSemaphoreAcquire(i2cReadySemHandle, pdMS_TO_TICKS(1000));
  if (wait_status != osOK) {
    printf("mpu6050_read timeout: %d\n", wait_status);
    return wait_status;
  }
  return status;
}

int32_t mpu6050_write(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t size) {
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write_IT(&hi2c2, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, size);
  if (status != HAL_OK) {
    printf("mpu6050_write failed: %d\n", status);
  }
  osStatus_t wait_status = osSemaphoreAcquire(i2cReadySemHandle, pdMS_TO_TICKS(1000));
  if (wait_status != osOK) {
    printf("mpu6050_write timeout: %d\n", wait_status);
    return wait_status;
  }
  return status;
}

void set_m0_dir(int8_t dir) {
  switch (dir) {
    case 1:
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
      break;
    case -1:
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
      break;
    default:
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
      break;
  }
}

void set_m0_pwm(uint32_t pwm) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
}

void set_m1_dir(int8_t dir) {
  switch (dir) {
    case -1:
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
      break;
    case 1:
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
      break;
    default:
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
      break;
  }
}

void set_m1_pwm(uint32_t pwm) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C2)
    osSemaphoreRelease(i2cReadySemHandle);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C2)
    osSemaphoreRelease(i2cReadySemHandle);
}

static __attribute__((section(".ccmram"))) bool s_is_key_pressed = false;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == MPU6050_INT_Pin) {
    osThreadFlagsSet(mpu6050TaskHandle, 0x01);
  }
  if (GPIO_Pin == USER_KEY_Pin) {
    s_is_key_pressed = true;

    EXTI->IMR &= ~(USER_KEY_Pin);
    HAL_TIM_Base_Start_IT(&htim6);
  }
}

void HAL_TIM6_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  UNUSED(htim);
  if (s_is_key_pressed) {
    if (HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_RESET) {
      osEventFlagsSet(g_keyEventHandle, USER_KEY_FLAG_BIT);
    }

    s_is_key_pressed = false;

    __HAL_GPIO_EXTI_CLEAR_IT(USER_KEY_Pin);
    EXTI->IMR |= USER_KEY_Pin;

    HAL_TIM_Base_Stop_IT(&htim6);
  }
}

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

#ifdef __GNUC__
  #define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
  #define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

GETCHAR_PROTOTYPE
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END Application */

