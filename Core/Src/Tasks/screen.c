#include "Tasks/screen.h"

#include <FreeRTOS.h>
#include "cmsis_os.h"

#include <stdint.h>
#include <stdio.h>
#include <atomic.h>

#include <tim.h>

#include "main.h"

#include "Devices/oled.h"
#include "Devices/mpu6050.h"

void Screen_Update(void) {
  // Show running flag.
  if (g_is_balancing) {
    OLED_ShowString(0, 0, "Balancing");
  } else {
    OLED_ShowString(0, 0, "Stopped  ");
  }

  // Show gyro and accel information.
  uint32_t init_bits = osEventFlagsGet(g_initEventHandle);
  if (init_bits & MPU6050_INIT_FLAG_BIT) {
    sprintf(g_fmt_string, "T:%.1fC", g_temperature);
    OLED_ShowString(64, 64 - 12, g_fmt_string);
    sprintf(g_fmt_string, "P:%.1f  ", g_pitch);
    OLED_ShowString(0, 12, g_fmt_string);
    sprintf(g_fmt_string, "G:%.1f  ", g_gyro_pitch_rate);
    OLED_ShowString(64, 12, g_fmt_string);
  }

  // Show left & right encoder values.
  sprintf(g_fmt_string, "L:%.1f   ", g_velocity[0]);
  OLED_ShowString(0, 24, g_fmt_string);
  sprintf(g_fmt_string, "R:%.1f   ", g_velocity[1]);
  OLED_ShowString(64, 24, g_fmt_string);

  // Show motor PWM values.
  sprintf(g_fmt_string, "0:%05ld ", g_motor_pwm[0]);
  OLED_ShowString(0, 36, g_fmt_string);
  sprintf(g_fmt_string, "1:%05ld ", g_motor_pwm[1]);
  OLED_ShowString(64, 36, g_fmt_string);

  // Show battery voltage.
  uint32_t battery_voltage = Atomic_OR_u32(&g_battery_voltage, 0);
  sprintf(g_fmt_string, "B:%.2fv", battery_voltage / 1000.f);
  OLED_ShowString(0, 64 - 12, g_fmt_string);
  OLED_Refresh_Gram();
}