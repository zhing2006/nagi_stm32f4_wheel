# nagi-stm32f4-wheel

[![License](https://img.shields.io/badge/License-GPL3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0.en.html)

[English](README.md) | [中文](README_CN.md)

# 介绍
本项目使用了[WHEELTEC](https://www.wheeltec.net/)在2017年发售的基于STM32F405RGT6的迷你两轮平衡小车的硬件，全部重新实现了软件部分。

配套硬件为STM32F4主控板（由于时间久远，原理图已丢失，这里只附上引脚定义图），

![stm32f4_pin](images/stm32f405_config.png)

电机驱动模块为TB6612FNG，6轴陀螺仪和加速度计为MPU6050。

小车控制分为了主要两个函数：

```c
static void balance_control(float *pwm) {
  // Calculate actual speed
  float actual_speed = (g_velocity[0] + g_velocity[1]) * 0.5f;
  // Calculate speed error
  float speed_error = BALANCE_SPEED - actual_speed + g_forward;

  // Calculate expected angle
  float expected_angle = -arm_pid_f32(&g_velocity_pid, speed_error);

  // Calculate angle error
  float angle_error = BALANCE_PITCH + expected_angle - g_pitch;

  // Calculate expected rate
  float expected_rate = arm_pid_f32(&g_pitch_angle_pid, angle_error);

  // Calculate rate error
  float rate_error = expected_rate - (g_gyro_pitch_rate);

  // Calculate PWM
  *pwm = arm_pid_f32(&g_pitch_rate_pid, rate_error);
}
```

平衡控制，采用了三环PID控制，分别是速度环（位置环）、俯仰角度环、俯仰角速度环。直接输出驱动电机的PWM信号。

```c
static void trun_control(float *pwm) {
  // Calculate yaw rate error
  float rate_error = 0.f - g_gyro_yaw_rate + g_turn_rate;

  // Calculate PWM
  *pwm = arm_pid_f32(&g_turn_pid, rate_error);
}
```

转向控制就很简单，直接使用了一环PID。

最后在通过简单的加减与平衡控制输出的PWM结合。

```c
g_motor_pwm[0] = pwm - trun;
g_motor_pwm[1] = pwm + trun;
```

## 开发环境

使用STM32CubeMX打开`nagi_stm32f4_wheel.ioc`，导出CMake工程。再使用VSCode开发（需要安装STM32的扩展）。