#include "Tasks/control.h"

#include <FreeRTOS.h>
#include <cmsis_os2.h>

#include <stdio.h>
#include <stdbool.h>
#include <arm_math.h>

#include "main.h"

#include "tim.h"

#include "Devices/mpu6050.h"

#define CALIBRATION_SAMPLES 1000
bool g_is_calibrated = false;
static uint16_t s_calibration_count = 0;
static uint32_t __attribute__((section(".ccmram"))) s_sample_count = 0;

#define BALANCE_SPEED 0.0f
#define BALANCE_PITCH 0.0f
float g_gyro_bias[3] = {0};
float g_temperature = 0;
float g_pitch = 0;
float g_gyro_pitch_rate = 0;
float g_gyro_yaw_rate = 0;
float g_angular_velocity[2] = {0};
float g_velocity[2] = {0};

float g_forward = 0;
float g_turn_rate = 0;

arm_pid_instance_f32 g_pitch_angle_pid;
arm_pid_instance_f32 g_pitch_rate_pid;
arm_pid_instance_f32 g_velocity_pid;
arm_pid_instance_f32 g_turn_pid;

bool g_is_balancing = false;
int32_t g_motor_pwm[2] = {0};

static void calibrate_gyro_bias(float *gyro) {
  s_calibration_count++;
  // Discard first 1000 samples
  if (s_calibration_count > 1000) {
    for (size_t i = 0; i < 3; i++) {
      g_gyro_bias[i] += gyro[i];
    }
    if (s_calibration_count >= 1000 + CALIBRATION_SAMPLES) {
      for (size_t i = 0; i < 3; i++) {
        g_gyro_bias[i] /= CALIBRATION_SAMPLES;
      }
      g_is_calibrated = true;

      LEDBlink_SetDelayTime(&g_led_blink, 0, 900);
      LEDBlink_SetDelayTime(&g_led_blink, 1, 100);
      printf("Gyroscope calibration complete: %.2f %.2f %.2f\n", g_gyro_bias[0], g_gyro_bias[1], g_gyro_bias[2]);
    }
  }
}

static void correct_gyro(float *gyro) {
  // Apply calibration
  for (size_t i = 0; i < 3; i++) {
    gyro[i] -= g_gyro_bias[i];
  }
}

// static void complementary_filter_pitch(const float *accel, const float *gyro, float dt) {
//   // Calculate pitch from accelerometer data
//   float accel_pitch;
//   arm_atan2_f32(accel[1], accel[2], &accel_pitch);
//   accel_pitch *= (180.0f / M_PI);

//   // Integrate gyroscope data to get angles
//   g_gyro_pitch_rate = gyro[0];
//   g_gyro_yaw_rate = gyro[2];
//   float pitch = g_pitch + g_gyro_pitch_rate * dt;

//   // Calculate dynamic alpha based on accelerometer magnitude
//   float accel_magnitude;
//   arm_sqrt_f32(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2], &accel_magnitude);
//   float dynamic_alpha = (fabsf(accel_magnitude - 9.81f) < 0.3f) ? 0.96f : 0.98f;

//   // Complementary filter to combine accelerometer and gyroscope data
//   g_pitch = dynamic_alpha * pitch + (1.0f - dynamic_alpha) * accel_pitch;
// }

static void kalman_filter_pitch(const float *accel, const float *gyro, float dt) {
  // Kalman filter state variables (static to persist between function calls)
  static float kalman_angle = 0.0f;      // Estimated pitch angle in degrees
  static float kalman_error_cov = 1.0f;    // Estimate error covariance

  // Process noise variance (tuning parameter)
  const float Q_angle = 0.01f;
  // Measurement noise variance (will be dynamically adjusted)

  // Calculate pitch angle from accelerometer data (in degrees)
  float accel_pitch;
  arm_atan2_f32(accel[1], accel[2], &accel_pitch);
  accel_pitch *= (180.0f / M_PI);

  // Acquire gyroscope rates (assuming gyro[0] is pitch rate, gyro[2] is yaw rate)
  g_gyro_pitch_rate = gyro[0];
  g_gyro_yaw_rate = gyro[2];

  // Prediction step: integrate gyroscope data to predict new angle
  float predicted_angle = kalman_angle + g_gyro_pitch_rate * dt;

  // Calculate accelerometer magnitude to adjust measurement noise dynamically
  float accel_magnitude;
  arm_sqrt_f32(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2], &accel_magnitude);
  // If the accelerometer reading is close to gravity (9.81 m/s^2), use low measurement noise; otherwise, higher noise
  float R_measure = (fabsf(accel_magnitude - 9.81f) < 0.3f) ? 0.0015f : 0.005f;

  // Prediction update: update error covariance with process noise
  float predicted_error_cov = kalman_error_cov + Q_angle * dt;

  // Compute Kalman gain
  float kalman_gain = predicted_error_cov / (predicted_error_cov + R_measure);

  // Update step: update the estimate using the measurement from the accelerometer
  kalman_angle = predicted_angle + kalman_gain * (accel_pitch - predicted_angle);

  // Update the error covariance after measurement update
  kalman_error_cov = (1.0f - kalman_gain) * predicted_error_cov;

  // Update the global pitch variable with the filtered pitch angle
  g_pitch = kalman_angle;
}

#define ENCODER_MULTIPLIER 4.f
#define ENCODER_PRECISION 13.f
#define ENCODER_REDUCTION_RATIO 30.f
#define WHEEL_DIAMETER 67.f
static void calculate_speed(float dt) {
  int16_t left_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
  __HAL_TIM_SET_COUNTER(&htim5, 0);
  int16_t right_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  int32_t encoder[2] = {-left_cnt, -right_cnt};
  for (int i = 0; i < 2; ++i) {
    g_angular_velocity[i] = (float)encoder[i] / (ENCODER_MULTIPLIER * ENCODER_PRECISION * ENCODER_REDUCTION_RATIO) / dt;
    g_velocity[i] = g_angular_velocity[i] * WHEEL_DIAMETER * M_PI;
  }
}

static void process_user_key(void) {
  uint32_t bits = osEventFlagsGet(g_keyEventHandle);
  if (bits & USER_KEY_FLAG_BIT) {
    osEventFlagsClear(g_keyEventHandle, USER_KEY_FLAG_BIT);

    g_is_balancing = !g_is_balancing;

    if (g_is_balancing) {
      LEDBlink_SetDelayTime(&g_led_blink, 0, 700);
      LEDBlink_SetDelayTime(&g_led_blink, 1, 100);
      LEDBlink_SetDelayTime(&g_led_blink, 2, 100);
      LEDBlink_SetDelayTime(&g_led_blink, 3, 100);
    } else {
      LEDBlink_SetDelayTime(&g_led_blink, 0, 900);
      LEDBlink_SetDelayTime(&g_led_blink, 1, 100);
      LEDBlink_SetDelayTime(&g_led_blink, 2, 0);
    }
  }
}

static bool can_run_motor(void) {
  return g_is_calibrated && g_is_balancing && fabsf(g_pitch) < 60.0f;
}

static void get_user_input(void) {
  if (g_uart2_command == 0x5A) { // 'Z'
    g_forward = 0;
    g_turn_rate = 0;
  } else if (g_uart2_command == 0x41) { // 'A'
    g_forward = 80;
    g_turn_rate = 0;
  } else if (g_uart2_command == 0x45) { // 'E'
    g_forward = -80;
    g_turn_rate = 0;
  } else if (g_uart2_command == 0x42 || g_uart2_command == 0x43 || g_uart2_command == 0x44) { // 'B', 'C', 'D'
    g_forward = 0;
    g_turn_rate = -500;
  } else if (g_uart2_command == 0x46 || g_uart2_command == 0x47 || g_uart2_command == 0x48) { // 'F', 'G', 'H'
    g_forward = 0;
    g_turn_rate = 500;
  } else {
    g_forward = 0;
    g_turn_rate = 0;
  }
}

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

static void trun_control(float *pwm) {
  // Calculate yaw rate error
  float rate_error = 0.f - g_gyro_yaw_rate + g_turn_rate;

  // Calculate PWM
  *pwm = arm_pid_f32(&g_turn_pid, rate_error);
}

void Control_Update() {
  static uint32_t last_update_ticks = 0;
  static uint32_t elapsed_ticks = 0;

  uint8_t status = 0;
  MPU6050_Error_t error = MPU6050_GetInterruptStatus(&g_mpu6050, &status);

  if (error == MPU6050_ERROR_OK && status & 0x01) {
    int16_t raw_accel[3];
    int16_t raw_gyro[3];
    float accel[3];
    float gyro[3];
    error = MPU6050_GetRawAccel(&g_mpu6050, raw_accel);
    if (error == MPU6050_ERROR_OK) {
      error = MPU6050_GetRawGyro(&g_mpu6050, raw_gyro);
      if (error == MPU6050_ERROR_OK) {
        for (size_t i = 0; i < 3; i++) {
          accel[i] = raw_accel[i] * 0.0005985504150390625f;
          gyro[i] = raw_gyro[i] * 0.06103515625f;
        }

        // Calculate elapsed time since last update (in seconds)
        uint32_t sys_tick = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t delta_ticks = sys_tick - last_update_ticks;
        float dt = delta_ticks * 1e-6f;
        last_update_ticks = sys_tick;

        if (!g_is_calibrated) {
          calibrate_gyro_bias(gyro);
        } else {
          elapsed_ticks += delta_ticks;

          correct_gyro(gyro);
          // complementary_filter_pitch(accel, gyro, dt);
          kalman_filter_pitch(accel, gyro, dt);
          calculate_speed(dt);
          process_user_key();

          g_forward = 0;
          g_turn_rate = 0;

          if (can_run_motor()) {
            get_user_input();

            float balance_pwm;
            balance_control(&balance_pwm);
            int32_t pwm = (int32_t)(balance_pwm);
            if (pwm < -15000) pwm = -15000;
            if (pwm > 15000) pwm = 15000;
            pwm = pwm * 8399 / 15000;  // Scale PWM to 8399

            float trun_pwm;
            trun_control(&trun_pwm);
            int32_t trun = (int32_t)(trun_pwm);
            if (trun < -8399) trun = -8399;
            if (trun > 8399) trun = 8399;

            g_motor_pwm[0] = pwm - trun;
            g_motor_pwm[1] = pwm + trun;

            int8_t left_dir = (g_motor_pwm[0] == 0) ? 0 : (g_motor_pwm[0] > 0) ? 1 : -1;
            int8_t right_dir = (g_motor_pwm[1] == 0) ? 0 : (g_motor_pwm[1] > 0) ? 1 : -1;
            PWMMotor_Run(&g_motor[0], left_dir, (uint32_t)(g_motor_pwm[0] > 0 ? g_motor_pwm[0] : -g_motor_pwm[0]));
            PWMMotor_Run(&g_motor[1], right_dir, (uint32_t)(g_motor_pwm[1] > 0 ? g_motor_pwm[1] : -g_motor_pwm[1]));
          } else {
            g_motor_pwm[0] = 0;
            g_motor_pwm[1] = 0;

            // Reset PID controllers
            arm_pid_reset_f32(&g_velocity_pid);
            arm_pid_reset_f32(&g_pitch_angle_pid);
            arm_pid_reset_f32(&g_pitch_rate_pid);
            arm_pid_reset_f32(&g_turn_pid);

            // Stop motors
            PWMMotor_Run(&g_motor[0], 0, 0);
            PWMMotor_Run(&g_motor[1], 0, 0);
          }

          s_sample_count++;

          // 1s update
          if (elapsed_ticks > 1000000) {
            // Calculate temperature
            MPU6050_GetTemperature(&g_mpu6050, &g_temperature);
            elapsed_ticks = 0;

            // Print debug information
            // printf("%u samples/s\n", s_sample_count);
            s_sample_count = 0;
          }
        }
      }
    }
  }
}