#include "Devices/pwm_motor.h"

PWMMotor_Error_t PWMMotor_Init(
  PWMMotor_t *motor,
  void (*dir_fn)(int8_t dir),
  void (*pwm_fn)(uint32_t pwm)
) {
  if (dir_fn == NULL) {
    return PWM_MOTOR_ERROR_DIR_FN_NULL;
  }
  if (pwm_fn == NULL) {
    return PWM_MOTOR_ERROR_PWM_FN_NULL;
  }

  motor->dir_fn = dir_fn;
  motor->pwm_fn = pwm_fn;

  return PWM_MOTOR_ERROR_OK;
}

PWMMotor_Error_t PWMMotor_Run(PWMMotor_t *motor, int8_t dir, uint32_t pwm) {
  if (motor == NULL) {
    return PWM_MOTOR_ERROR_HANDLE_NULL;
  }
  if (motor->dir_fn == NULL) {
    return PWM_MOTOR_ERROR_DIR_FN_NULL;
  }
  if (motor->pwm_fn == NULL) {
    return PWM_MOTOR_ERROR_PWM_FN_NULL;
  }

  motor->dir_fn(dir);
  motor->pwm_fn(pwm);

  return PWM_MOTOR_ERROR_OK;
}