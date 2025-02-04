#ifndef __PWM_MOTOR_H__
#define __PWM_MOTOR_H__

#include <stddef.h>
#include <stdint.h>

/// @brief The PWM motor error codes.
typedef enum {
  PWM_MOTOR_ERROR_OK = 0,
  PWM_MOTOR_ERROR_HANDLE_NULL,
  PWM_MOTOR_ERROR_DIR_FN_NULL,
  PWM_MOTOR_ERROR_PWM_FN_NULL,
} PWMMotor_Error_t;

/// @brief The PWM motor structure.
typedef struct {
  /// @brief Direction function.
  void (*dir_fn)(int8_t dir);
  /// @brief PWM function.
  void (*pwm_fn)(uint32_t pwm);
} PWMMotor_t;

/// @brief Initialize PWM motor.
/// @param motor The PWM motor structure.
/// @param dir_fn The direction function.
/// @param pwm_fn The PWM function.
/// @return The error code.
PWMMotor_Error_t PWMMotor_Init(
  PWMMotor_t *motor,
  void (*dir_fn)(int8_t dir),
  void (*pwm_fn)(uint32_t pwm)
);

/// @brief Run PWM motor.
/// @param motor The PWM motor structure.
/// @param dir The direction.
/// @param pwm The PWM value.
PWMMotor_Error_t PWMMotor_Run(PWMMotor_t *motor, int8_t dir, uint32_t pwm);

#endif // __PWM_MOTOR_H__