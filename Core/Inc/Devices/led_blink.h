#ifndef __LED_BLINK_H__
#define __LED_BLINK_H__

#include <stddef.h>
#include <stdint.h>

#define LED_BLINK_MAX_DELAY_TIMES (5)

/// @brief LED blink error code.
typedef enum {
  LED_BLINK_ERROR_OK = 0,
  LED_BLINK_ERROR_HANDLE_NULL,
  LED_BLINK_ERROR_WRITE_FN_NULL,
  LED_BLINK_ERROR_DELAY_FN_NULL,
  LED_BLINK_ERROR_INDEX_OUT_OF_RANGE,
} LEDBlinkError_t;

/// @brief LED blink structure.
typedef struct {
  /// @brief Write GPIO function.
  void (*write_fn)(uint8_t);
  /// @brief Delay function.
  void (*delay_fn)(uint32_t);
  /// @brief Lock function.
  void (*lock_fn)(void);
  /// @brief Unlock function.
  void (*unlock_fn)(void);

  /// @brief First state.
  uint8_t first_state;
  /// @brief Delay times in milliseconds.
  uint32_t delay_times[LED_BLINK_MAX_DELAY_TIMES];
  /// @brief Number of delay times.
  uint8_t delay_times_count;
} LEDBlink_t;

/// @brief Initialize LED blink.
/// @param led_blink LED blink structure.
/// @param write_fn Write GPIO function.
/// @param delay_fn Delay function.
/// @param lock_fn Lock function. If it is NULL, it will not be thread safe.
/// @param unlock_fn Unlock function. If it is NULL, it will not be thread safe.
/// @param first_state First state.
/// @return 0 if success, otherwise return -1.
LEDBlinkError_t LEDBlink_Init(
  LEDBlink_t *led_blink,
  void (*write_fn)(uint8_t),
  void (*delay_fn)(uint32_t),
  void (*lock_fn)(void),
  void (*unlock_fn)(void),
  uint8_t first_state
);
/// @brief Set delay time.
/// @param led_blink LED blink structure.
/// @param index Index of delay time.
/// @param delay_time Delay time in milliseconds.
/// @return 0 if success, otherwise return -1.
LEDBlinkError_t LEDBlink_SetDelayTime(LEDBlink_t *led_blink, uint8_t index, uint32_t delay_time);
/// @brief LED blink loop.
/// @note This function will never return. You can use it in FreeRTOS task.
/// @param led_blink LED blink structure.
void LEDBlink_Loop(LEDBlink_t *led_blink);

#endif // __LED_BLINK_H__