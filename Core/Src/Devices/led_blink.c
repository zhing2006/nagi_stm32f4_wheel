#include "Devices/led_blink.h"

#include "gpio.h"

LEDBlinkError_t LEDBlink_Init(
  LEDBlink_t *led_blink,
  void (*write_fn)(uint8_t),
  void (*delay_fn)(uint32_t),
  void (*lock_fn)(void),
  void (*unlock_fn)(void),
  uint8_t first_state)
{
  if (led_blink == NULL) {
    return LED_BLINK_ERROR_HANDLE_NULL;
  }
  if (write_fn == NULL) {
    return LED_BLINK_ERROR_WRITE_FN_NULL;
  }
  if (delay_fn == NULL) {
    return LED_BLINK_ERROR_DELAY_FN_NULL;
  }

  led_blink->write_fn = write_fn;
  led_blink->delay_fn = delay_fn;
  led_blink->lock_fn = lock_fn;
  led_blink->unlock_fn = unlock_fn;
  led_blink->first_state = first_state;
  led_blink->delay_times_count = 1;
  led_blink->delay_times[0] = 500;

  return LED_BLINK_ERROR_OK;
}

LEDBlinkError_t LEDBlink_SetDelayTime(LEDBlink_t *led_blink, uint8_t index, uint32_t delay_time) {
  if (led_blink == NULL) {
    return LED_BLINK_ERROR_HANDLE_NULL;
  }
  if (index >= LED_BLINK_MAX_DELAY_TIMES) {
    return LED_BLINK_ERROR_INDEX_OUT_OF_RANGE;
  }

  if (led_blink->lock_fn != NULL) {
    led_blink->lock_fn();
  }

  if (delay_time > 0) {
    led_blink->delay_times[index] = delay_time;
    if (index >= led_blink->delay_times_count) {
      for (uint8_t i = led_blink->delay_times_count; i < index; i++) {
        led_blink->delay_times[i] = 0;
      }
      led_blink->delay_times_count = index + 1;
    }
  } else {
    for (uint8_t i = index; i < led_blink->delay_times_count; i++) {
      led_blink->delay_times[i] = 0;
    }
    led_blink->delay_times_count = index;
  }

  if (led_blink->unlock_fn != NULL) {
    led_blink->unlock_fn();
  }

  return LED_BLINK_ERROR_OK;
}

void LEDBlink_Loop(LEDBlink_t *led_blink) {
  uint8_t state = led_blink->first_state;
  uint8_t delay_index = 0;
  for (;;) {
    if (led_blink->lock_fn != NULL) {
      led_blink->lock_fn();
    }

    uint32_t delay_time = led_blink->delay_times[delay_index];
    uint8_t delay_times_count = led_blink->delay_times_count;

    if (led_blink->unlock_fn != NULL) {
      led_blink->unlock_fn();
    }

    led_blink->write_fn(state);
    led_blink->delay_fn(delay_time);
    state = !state;
    delay_index = (delay_index + 1) % delay_times_count;
  }
}