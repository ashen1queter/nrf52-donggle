#include "keyboard.h"
//#include "DRV2605L.h"
#include "hid.h"
//#include <class/hid/hid.h>
#include <stdlib.h>
#include "main.h"
#include "config.h"


struct key keyboard_keys[3] = {0};
struct state flex_keys[3] = {0};


struct user_config keyboard_user_config = {
    //.reverse_magnet_pole = DEFAULT_REVERSE_MAGNET_POLE,
    .trigger_offset = {0},
    .reset_threshold = DEFAULT_RESET_THRESHOLD,
    .rapid_trigger_offset = {0},
    .screaming_velocity_trigger = DEFAULT_SCREAMING_VELOCITY_TRIGGER,
    .tap_timeout = DEFAULT_TAP_TIMEOUT,
    .keymaps = {
        [_BASE_LAYER] = {
            {APP_USBD_HID_KBD_A, APP_USBD_HID_KBD_A, APP_USBD_HID_KBD_D}
        },
        [_TAP_LAYER] = {
            {____, APP_USBD_HID_KBD_S, ____}
        },
    }
};


uint32_t keyboard_last_cycle_duration = 0;

static uint8_t key_triggered = 0;

/**
uint8_t get_bitmask_for_modifier(uint8_t keycode) {
  switch (keycode) {
  case HID_KEY_CONTROL_LEFT:
    return 0b00000001;
  case HID_KEY_SHIFT_LEFT:
    return 0b00000010;
  case HID_KEY_ALT_LEFT:
    return 0b00000100;
  case HID_KEY_GUI_LEFT:
    return 0b00001000;
  case HID_KEY_CONTROL_RIGHT:
    return 0b00010000;
  case HID_KEY_SHIFT_RIGHT:
    return 0b00100000;
  case HID_KEY_ALT_RIGHT:
    return 0b01000000;
  case HID_KEY_GUI_RIGHT:
    return 0b10000000;
  default:
    return 0b00000000;
  }
}
*/


uint16_t get_usage_consumer_control(uint16_t value) {
  if (value > 0xFF) {
    return value & 0b0111111111111111;
  } else {
    return 0;
  }
}

void init_key(uint8_t i, uint8_t row, uint8_t column) {
  struct key *key = &keyboard_keys[i];

  keyboard_user_config.rapid_trigger_offset[i] = cus_actuation[i];
  keyboard_user_config.trigger_offset[i] = cus_actuation[i];

  key->is_enabled = 1;
  key->is_idle = 0;
  key->row = row;
  key->column = column;

  key->calibration.cycles_count = 0;
  key->calibration.idle_value = IDLE_VALUE_APPROX;
  key->calibration.max_distance = MAX_DISTANCE_APPROX;

  key->actuation.status = STATUS_RESET;
  key->actuation.trigger_offset = keyboard_user_config.trigger_offset[i];
  key->actuation.reset_offset = keyboard_user_config.trigger_offset[i] - keyboard_user_config.reset_threshold;
  key->actuation.rapid_trigger_offset = keyboard_user_config.rapid_trigger_offset[i];
  key->actuation.is_continuous_rapid_trigger_enabled = 0;

  for (uint8_t i = 0; i < LAYERS_COUNT; i++) {
    if (keyboard_user_config.keymaps[i][row][column] != ____) {
      uint16_t usage_consumer_control = get_usage_consumer_control(keyboard_user_config.keymaps[i][row][column]);
      if (usage_consumer_control) {
        key->layers[i].type = KEY_TYPE_CONSUMER_CONTROL;
        key->layers[i].value = usage_consumer_control;
      } else {
        /**uint8_t bitmask = get_bitmask_for_modifier(keyboard_user_config.keymaps[i][row][column]);
        if (bitmask) {
          key->layers[i].type = KEY_TYPE_MODIFIER;
          key->layers[i].value = bitmask;}
          */
          key->layers[i].type = KEY_TYPE_NORMAL;
          key->layers[i].value = keyboard_user_config.keymaps[i][row][column];
        }
    }
  }
}

uint8_t update_key_state(struct key *key, struct state *state) {

  // Get a reading
  //state.value = keyboard_user_config.reverse_magnet_pole ? 4500 - keyboard_read_adc() : keyboard_read_adc();

  if (key->calibration.cycles_count < CALIBRATION_CYCLES) {
    // Calibrate idle value
    float delta = 0.6;
    key->calibration.idle_value = (1 - delta) * state->value + delta * key->calibration.idle_value;
    key->calibration.cycles_count++;

    return 0;
  }

  // Calibrate idle value
  if (state->value < key->calibration.idle_value) {
    // opti possible sur float
    float delta = 0.8;
    key->calibration.idle_value = (1 - delta) * state->value + delta * key->calibration.idle_value;
    state->value = key->calibration.idle_value;
  }

  // Do nothing if key is idle
  if (key->state.distance == 0 && state->value <= key->calibration.idle_value + IDLE_VALUE_OFFSET) {
    if (key->idle_counter >= IDLE_CYCLES_UNTIL_SLEEP) {
      key->is_idle = 1;
      return 0;
    }
    key->idle_counter++;
  }

  // Get distance from top
  if (state->value <= key->calibration.idle_value + IDLE_VALUE_OFFSET) {
    state->distance = 0;
    key->actuation.direction_changed_point = 0;
  } else {
    state->distance = state->value - key->calibration.idle_value + IDLE_VALUE_OFFSET;
    key->is_idle = 0;
    key->idle_counter = 0;
  }

  // Calibrate max distance value
  if (state->distance > key->calibration.max_distance) {
    key->calibration.max_distance = state->distance;
  }

  // Limit max distance
  if (state->distance >= key->calibration.max_distance - MAX_DISTANCE_OFFSET) {
    state->distance = key->calibration.max_distance;
  }

  // Map distance in percentages
  state->distance_8bits = (state->distance * 0xff) / key->calibration.max_distance;

  float delta = 0.8;
  state->filtered_distance = (1 - delta) * state->distance_8bits + delta * key->state.filtered_distance;
  state->filtered_distance_8bits = state->filtered_distance;

  // Update velocity
  state->velocity = state->filtered_distance_8bits - key->state.filtered_distance_8bits;

  // Update direction
  if (key->state.velocity > 0 && state->velocity > 0 && key->actuation.direction != GOING_DOWN) {
    key->actuation.direction = GOING_DOWN;
    if (key->actuation.direction_changed_point != 0) {
      key->actuation.direction_changed_point = key->state.filtered_distance_8bits;
    }
  } else if (key->state.velocity < 0 && state->velocity < 0 && key->actuation.direction != GOING_UP) {
    key->actuation.direction = GOING_UP;
    if (key->actuation.direction_changed_point != 255) {
      key->actuation.direction_changed_point = key->state.filtered_distance_8bits;
    }
  }

  key->state = *state;
  return 1;
}

void update_key_actuation(struct key *key) {
  /**
   * https://www.youtube.com/watch?v=_Sl-T6iQr8U&t
   *
   *                          -----   |--------|                           -
   *                            |     |        |                           |
   *    is_before_reset_offset  |     |        |                           |
   *                            |     |        |                           | Continuous rapid trigger domain (deactivated when full_reset)
   *                          -----   | ------ | <- reset_offset           |
   *                            |     |        |                           |
   *                          -----   | ------ | <- trigger_offset         -
   *                            |     |        |                           |
   *                            |     |        |                           |
   *   is_after_trigger_offset  |     |        |                           | Rapid trigger domain
   *                            |     |        |                           |
   *                            |     |        |                           |
   *                          -----   |--------|                           -
   *
   */

  // if rapid trigger enable, move trigger and reset offsets according to the distance taht began the trigger

  uint32_t now = keyboard_get_time();
  uint8_t is_after_trigger_offset = key->state.distance_8bits > key->actuation.trigger_offset;
  uint8_t is_before_reset_offset = key->state.distance_8bits < key->actuation.reset_offset;
  uint8_t has_rapid_trigger = key->actuation.rapid_trigger_offset != 0;
  uint8_t is_after_rapid_trigger_offset = key->state.distance_8bits > key->actuation.direction_changed_point - key->actuation.rapid_trigger_offset + keyboard_user_config.reset_threshold;
  uint8_t is_before_rapid_reset_offset = key->state.distance_8bits < key->actuation.direction_changed_point - key->actuation.rapid_trigger_offset;

  switch (key->actuation.status) {

  case STATUS_RESET:
    // if reset, can be triggered or tap
    if (is_after_trigger_offset) {
      if (key->layers[_TAP_LAYER].value) {
        key->actuation.status = STATUS_MIGHT_BE_TAP;
        // key_triggered = 1;
      } else {
        key->actuation.status = STATUS_TRIGGERED;
        key_triggered = 1;
        hid_press_key(key, _BASE_LAYER);
      }
      key->actuation.triggered_at = now;
    }
    break;

  case STATUS_RAPID_TRIGGER_RESET:
    if (!has_rapid_trigger) {
      key->actuation.status = STATUS_RESET;
      break;
    }
    // if reset, can be triggered or tap
    if (is_after_trigger_offset && key->actuation.direction == GOING_DOWN && is_after_rapid_trigger_offset) {
      if (key->layers[_TAP_LAYER].value) {
        key->actuation.status = STATUS_MIGHT_BE_TAP;
        key_triggered = 1;
      } else {
        key->actuation.status = STATUS_TRIGGERED;
        key_triggered = 1;
        hid_press_key(key, _BASE_LAYER);
      }
      key->actuation.triggered_at = now;
    } else if (is_before_reset_offset) {
      key->actuation.status = STATUS_RESET;
    }
    break;

  case STATUS_TAP:
    // if tap, can be reset
    key->actuation.status = STATUS_RESET;
    hid_release_key(key, _TAP_LAYER);
    break;

  case STATUS_TRIGGERED:
    // if triggered, can be reset
    if (is_before_reset_offset) {
      key->actuation.status = STATUS_RESET;
      hid_release_key(key, _BASE_LAYER);
    } else if (has_rapid_trigger && key->actuation.direction == GOING_UP && is_before_rapid_reset_offset) {
      key->actuation.status = STATUS_RAPID_TRIGGER_RESET;
      hid_release_key(key, _BASE_LAYER);
    }
    break;

  default:
    break;
  }
}

void update_key(struct key *key, struct state *state) {
  if (!update_key_state(key, state)) {
    return;
  }

  update_key_actuation(key);
}

void keyboard_init_keys(void) {

  for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
    for (uint8_t col = 0; col < MATRIX_COLS; col++) {
      if (channels_by_row_col[row][col][0] != XXXX) {
        init_key(channels_by_row_col[row][col][0], row, col);
      }
    }
  }
}

void keyboard_task(void) {
  uint32_t started_at = keyboard_get_time();

  for (uint8_t i = 0; i < 3; i++) {
    struct key *key = &keyboard_keys[i];
    struct state *state = &flex_keys[i];
    state->value = adc_values[i];

    if (!key->is_enabled)
      continue;

    update_key(key, state); 
  }

  for (uint8_t i = 0; i < 3; i++) {
    struct key *key = &keyboard_keys[i];

    if (!key->is_enabled || key->actuation.status != STATUS_MIGHT_BE_TAP)
      continue;

    uint8_t is_before_reset_offset = key->state.distance_8bits < key->actuation.reset_offset;
    uint8_t is_before_timeout = (keyboard_get_time() - key->actuation.triggered_at) <= keyboard_user_config.tap_timeout;

    if (is_before_reset_offset && is_before_timeout) {
      key->actuation.status = STATUS_TAP;
      hid_press_key(key, _TAP_LAYER);  // tap behavior
    } else if (!is_before_timeout || key_triggered) {
      key->actuation.status = STATUS_TRIGGERED;
      hid_press_key(key, _BASE_LAYER);  // normal key
    }
  }

  keyboard_last_cycle_duration = keyboard_get_time() - started_at;
}