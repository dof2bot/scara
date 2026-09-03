/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * homing_controller.c
 * Copyright (C) 2026 Vladimir Roncevic <elektron.ronca@gmail.com>
 *
 * scara-base is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * scara-base is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#include "homing_controller.h"
#include "config/config_storage.h"
#include "hardware/gpio.h"
#include "io/io_gpio.h"
#include "motion/motion_planner.h"
#include "pico/stdlib.h"
#include "stepper/stepper_driver.h"
#include <math.h>

#define BACKOFF_STEPS 300U

static volatile bool homing_requested = false;
static volatile homing_state_t current_homing_state = HOMING_STATE_IDLE;

void homing_controller_request(void) {
  homing_requested = true;
}

bool homing_controller_is_requested(void) {
  return homing_requested;
}

homing_state_t homing_controller_get_state(void) {
  return current_homing_state;
}

static bool home_single_axis(
    uint dir_pin, uint step_pin, uint endstop_pin, uint8_t search_dir,
    uint32_t max_steps, uint32_t half_period_us, int32_t *pos_counter
) {
  /* Set search direction */
  gpio_put(dir_pin, search_dir);
  bool switch_hit = false;

  for (uint32_t step = 0; step < max_steps; step++) {
    if (io_gpio_read_endstop(endstop_pin)) {
      switch_hit = true;
      break;
    }
    gpio_put(step_pin, 1);
    sleep_us(half_period_us);
    gpio_put(step_pin, 0);
    sleep_us(half_period_us);
    if (pos_counter) {
      *pos_counter += (search_dir == 1) ? 1 : -1;
    }
  }

  if (!switch_hit) {
    return false;
  }

  /* Back off slightly from switch to clear trigger */
  uint8_t backoff_dir = (search_dir == 1) ? 0 : 1;
  gpio_put(dir_pin, backoff_dir);

  for (uint32_t b = 0; b < BACKOFF_STEPS; b++) {
    gpio_put(step_pin, 1);
    sleep_us(half_period_us);
    gpio_put(step_pin, 0);
    sleep_us(half_period_us);
    if (pos_counter) {
      *pos_counter += (backoff_dir == 1) ? 1 : -1;
    }
  }

  return true;
}

bool homing_controller_run(void) {
  homing_requested = false;
  current_homing_state = HOMING_STATE_Z;

  if (!stepper_driver_is_enabled()) {
    stepper_driver_set_enabled(true);
  }

  const scara_runtime_config_t *cfg = config_storage_get();
  uint32_t rate = (cfg->homing_rate_hz >= 100U) ? cfg->homing_rate_hz
                                                : SCARA_HOMING_STEP_RATE_HZ;
  uint32_t half_period = 1000000 / (2 * rate);
  scara_step_coords_t cur_steps = stepper_driver_get_position();

  /* 1. Sequence Step 1: Home Z axis to top switch */
  bool z_ok = home_single_axis(
      SCARA_Z_DIR_PIN, SCARA_Z_STEP_PIN, SCARA_ENDSTOP_Z_PIN, 1,
      SCARA_HOMING_MAX_STEPS_Z, half_period, &cur_steps.z_steps
  );
  if (!z_ok) {
    current_homing_state = HOMING_STATE_FAILED;
    return false;
  }

  /* 2. Sequence Step 2: Home J2 (elbow) */
  current_homing_state = HOMING_STATE_J2;
  bool j2_ok = home_single_axis(
      SCARA_J2_DIR_PIN, SCARA_J2_STEP_PIN, SCARA_ENDSTOP_Y_PIN, 0,
      SCARA_HOMING_MAX_STEPS_J2, half_period, &cur_steps.j2_steps
  );
  if (!j2_ok) {
    current_homing_state = HOMING_STATE_FAILED;
    return false;
  }

  /* 3. Sequence Step 3: Home J1 (shoulder) */
  current_homing_state = HOMING_STATE_J1;
  bool j1_ok = home_single_axis(
      SCARA_J1_DIR_PIN, SCARA_J1_STEP_PIN, SCARA_ENDSTOP_X_PIN, 0,
      SCARA_HOMING_MAX_STEPS_J1, half_period, &cur_steps.j1_steps
  );
  if (!j1_ok) {
    current_homing_state = HOMING_STATE_FAILED;
    return false;
  }

  /* 4. Sequence Step 4: Calibrate positions and set absolute pose */
  float t1 = cfg->home_offset_j1;
  float t2 = cfg->home_offset_j2;
  float z_val = cfg->z_max;

  cur_steps.j1_steps =
      (int32_t)lroundf(t1 * config_storage_get_steps_per_rad_j1());
  cur_steps.j2_steps =
      (int32_t)lroundf(t2 * config_storage_get_steps_per_rad_j2());
  cur_steps.z_steps =
      (int32_t)lroundf(z_val * config_storage_get_steps_per_mm_z());
  cur_steps.j4_steps = 0;
  stepper_driver_set_position(&cur_steps);

  float x = cfg->l1 * cosf(t1) + cfg->l2 * cosf(t1 + t2);
  float y = cfg->l1 * sinf(t1) + cfg->l2 * sinf(t1 + t2);

  scara_pose_t home_pose = {.x = x, .y = y, .z = z_val, .phi = t1 + t2};
  motion_planner_set_current_pose(&home_pose);

  current_homing_state = HOMING_STATE_SUCCESS;
  return true;
}
