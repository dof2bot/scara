/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * stepper_pulse.c
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
#include "stepper_pulse.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdlib.h>

void stepper_pulse_move_sync(
    const scara_step_coords_t *target_steps, scara_step_coords_t *current_pos,
    uint32_t step_rate_hz
) {
  if (!target_steps || !current_pos || step_rate_hz == 0) {
    return;
  }

  int32_t d_j1 = target_steps->j1_steps - current_pos->j1_steps;
  int32_t d_j2 = target_steps->j2_steps - current_pos->j2_steps;
  int32_t d_z = target_steps->z_steps - current_pos->z_steps;
  int32_t d_j4 = target_steps->j4_steps - current_pos->j4_steps;

  /* Set direction pins */
  gpio_put(SCARA_J1_DIR_PIN, d_j1 >= 0 ? 1 : 0);
  gpio_put(SCARA_J2_DIR_PIN, d_j2 >= 0 ? 1 : 0);
  gpio_put(SCARA_Z_DIR_PIN, d_z >= 0 ? 1 : 0);
  gpio_put(SCARA_J4_DIR_PIN, d_j4 >= 0 ? 1 : 0);

  int32_t abs_j1 = abs(d_j1);
  int32_t abs_j2 = abs(d_j2);
  int32_t abs_z = abs(d_z);
  int32_t abs_j4 = abs(d_j4);

  int32_t max_steps = abs_j1;
  if (abs_j2 > max_steps) {
    max_steps = abs_j2;
  }
  if (abs_z > max_steps) {
    max_steps = abs_z;
  }
  if (abs_j4 > max_steps) {
    max_steps = abs_j4;
  }

  if (max_steps == 0) {
    return;
  }

  uint32_t half_period_us = 1000000 / (2 * step_rate_hz);
  if (half_period_us < 2) {
    half_period_us = 2;
  }

  int32_t err_j1 = 0, err_j2 = 0, err_z = 0, err_j4 = 0;

  for (int32_t i = 0; i < max_steps; i++) {
    bool step_j1 = false, step_j2 = false, step_z = false, step_j4 = false;

    err_j1 += abs_j1;
    if (err_j1 >= max_steps) {
      err_j1 -= max_steps;
      step_j1 = true;
      current_pos->j1_steps += (d_j1 >= 0) ? 1 : -1;
    }

    err_j2 += abs_j2;
    if (err_j2 >= max_steps) {
      err_j2 -= max_steps;
      step_j2 = true;
      current_pos->j2_steps += (d_j2 >= 0) ? 1 : -1;
    }

    err_z += abs_z;
    if (err_z >= max_steps) {
      err_z -= max_steps;
      step_z = true;
      current_pos->z_steps += (d_z >= 0) ? 1 : -1;
    }

    err_j4 += abs_j4;
    if (err_j4 >= max_steps) {
      err_j4 -= max_steps;
      step_j4 = true;
      current_pos->j4_steps += (d_j4 >= 0) ? 1 : -1;
    }

    /* Pulse active pins HIGH */
    if (step_j1) {
      gpio_put(SCARA_J1_STEP_PIN, 1);
    }
    if (step_j2) {
      gpio_put(SCARA_J2_STEP_PIN, 1);
    }
    if (step_z) {
      gpio_put(SCARA_Z_STEP_PIN, 1);
    }
    if (step_j4) {
      gpio_put(SCARA_J4_STEP_PIN, 1);
    }

    sleep_us(half_period_us);

    /* Reset pins LOW */
    if (step_j1) {
      gpio_put(SCARA_J1_STEP_PIN, 0);
    }
    if (step_j2) {
      gpio_put(SCARA_J2_STEP_PIN, 0);
    }
    if (step_z) {
      gpio_put(SCARA_Z_STEP_PIN, 0);
    }
    if (step_j4) {
      gpio_put(SCARA_J4_STEP_PIN, 0);
    }

    sleep_us(half_period_us);
  }
}
