/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * velocity_profile.c
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
#include "velocity_profile.h"
#include <math.h>

#define MIN_PROGRESS_S 0.01f
#define MIN_PIO_CYCLES 200U
#define MAX_PIO_CYCLES 0x00FFFFFFU

bool velocity_profile_plan(
    velocity_profile_t *profile, float distance, float target_speed, float accel
) {
  if (!profile || distance <= 0.001f) {
    return false;
  }

  float a = (accel > 1.0f) ? accel : SCARA_DEFAULT_ACCEL_MM_S2;
  if (a > SCARA_MAX_ACCEL_MM_S2) {
    a = SCARA_MAX_ACCEL_MM_S2;
  }

  float v_req = (target_speed > SCARA_MIN_SPEED_MM_S)
                    ? target_speed
                    : SCARA_DEFAULT_SPEED_MM_S;
  if (v_req > SCARA_MAX_SPEED_MM_S) {
    v_req = SCARA_MAX_SPEED_MM_S;
  }

  profile->total_distance = distance;
  profile->accel = a;
  profile->v_target = v_req;

  /* Calculate distance required to reach requested speed */
  float s_accel = (v_req * v_req) / (2.0f * a);

  /* If distance is too short, fall back to triangular profile */
  if (s_accel > distance / 2.0f) {
    s_accel = distance / 2.0f;
    profile->v_max = sqrtf(2.0f * a * s_accel);
  } else {
    profile->v_max = v_req;
  }

  profile->accel_dist = s_accel;
  profile->decel_start_dist = distance - s_accel;

  return true;
}

float velocity_profile_get_velocity(
    const velocity_profile_t *profile, float s
) {
  if (!profile || profile->total_distance <= 0.001f) {
    return SCARA_MIN_SPEED_MM_S;
  }

  float current_v;
  float s_clamped = (s < 0.0f) ? 0.0f : s;
  if (s_clamped > profile->total_distance) {
    s_clamped = profile->total_distance;
  }

  if (s_clamped < profile->accel_dist) {
    /* Acceleration phase: v = sqrt(2 * a * s) */
    float s_eff = fmaxf(s_clamped, MIN_PROGRESS_S);
    current_v = sqrtf(2.0f * profile->accel * s_eff);
  } else if (s_clamped >= profile->decel_start_dist) {
    /* Deceleration phase: v = sqrt(2 * a * (dist - s)) */
    float s_rem = fmaxf(profile->total_distance - s_clamped, MIN_PROGRESS_S);
    current_v = sqrtf(2.0f * profile->accel * s_rem);
  } else {
    /* Constant speed cruise phase */
    current_v = profile->v_max;
  }

  if (current_v < SCARA_MIN_SPEED_MM_S) {
    current_v = SCARA_MIN_SPEED_MM_S;
  }

  return current_v;
}

uint32_t velocity_profile_get_delay_cycles(float dt, uint32_t sys_clk_hz) {
  if (dt <= 0.0f || sys_clk_hz == 0) {
    return MIN_PIO_CYCLES;
  }

  uint32_t cycles = (uint32_t)(sys_clk_hz * dt);
  if (cycles < MIN_PIO_CYCLES) {
    cycles = MIN_PIO_CYCLES;
  }
  if (cycles > MAX_PIO_CYCLES) {
    cycles = MAX_PIO_CYCLES;
  }

  return cycles;
}
