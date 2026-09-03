/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * velocity_profile.h
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
#pragma once

#include "scara_config.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Trapezoidal velocity profile planning state.
 */
typedef struct {
  float total_distance;   /* Total move distance (mm) */
  float accel;            /* Acceleration / Deceleration (mm/s^2) */
  float v_target;         /* Requested feedrate (mm/s) */
  float v_max;            /* Achievable peak velocity (mm/s) */
  float accel_dist;       /* Acceleration phase distance (mm) */
  float decel_start_dist; /* Deceleration start distance (mm) */
} velocity_profile_t;

/**
 * @brief Plans a trapezoidal acceleration/cruise/deceleration profile.
 *
 * @param profile Pointer to velocity profile structure to populate.
 * @param distance Total travel distance (mm).
 * @param target_speed Desired cruise feedrate (mm/s).
 * @param accel Acceleration limit (mm/s^2).
 * @return true if valid move distance, false otherwise.
 */
bool velocity_profile_plan(
    velocity_profile_t *profile, float distance, float target_speed, float accel
);

/**
 * @brief Calculates instantaneous velocity at a given distance along path.
 *
 * @param profile Pointer to planned velocity profile.
 * @param s Current distance traveled along path (mm).
 * @return float Instantaneous velocity (mm/s).
 */
float velocity_profile_get_velocity(const velocity_profile_t *profile, float s);

/**
 * @brief Converts segment duration dt into clamped PIO hardware delay cycles.
 *
 * @param dt Segment time duration in seconds.
 * @param sys_clk_hz System clock frequency in Hz.
 * @return uint32_t Clamped 24-bit delay cycle count.
 */
uint32_t velocity_profile_get_delay_cycles(float dt, uint32_t sys_clk_hz);

#ifdef __cplusplus
}
#endif
