/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * trajectory_chunk.c
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
#include "trajectory_chunk.h"
#include "hardware/clocks.h"
#include "motion_planner.h"
#include "velocity_profile.h"
#include <math.h>

bool trajectory_chunk_generate(
    const scara_geometry_t *geometry, scara_elbow_config_t elbow_config,
    const scara_pose_t *p1, const scara_pose_t *p2, float speed,
    uint32_t *out_buffer, size_t max_words, size_t *out_count
) {
  if (!geometry || !p1 || !p2 || !out_buffer || max_words == 0 || !out_count) {
    return false;
  }

  float dx = p2->x - p1->x;
  float dy = p2->y - p1->y;
  float dz = p2->z - p1->z;
  float dphi = p2->phi - p1->phi;

  float distance = sqrtf(dx * dx + dy * dy + dz * dz);

  if (distance < 0.001f) {
    *out_count = 0;
    return true;
  }

  if (speed <= 0.1f) {
    speed = SCARA_DEFAULT_SPEED_MM_S;
  }

  if (speed > SCARA_MAX_SPEED_MM_S) {
    speed = SCARA_MAX_SPEED_MM_S;
  }

  uint32_t num_segments = (uint32_t)ceilf(distance / SCARA_SEGMENT_LEN_MM);

  if (num_segments > max_words) {
    num_segments = max_words;
  }

  velocity_profile_t v_prof;
  velocity_profile_plan(&v_prof, distance, speed, SCARA_DEFAULT_ACCEL_MM_S2);
  uint32_t sys_clk = clock_get_hz(clk_sys);
  float ds = distance / (float)num_segments;

  scara_joints_t j_init = scara_ik_solve(geometry, p1, elbow_config);

  if (!j_init.reachable || j_init.theta2 < SCARA_J2_MIN_RAD ||
      j_init.theta2 > SCARA_J2_MAX_RAD || j_init.theta1 < SCARA_J1_MIN_RAD ||
      j_init.theta1 > SCARA_J1_MAX_RAD) {
    return false;
  }

  scara_step_coords_t last_s = motion_planner_joints_to_steps(&j_init);

  for (uint32_t i = 1; i <= num_segments; i++) {
    float t = (float)i / (float)num_segments;
    float s = distance * t;
    float current_v = velocity_profile_get_velocity(&v_prof, s);
    float dt = ds / current_v;
    uint32_t delay_cycles = velocity_profile_get_delay_cycles(dt, sys_clk);

    scara_pose_t p_cur = {
        .x = p1->x + dx * t,
        .y = p1->y + dy * t,
        .z = p1->z + dz * t,
        .phi = p1->phi + dphi * t
    };

    scara_joints_t j_cur = scara_ik_solve(geometry, &p_cur, elbow_config);

    if (!j_cur.reachable || j_cur.theta2 < SCARA_J2_MIN_RAD ||
        j_cur.theta2 > SCARA_J2_MAX_RAD || j_cur.theta1 < SCARA_J1_MIN_RAD ||
        j_cur.theta1 > SCARA_J1_MAX_RAD) {
      return false;
    }

    scara_step_coords_t cur_s = motion_planner_joints_to_steps(&j_cur);

    uint8_t dir_mask = 0;
    uint8_t step_mask = 0;

    if (cur_s.j1_steps != last_s.j1_steps) {
      step_mask |= (1 << 0);
      if (cur_s.j1_steps > last_s.j1_steps) {
        dir_mask |= (1 << 0);
      }
    }
    if (cur_s.j2_steps != last_s.j2_steps) {
      step_mask |= (1 << 1);
      if (cur_s.j2_steps > last_s.j2_steps) {
        dir_mask |= (1 << 1);
      }
    }
    if (cur_s.z_steps != last_s.z_steps) {
      step_mask |= (1 << 2);
      if (cur_s.z_steps > last_s.z_steps) {
        dir_mask |= (1 << 2);
      }
    }
    if (cur_s.j4_steps != last_s.j4_steps) {
      step_mask |= (1 << 3);
      if (cur_s.j4_steps > last_s.j4_steps) {
        dir_mask |= (1 << 3);
      }
    }

    last_s = cur_s;

    /* Format 32-bit PIO word: [DIR(4)] [STEP(4)] [DELAY(24)] */
    out_buffer[i - 1] = ((uint32_t)dir_mask << 28) |
                        ((uint32_t)step_mask << 24) |
                        (delay_cycles & 0x00FFFFFF);
  }

  *out_count = num_segments;
  return true;
}
