/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * motion_planner.c
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
#include "motion_planner.h"
#include "hardware/clocks.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static scara_geometry_t arm_geometry = {
    .l1 = SCARA_ARM_L1_MM, .l2 = SCARA_ARM_L2_MM
};

static scara_pose_t current_cartesian_pose = {
    .x = SCARA_ARM_L1_MM + SCARA_ARM_L2_MM, .y = 0.0f, .z = 0.0f, .phi = 0.0f
};

static scara_elbow_config_t active_elbow_config = SCARA_ELBOW_RIGHT;

void motion_planner_init(float l1, float l2) {
  if (l1 > 0.0f) {
    arm_geometry.l1 = l1;
  }
  if (l2 > 0.0f) {
    arm_geometry.l2 = l2;
  }
}

scara_step_coords_t
motion_planner_joints_to_steps(const scara_joints_t *joints) {
  scara_step_coords_t steps = {0};
  if (!joints) {
    return steps;
  }

  steps.j1_steps = (int32_t)lroundf(joints->theta1 * SCARA_STEPS_PER_RAD_J1);
  steps.j2_steps = (int32_t)lroundf(joints->theta2 * SCARA_STEPS_PER_RAD_J2);
  steps.z_steps = (int32_t)lroundf(joints->z * SCARA_STEPS_PER_MM_Z);
  steps.j4_steps = (int32_t)lroundf(joints->theta4 * SCARA_STEPS_PER_RAD_J4);

  return steps;
}

scara_joints_t
motion_planner_steps_to_joints(const scara_step_coords_t *steps) {
  scara_joints_t joints = {0};

  if (!steps) {
    return joints;
  }

  joints.theta1 = (float)steps->j1_steps / SCARA_STEPS_PER_RAD_J1;
  joints.theta2 = (float)steps->j2_steps / SCARA_STEPS_PER_RAD_J2;
  joints.z = (float)steps->z_steps / SCARA_STEPS_PER_MM_Z;
  joints.theta4 = (float)steps->j4_steps / SCARA_STEPS_PER_RAD_J4;
  joints.reachable = true;

  return joints;
}

scara_pose_t motion_planner_get_current_pose(void) {
  return current_cartesian_pose;
}

void motion_planner_set_current_pose(const scara_pose_t *pose) {
  if (pose) {
    current_cartesian_pose = *pose;
    scara_joints_t j = scara_ik_solve(&arm_geometry, pose, active_elbow_config);

    if (j.reachable) {
      scara_step_coords_t s = motion_planner_joints_to_steps(&j);
      stepper_driver_set_position(&s);
    }
  }
}

bool motion_planner_move_linear(const scara_waypoint_t *target) {
  if (!target) {
    return false;
  }

  scara_pose_t target_pose = {
      .x = target->x, .y = target->y, .z = target->z, .phi = target->phi
  };

  /* Verify target reachability first */
  scara_joints_t final_joints =
      scara_ik_solve(&arm_geometry, &target_pose, active_elbow_config);

  if (!final_joints.reachable) {
    return false;
  }

  float dx = target_pose.x - current_cartesian_pose.x;
  float dy = target_pose.y - current_cartesian_pose.y;
  float dz = target_pose.z - current_cartesian_pose.z;
  float dphi = target_pose.phi - current_cartesian_pose.phi;

  float distance = sqrtf(dx * dx + dy * dy + dz * dz);

  if (distance < 0.001f && fabsf(dphi) < 0.001f) {
    return true; /* Already at target */
  }

  float speed =
      (target->speed > 0.1f) ? target->speed : SCARA_DEFAULT_SPEED_MM_S;

  if (speed > SCARA_MAX_SPEED_MM_S) {
    speed = SCARA_MAX_SPEED_MM_S;
  }

  /* Segment line by SCARA_SEGMENT_LEN_MM */
  uint32_t num_segments = (uint32_t)ceilf(distance / SCARA_SEGMENT_LEN_MM);
  if (num_segments == 0) {
    num_segments = 1;
  }

  float dt_segment =
      (distance > 0.001f) ? ((distance / speed) / (float)num_segments) : 0.01f;

  for (uint32_t i = 1; i <= num_segments; i++) {
    float t = (float)i / (float)num_segments;
    scara_pose_t intermediate_pose = {
        .x = current_cartesian_pose.x + dx * t,
        .y = current_cartesian_pose.y + dy * t,
        .z = current_cartesian_pose.z + dz * t,
        .phi = current_cartesian_pose.phi + dphi * t
    };

    scara_joints_t seg_joints =
        scara_ik_solve(&arm_geometry, &intermediate_pose, active_elbow_config);

    if (!seg_joints.reachable) {
      return false;
    }

    scara_step_coords_t seg_steps = motion_planner_joints_to_steps(&seg_joints);
    scara_step_coords_t cur_pos = stepper_driver_get_position();

    int32_t d1 = labs((long)seg_steps.j1_steps - (long)cur_pos.j1_steps);
    int32_t d2 = labs((long)seg_steps.j2_steps - (long)cur_pos.j2_steps);
    int32_t dz_s = labs((long)seg_steps.z_steps - (long)cur_pos.z_steps);
    int32_t d4 = labs((long)seg_steps.j4_steps - (long)cur_pos.j4_steps);

    int32_t max_steps = d1;
    if (d2 > max_steps) {
      max_steps = d2;
    }
    if (dz_s > max_steps) {
      max_steps = dz_s;
    }
    if (d4 > max_steps) {
      max_steps = d4;
    }

    uint32_t step_rate_hz = 1000;
    if (dt_segment > 0.0001f && max_steps > 0) {
      step_rate_hz = (uint32_t)ceilf((float)max_steps / dt_segment);
    }

    if (step_rate_hz < 100) {
      step_rate_hz = 100;
    }
    if (step_rate_hz > 50000) {
      step_rate_hz = 50000;
    }

    stepper_driver_move_steps_sync(&seg_steps, step_rate_hz);

    /* Stream real-time telemetry back to host (HIL Digital Twin feedback) */
    if (i % 2 == 0 || i == num_segments) {
      printf(
          "<TELEM#X=%.2f#Y=%.2f#Z=%.2f#PHI=%.2f#J1=%ld#J2=%ld#Z_STEP=%ld#J4=%"
          "ld>\n",
          intermediate_pose.x, intermediate_pose.y, intermediate_pose.z,
          intermediate_pose.phi, (long)seg_steps.j1_steps,
          (long)seg_steps.j2_steps, (long)seg_steps.z_steps,
          (long)seg_steps.j4_steps
      );
    }
  }

  current_cartesian_pose = target_pose;
  return true;
}

bool motion_planner_generate_chunk(
    const scara_pose_t *p1, const scara_pose_t *p2, float speed,
    uint32_t *out_buffer, size_t max_words, size_t *out_count
) {
  if (!p1 || !p2 || !out_buffer || max_words == 0 || !out_count) {
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

  float dt = (distance / speed) / (float)num_segments;
  uint32_t delay_cycles = (uint32_t)(clock_get_hz(clk_sys) * dt);

  if (delay_cycles < 200) {
    delay_cycles = 200;
  }

  if (delay_cycles > 0x00FFFFFF) {
    delay_cycles = 0x00FFFFFF;
  }

  scara_joints_t j_init =
      scara_ik_solve(&arm_geometry, p1, active_elbow_config);

  if (!j_init.reachable) {
    return false;
  }

  scara_step_coords_t last_s = motion_planner_joints_to_steps(&j_init);

  for (uint32_t i = 1; i <= num_segments; i++) {
    float t = (float)i / (float)num_segments;
    scara_pose_t p_cur = {
        .x = p1->x + dx * t,
        .y = p1->y + dy * t,
        .z = p1->z + dz * t,
        .phi = p1->phi + dphi * t
    };

    scara_joints_t j_cur =
        scara_ik_solve(&arm_geometry, &p_cur, active_elbow_config);

    if (!j_cur.reachable) {
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
