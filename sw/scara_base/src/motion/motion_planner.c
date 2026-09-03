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
#include "config/config_storage.h"
#include "hardware/clocks.h"
#include "kinematics/safety_guard.h"
#include "trajectory_chunk.h"
#include "velocity_profile.h"
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

static const char RESP_TELEM_FMT[] =
    "<TELEM#X=%.2f#Y=%.2f#Z=%.2f#PHI=%.2f#J1=%ld#J2=%ld#Z_STEP=%ld#J4=%ld>\n";

void motion_planner_init(float l1, float l2) {
  if (l1 > 0.0f) {
    arm_geometry.l1 = l1;
  }
  if (l2 > 0.0f) {
    arm_geometry.l2 = l2;
  }
}

void motion_planner_set_elbow_config(scara_elbow_config_t config) {
  active_elbow_config = config;
}

scara_elbow_config_t motion_planner_get_elbow_config(void) {
  return active_elbow_config;
}

scara_step_coords_t motion_planner_joints_to_steps(const scara_joints_t *joints
) {
  scara_step_coords_t steps = {0};
  if (!joints) {
    return steps;
  }

  steps.j1_steps =
      (int32_t)lroundf(joints->theta1 * config_storage_get_steps_per_rad_j1());
  steps.j2_steps =
      (int32_t)lroundf(joints->theta2 * config_storage_get_steps_per_rad_j2());
  steps.z_steps =
      (int32_t)lroundf(joints->z * config_storage_get_steps_per_mm_z());
  steps.j4_steps =
      (int32_t)lroundf(joints->theta4 * config_storage_get_steps_per_rad_j4());

  return steps;
}

scara_joints_t motion_planner_steps_to_joints(const scara_step_coords_t *steps
) {
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

  /* Verify safety constraints (limits, singularities) */
  scara_joints_t final_joints;
  safety_status_t safety_stat = safety_guard_check_pose(
      &arm_geometry, &target_pose, active_elbow_config, &final_joints
  );

  if (safety_stat != SAFETY_STATUS_OK) {
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

  const scara_runtime_config_t *cfg = config_storage_get();
  float def_spd = (cfg->default_speed > 0.1f) ? cfg->default_speed
                                              : SCARA_DEFAULT_SPEED_MM_S;
  float speed = (target->speed > 0.1f) ? target->speed : def_spd;

  /* Scale Cartesian speed smoothly near singularities */
  float speed_scale = safety_guard_get_speed_scaling(final_joints.theta2);
  speed *= speed_scale;

  float max_spd =
      (cfg->max_speed > 0.1f) ? cfg->max_speed : SCARA_MAX_SPEED_MM_S;
  float min_spd =
      (cfg->min_speed > 0.01f) ? cfg->min_speed : SCARA_MIN_SPEED_MM_S;

  if (speed > max_spd) {
    speed = max_spd;
  } else if (speed < min_spd) {
    speed = min_spd;
  }

  /* Segment line by SCARA_SEGMENT_LEN_MM */
  uint32_t num_segments = (uint32_t)ceilf(distance / SCARA_SEGMENT_LEN_MM);
  if (num_segments == 0) {
    num_segments = 1;
  }

  uint32_t remaining = num_segments;
  uint32_t seg_offset = 0;

  while (remaining > 0) {
    uint32_t batch_size = remaining;
    if (batch_size > SCARA_STREAM_BATCH_SEGMENTS) {
      batch_size = SCARA_STREAM_BATCH_SEGMENTS;
    }

    float t_start = (float)seg_offset / (float)num_segments;
    float t_end = (float)(seg_offset + batch_size) / (float)num_segments;

    scara_pose_t p_start = {
        .x = current_cartesian_pose.x + dx * t_start,
        .y = current_cartesian_pose.y + dy * t_start,
        .z = current_cartesian_pose.z + dz * t_start,
        .phi = current_cartesian_pose.phi + dphi * t_start
    };
    scara_pose_t p_end = {
        .x = current_cartesian_pose.x + dx * t_end,
        .y = current_cartesian_pose.y + dy * t_end,
        .z = current_cartesian_pose.z + dz * t_end,
        .phi = current_cartesian_pose.phi + dphi * t_end
    };

    static uint32_t chunk_words[SCARA_MAX_SEGMENTS_CHUNK];
    size_t words_generated = 0;

    bool ok = trajectory_chunk_generate(
        &arm_geometry, active_elbow_config, &p_start, &p_end, speed,
        chunk_words, batch_size, &words_generated
    );

    if (!ok) {
      return false;
    }

    if (words_generated > 0) {
      stepper_driver_stream_chunk(chunk_words, words_generated);
      scara_joints_t cur_j =
          scara_ik_solve(&arm_geometry, &p_end, active_elbow_config);
      scara_step_coords_t cur_s = motion_planner_joints_to_steps(&cur_j);
      printf(
          RESP_TELEM_FMT,
          p_end.x, p_end.y, p_end.z, p_end.phi, (long)cur_s.j1_steps,
          (long)cur_s.j2_steps, (long)cur_s.z_steps, (long)cur_s.j4_steps
      );
    }

    seg_offset += batch_size;
    remaining -= batch_size;
  }

  scara_step_coords_t final_steps =
      motion_planner_joints_to_steps(&final_joints);
  stepper_driver_set_position(&final_steps);
  current_cartesian_pose = target_pose;
  return true;
}

bool motion_planner_generate_chunk(
    const scara_pose_t *p1, const scara_pose_t *p2, float speed,
    uint32_t *out_buffer, size_t max_words, size_t *out_count
) {
  return trajectory_chunk_generate(
      &arm_geometry, active_elbow_config, p1, p2, speed, out_buffer, max_words,
      out_count
  );
}
