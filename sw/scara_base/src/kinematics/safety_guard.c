/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * safety_guard.c
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
#include "safety_guard.h"
#include "config/config_storage.h"
#include <math.h>

safety_status_t safety_guard_check_pose(
    const scara_geometry_t *geo, const scara_pose_t *target,
    scara_elbow_config_t config, scara_joints_t *out_joints
) {
  if (!geo || !target) {
    return SAFETY_ERR_OUT_OF_REACH;
  }

  const scara_runtime_config_t *cfg = config_storage_get();
  float z_min = (cfg && cfg->z_min < cfg->z_max) ? cfg->z_min : SCARA_Z_MIN_MM;
  float z_max = (cfg && cfg->z_min < cfg->z_max) ? cfg->z_max : SCARA_Z_MAX_MM;

  /* 1. Check Z travel limits */
  if (target->z < z_min || target->z > z_max) {
    return SAFETY_ERR_Z_LIMIT;
  }

  float r_sq = target->x * target->x + target->y * target->y;
  float r = sqrtf(r_sq);
  float r_max = geo->l1 + geo->l2;
  float r_min = fabsf(geo->l1 - geo->l2);

  /* 2. Check basic radial reachability */
  if (r > r_max || r < r_min) {
    return SAFETY_ERR_OUT_OF_REACH;
  }

  /* 3. Check proximity to outer singularity (fully extended arm) */
  if (r > (r_max - SCARA_SINGULARITY_OUTER_MARGIN_MM)) {
    return SAFETY_ERR_SINGULARITY_OUTER;
  }

  /* 4. Check proximity to inner singularity (fully folded arm) */
  if (r < (r_min + SCARA_SINGULARITY_INNER_MARGIN_MM)) {
    return SAFETY_ERR_SINGULARITY_INNER;
  }

  /* 5. Solve analytical IK and verify joint angle limits */
  scara_joints_t j = scara_ik_solve(geo, target, config);
  if (!j.reachable) {
    return SAFETY_ERR_OUT_OF_REACH;
  }

  float j1_min = (cfg && cfg->j1_min_rad < cfg->j1_max_rad) ? cfg->j1_min_rad
                                                           : SCARA_J1_MIN_RAD;
  float j1_max = (cfg && cfg->j1_min_rad < cfg->j1_max_rad) ? cfg->j1_max_rad
                                                           : SCARA_J1_MAX_RAD;
  float j2_min = (cfg && cfg->j2_min_rad < cfg->j2_max_rad) ? cfg->j2_min_rad
                                                           : SCARA_J2_MIN_RAD;
  float j2_max = (cfg && cfg->j2_min_rad < cfg->j2_max_rad) ? cfg->j2_max_rad
                                                           : SCARA_J2_MAX_RAD;

  if (j.theta1 < j1_min || j.theta1 > j1_max) {
    return SAFETY_ERR_JOINT_LIMIT_J1;
  }

  if (j.theta2 < j2_min || j.theta2 > j2_max) {
    return SAFETY_ERR_JOINT_LIMIT_J2;
  }

  /* Check elbow angle against singularity threshold */
  if (fabsf(j.theta2) < SCARA_SINGULARITY_THETA2_MIN_RAD) {
    return SAFETY_ERR_SINGULARITY_OUTER;
  }

  if (fabsf((float)M_PI - fabsf(j.theta2)) < SCARA_SINGULARITY_THETA2_MIN_RAD) {
    return SAFETY_ERR_SINGULARITY_INNER;
  }

  if (out_joints) {
    *out_joints = j;
  }

  return SAFETY_STATUS_OK;
}

float safety_guard_get_speed_scaling(float theta2) {
  float sin_theta2 = fabsf(sinf(theta2));
  /* Smooth scaling based on Jacobian condition metric: 0.5 ~ 30 deg */
  float scale = sin_theta2 / 0.5f;
  if (scale > 1.0f) {
    scale = 1.0f;
  } else if (scale < 0.1f) {
    scale = 0.1f;
  }
  return scale;
}

safety_status_t safety_guard_check_path(
    const scara_geometry_t *geo, const scara_pose_t *p_start,
    const scara_pose_t *p_target
) {
  if (!geo || !p_start || !p_target) {
    return SAFETY_ERR_OUT_OF_REACH;
  }

  /* Calculate physical dead zone radius based on J2 max angle */
  float r_dead_sq = geo->l1 * geo->l1 + geo->l2 * geo->l2 +
                    2.0f * geo->l1 * geo->l2 * cosf(SCARA_J2_MAX_RAD);
  float r_dead = (r_dead_sq > 0.0f) ? sqrtf(r_dead_sq) : 0.0f;

  /* Closest approach to origin (0,0) on segment [p_start, p_target] */
  float dx = p_target->x - p_start->x;
  float dy = p_target->y - p_start->y;
  float len_sq = dx * dx + dy * dy;

  float d_min = 0.0f;
  if (len_sq < 1e-4f) {
    d_min = sqrtf(p_start->x * p_start->x + p_start->y * p_start->y);
  } else {
    float t = -(p_start->x * dx + p_start->y * dy) / len_sq;
    if (t < 0.0f) {
      t = 0.0f;
    } else if (t > 1.0f) {
      t = 1.0f;
    }
    float cx = p_start->x + t * dx;
    float cy = p_start->y + t * dy;
    d_min = sqrtf(cx * cx + cy * cy);
  }

  if (d_min < r_dead) {
    return SAFETY_ERR_PATH_CROSSES_DEADZONE;
  }

  return SAFETY_STATUS_OK;
}
