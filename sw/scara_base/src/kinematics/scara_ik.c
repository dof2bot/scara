/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * scara_ik.c
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
#include "scara_ik.h"
#include <math.h>

/**
 * @brief Normalizes angle to range [-PI, +PI].
 */
static inline float normalize_angle(float angle) {
  while (angle > (float)M_PI) {
    angle -= TWO_PI;
  }

  while (angle < -(float)M_PI) {
    angle += TWO_PI;
  }

  return angle;
}

bool scara_ik_is_reachable(const scara_geometry_t *geo, float x, float y) {
  if (!geo || geo->l1 <= 0.0f || geo->l2 <= 0.0f) {
    return false;
  }

  float r_sq = x * x + y * y;
  float r = sqrtf(r_sq);
  float r_max = geo->l1 + geo->l2;
  float r_min = fabsf(geo->l1 - geo->l2);

  return (r <= r_max && r >= r_min);
}

scara_joints_t scara_ik_solve(
    const scara_geometry_t *geo, const scara_pose_t *target,
    scara_elbow_config_t config
) {
  scara_joints_t result = {0};
  result.reachable = false;

  if (!geo || !target) {
    return result;
  }

  float l1 = geo->l1;
  float l2 = geo->l2;
  float x = target->x;
  float y = target->y;

  float r_sq = x * x + y * y;
  float r = sqrtf(r_sq);

  float r_max = l1 + l2;
  float r_min = fabsf(l1 - l2);

  /* Check boundary conditions */
  if (r > r_max || r < r_min) {
    return result;
  }

  /* Cosine theorem for elbow joint angle theta2 */
  float cos_q2 = (r_sq - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);

  /* Numerical clamping for floating point inaccuracies */
  if (cos_q2 > 1.0f) {
    cos_q2 = 1.0f;
  } else if (cos_q2 < -1.0f) {
    cos_q2 = -1.0f;
  }

  float sin_q2 = sqrtf(1.0f - cos_q2 * cos_q2);

  /* Select elbow configuration */
  if (config == SCARA_ELBOW_LEFT) {
    sin_q2 = -sin_q2;
  }

  result.theta2 = atan2f(sin_q2, cos_q2);

  /* Shoulder angle theta1 */
  float k1 = l1 + l2 * cos_q2;
  float k2 = l2 * sin_q2;
  result.theta1 = atan2f(y, x) - atan2f(k2, k1);

  /* Normalization to [-PI, PI] */
  result.theta1 = normalize_angle(result.theta1);
  result.theta2 = normalize_angle(result.theta2);

  /* Direct mapping for prismatic Z axis */
  result.z = target->z;

  /* Tool orientation theta4 = target_phi - (theta1 + theta2) */
  result.theta4 = target->phi - (result.theta1 + result.theta2);
  result.theta4 = normalize_angle(result.theta4);

  result.reachable = true;

  return result;
}

scara_pose_t
scara_fk_solve(const scara_geometry_t *geo, const scara_joints_t *joints) {
  scara_pose_t pose = {0};

  if (!geo || !joints) {
    return pose;
  }

  float l1 = geo->l1;
  float l2 = geo->l2;
  float q1 = joints->theta1;
  float q2 = joints->theta2;

  pose.x = l1 * cosf(q1) + l2 * cosf(q1 + q2);
  pose.y = l1 * sinf(q1) + l2 * sinf(q1 + q2);
  pose.z = joints->z;
  pose.phi = normalize_angle(q1 + q2 + joints->theta4);

  return pose;
}
