/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * safety_guard.h
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

#include "config/scara_config.h"
#include "scara_ik.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  SAFETY_STATUS_OK = 0,
  SAFETY_ERR_OUT_OF_REACH,
  SAFETY_ERR_SINGULARITY_OUTER,
  SAFETY_ERR_SINGULARITY_INNER,
  SAFETY_ERR_JOINT_LIMIT_J1,
  SAFETY_ERR_JOINT_LIMIT_J2,
  SAFETY_ERR_Z_LIMIT,
  SAFETY_ERR_PATH_CROSSES_DEADZONE
} safety_status_t;

/**
 * @brief Evaluates whether a candidate Cartesian pose violates safety limits.
 *
 * Checks:
 * - Geometric reachability (r_min <= r <= r_max)
 * - Proximity to outer singularity (r ~ L1 + L2)
 * - Proximity to inner singularity (r ~ |L1 - L2|)
 * - Z travel bounds
 * - Joint 1 (shoulder) angular travel limits
 * - Joint 2 (elbow) angular travel limits
 *
 * @param geo Pointer to arm geometry definition.
 * @param target Pointer to target Cartesian pose.
 * @param config Elbow configuration.
 * @param out_joints Optional pointer to store calculated valid joints.
 * @return safety_status_t SAFETY_STATUS_OK if safe, error code otherwise.
 */
safety_status_t safety_guard_check_pose(
    const scara_geometry_t *geo, const scara_pose_t *target,
    scara_elbow_config_t config, scara_joints_t *out_joints
);

/**
 * @brief Evaluates whether a straight-line Cartesian path crosses the deadzone.
 *
 * @param geo Pointer to arm geometry definition.
 * @param p_start Pointer to starting Cartesian pose.
 * @param p_target Pointer to target Cartesian pose.
 * @return safety_status_t SAFETY_STATUS_OK if safe, error code otherwise.
 */
safety_status_t safety_guard_check_path(
    const scara_geometry_t *geo, const scara_pose_t *p_start,
    const scara_pose_t *p_target
);

/**
 * @brief Calculates speed damping scale factor (0.0 to 1.0) near singularity.
 *
 * As the arm approaches a singularity (|theta2| -> 0 or |theta2| -> PI),
 * small end-effector velocities demand excessive joint rates.
 * This function returns a scaling factor to smoothly slow down the toolhead.
 *
 * @param theta2 Elbow angle in radians.
 * @return float Velocity multiplier in range [0.1f, 1.0f].
 */
float safety_guard_get_speed_scaling(float theta2);

#ifdef __cplusplus
}
#endif
