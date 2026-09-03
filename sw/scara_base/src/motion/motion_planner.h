/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * motion_planner.h
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

#include "kinematics/scara_ik.h"
#include "scara_config.h"
#include "stepper/stepper_driver.h"
#include "trajectory_chunk.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 3D/4D Waypoint target for trajectory generation.
 */
typedef struct {
  float x;     /* X coordinate (mm) */
  float y;     /* Y coordinate (mm) */
  float z;     /* Z height (mm) */
  float phi;   /* End-effector orientation (rad) */
  float speed; /* Cartesian feedrate (mm/s) */
} scara_waypoint_t;

/**
 * @brief Initializes motion planner with physical arm geometry.
 *
 * @param l1 Length of inner link (mm).
 * @param l2 Length of outer link (mm).
 */
void motion_planner_init(float l1, float l2);

/**
 * @brief Sets active elbow kinematic configuration (Righty / Lefty).
 *
 * @param config SCARA_ELBOW_RIGHT or SCARA_ELBOW_LEFT.
 */
void motion_planner_set_elbow_config(scara_elbow_config_t config);

/**
 * @brief Gets currently active elbow kinematic configuration.
 *
 * @return scara_elbow_config_t Active elbow configuration.
 */
scara_elbow_config_t motion_planner_get_elbow_config(void);

/**
 * @brief Converts joint angles/positions into discrete motor steps.
 *
 * @param joints Pointer to calculated joint values.
 * @return scara_step_coords_t Motor step coordinates.
 */
scara_step_coords_t motion_planner_joints_to_steps(const scara_joints_t *joints
);

/**
 * @brief Converts motor steps back into joint angles/positions.
 *
 * @param steps Pointer to motor step coordinates.
 * @return scara_joints_t Resulting joint values.
 */
scara_joints_t motion_planner_steps_to_joints(const scara_step_coords_t *steps);

/**
 * @brief Returns the current Cartesian tool pose of the robot.
 * @return scara_pose_t Current X, Y, Z, Phi.
 */
scara_pose_t motion_planner_get_current_pose(void);

/**
 * @brief Sets the current Cartesian tool pose (after homing or calibration).
 * @param pose Pointer to new pose.
 */
void motion_planner_set_current_pose(const scara_pose_t *pose);

/**
 * @brief Executes a synchronized linear move from current pose to target pose.
 *
 * @param target Target waypoint (X, Y, Z, Phi, Speed).
 * @return true if target was reachable and executed, false otherwise.
 */
bool motion_planner_move_linear(const scara_waypoint_t *target);

/**
 * @brief Generates a DMA command chunk for PIO from segment [p1 -> p2].
 *
 * @param p1 Starting pose.
 * @param p2 Ending pose.
 * @param speed Feedrate (mm/s).
 * @param out_buffer Output buffer for 32-bit PIO words.
 * @param max_words Capacity of out_buffer.
 * @param out_count Pointer to store generated word count.
 * @return true on success, false if unreachable or buffer overflow.
 */
bool motion_planner_generate_chunk(
    const scara_pose_t *p1, const scara_pose_t *p2, float speed,
    uint32_t *out_buffer, size_t max_words, size_t *out_count
);

#ifdef __cplusplus
}
#endif
