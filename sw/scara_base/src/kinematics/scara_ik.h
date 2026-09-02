/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * scara_ik.h
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

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SCARA elbow orientation configuration.
 *
 * This configuration determines the handedness of the arm, i.e., whether the
 * elbow points to the right or left of the vector connecting the base to the
 * end-effector.
 */
typedef enum {
  SCARA_ELBOW_RIGHT = 0, /* Elbow Right / Righty (default) */
  SCARA_ELBOW_LEFT = 1   /* Elbow Left / Lefty */
} scara_elbow_config_t;

/**
 * @brief Geometric dimensions of SCARA arm links.
 *
 * This structure defines the physical dimensions of the SCARA arm, which are
 * used to calculate the reachable workspace and joint angles.
 */
typedef struct {
  /**
   * @brief Length of the inner arm segment (Joint 1 to Joint 2).
   *
   * This value is the distance from the center of Joint 1 to the center of
   * Joint 2.
   */
  float l1;
  /**
   * @brief Length of the outer arm segment (Joint 2 to End-Effector).
   *
   * This value is the distance from the center of Joint 2 to the center of
   * the tool mount.
   */
  float l2;
} scara_geometry_t;

/**
 * @brief Target Cartesian tool pose.
 *
 * This structure represents the desired position and orientation of the
 * end-effector in Cartesian space.
 */
typedef struct {
  float x;   /* X coordinate (mm) */
  float y;   /* Y coordinate (mm) */
  float z;   /* Z height (mm) */
  float phi; /* Tool end-effector orientation (rad) */
} scara_pose_t;

/**
 * @brief Robot joint angles and positions.
 *
 * This structure represents the resulting joint angles and positions after
 * solving the inverse kinematics problem.
 */
typedef struct {
  float theta1;   /* Shoulder angle Joint 1 (rad) */
  float theta2;   /* Elbow angle Joint 2 (rad) */
  float z;        /* Prismatic Z axis position (mm) */
  float theta4;   /* Wrist angle Joint 4 (rad) */
  bool reachable; /* True if target pose is within reachable workspace */
} scara_joints_t;

/**
 * @brief Solves analytical Inverse Kinematics for SCARA 4-DOF manipulator.
 *
 * @param geo Pointer to SCARA geometry definition (link lengths).
 * @param target Pointer to target Cartesian pose (X, Y, Z, Phi).
 * @param config Elbow configuration (SCARA_ELBOW_RIGHT or SCARA_ELBOW_LEFT).
 * @return scara_joints_t Calculated joint values and reachability status.
 */
scara_joints_t scara_ik_solve(
    const scara_geometry_t *geo, const scara_pose_t *target,
    scara_elbow_config_t config
);

/**
 * @brief Solves Forward Kinematics (FK) to calculate Cartesian pose from joint
 * angles.
 *
 * @param geo Pointer to SCARA geometry definition.
 * @param joints Pointer to joint angles.
 * @return scara_pose_t Resulting Cartesian end-effector pose.
 */
scara_pose_t
scara_fk_solve(const scara_geometry_t *geo, const scara_joints_t *joints);

/**
 * @brief Checks if a given (X, Y) coordinate is inside the reachable workspace
 * ring.
 *
 * @param geo Pointer to SCARA geometry definition.
 * @param x Target X coordinate (mm).
 * @param y Target Y coordinate (mm).
 * @return true if reachable, false otherwise.
 */
bool scara_ik_is_reachable(const scara_geometry_t *geo, float x, float y);

#ifdef __cplusplus
}
#endif
