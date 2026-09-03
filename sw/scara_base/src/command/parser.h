/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * parser.h
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

#include "config/config_storage.h"
#include "motion/motion_planner.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PARSER_RX_BUFFER_SIZE 128

/**
 * @brief Parsed command type identifier.
 */
typedef enum {
  CMD_TYPE_NONE = 0,
  CMD_TYPE_WAYPOINT,   /* <pt#X#Y#Z#PHI#SPEED#end> or <pt#X#Y#Z#SPEED#end> */
  CMD_TYPE_ENABLE,     /* <CMD:ENABLE> */
  CMD_TYPE_DISABLE,    /* <CMD:DISABLE> */
  CMD_TYPE_ESTOP,      /* <CMD:ESTOP> */
  CMD_TYPE_STATUS,     /* <CMD:STATUS> */
  CMD_TYPE_HOME,       /* <CMD:HOME> */
  CMD_TYPE_GETPOS,     /* <CMD:GETPOS> */
  CMD_TYPE_SETPOS,     /* <CMD:SETPOS#X#Y#Z#PHI#end> */
  CMD_TYPE_GET_CONFIG, /* <CMD:GET_CONFIG> */
  CMD_TYPE_SET_CONFIG, /* <CMD:SET_CONFIG#L1=...#L2=...#Z_MIN=...#Z_MAX=...#MIN_SPEED=...#MAX_SPEED=...> */
  CMD_TYPE_SET_DYNAMICS, /* <CMD:SET_DYNAMICS#ACCEL=...#MAX_ACCEL=...#DEF_SPD=...> */
  CMD_TYPE_SET_HOMING,   /* <CMD:SET_HOMING#OFF_J1=...#OFF_J2=...#RATE=...> */
  CMD_TYPE_SET_LIMITS,   /* <CMD:SET_LIMITS#J1_MIN=...#J1_MAX=...#J2_MIN=...#J2_MAX=...> */
  CMD_TYPE_SET_STEPS,    /* <CMD:SET_STEPS#GR_J1=...#GR_J2=...#GR_J4=...#LEAD_Z=...> */
  CMD_TYPE_SET_ELBOW,    /* <CMD:SET_ELBOW#LEFT> or <CMD:SET_ELBOW#RIGHT> */
  CMD_TYPE_GET_ELBOW,    /* <CMD:GET_ELBOW> */
  CMD_TYPE_SAVE_CONFIG,  /* <CMD:SAVE_CONFIG> */
  CMD_TYPE_RESET_CONFIG, /* <CMD:RESET_CONFIG> */
  CMD_TYPE_HOLD,         /* <CMD:HOLD> or <CMD:PAUSE> */
  CMD_TYPE_RESUME,       /* <CMD:RESUME> */
  CMD_TYPE_JOG,          /* <CMD:JOG#axis#step> */
  CMD_TYPE_PUMP,         /* <CMD:PUMP#1> or <CMD:PUMP#0> */
  CMD_TYPE_VALVE         /* <CMD:VALVE#1> or <CMD:VALVE#0> */
} scara_cmd_type_t;

/**
 * @brief Relative manual jog parameters.
 */
typedef struct {
  char axis;  /* 'X', 'Y', 'Z', 'P' (Phi) */
  float step; /* Relative step in mm or degrees */
} scara_jog_cmd_t;

/**
 * @brief End-effector tool actuation parameters.
 */
typedef struct {
  bool enable; /* true to turn on/open, false to turn off/close */
} scara_tool_cmd_t;

/**
 * @brief Dynamics & acceleration configuration parameters.
 */
typedef struct {
  float default_accel;
  float max_accel;
  float default_speed;
} scara_dynamics_cmd_t;

/**
 * @brief Homing calibration parameters.
 */
typedef struct {
  float home_offset_j1;
  float home_offset_j2;
  uint32_t homing_rate_hz;
} scara_homing_cmd_t;

/**
 * @brief Software joint angular travel limit parameters.
 */
typedef struct {
  float j1_min_rad;
  float j1_max_rad;
  float j2_min_rad;
  float j2_max_rad;
} scara_limits_cmd_t;

/**
 * @brief Stepper transmission and leadscrew parameters.
 */
typedef struct {
  float gear_ratio_j1;
  float gear_ratio_j2;
  float gear_ratio_j4;
  float leadscrew_pitch_z;
} scara_steps_cmd_t;

/**
 * @brief Elbow kinematic configuration parameter.
 */
typedef struct {
  scara_elbow_config_t config; /* SCARA_ELBOW_RIGHT or SCARA_ELBOW_LEFT */
} scara_elbow_cmd_t;

/**
 * @brief Parsed command payload.
 */
typedef struct {
  scara_cmd_type_t type;
  union {
    scara_waypoint_t waypoint;
    scara_pose_t set_pose;
    scara_runtime_config_t config;
    scara_dynamics_cmd_t dynamics;
    scara_homing_cmd_t homing;
    scara_limits_cmd_t limits;
    scara_steps_cmd_t steps;
    scara_elbow_cmd_t elbow;
    scara_jog_cmd_t jog;
    scara_tool_cmd_t tool;
  } data;
} scara_command_t;

/**
 * @brief Initializes serial parser buffers and state.
 */
void parser_init(void);

/**
 * @brief Non-blocking poll for incoming USB CDC characters.
 *
 * @param out_cmd Pointer to store a complete parsed command when available.
 * @return true if a complete valid command was received, false otherwise.
 */
bool parser_poll(scara_command_t *out_cmd);

#ifdef __cplusplus
}
#endif
