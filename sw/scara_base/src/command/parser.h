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
  CMD_TYPE_WAYPOINT,     /* <pt#X#Y#Z#PHI#SPEED#end> or <pt#X#Y#Z#SPEED#end> */
  CMD_TYPE_ENABLE,       /* <CMD:ENABLE> */
  CMD_TYPE_DISABLE,      /* <CMD:DISABLE> */
  CMD_TYPE_ESTOP,        /* <CMD:ESTOP> */
  CMD_TYPE_STATUS,       /* <CMD:STATUS> */
  CMD_TYPE_HOME,         /* <CMD:HOME> */
  CMD_TYPE_GETPOS,       /* <CMD:GETPOS> */
  CMD_TYPE_SETPOS,       /* <CMD:SETPOS#X#Y#Z#PHI#end> */
  CMD_TYPE_GET_CONFIG,   /* <CMD:GET_CONFIG> */
  CMD_TYPE_SET_CONFIG,   /* <CMD:SET_CONFIG#L1=...#L2=...#Z_MIN=...#Z_MAX=...#MIN_SPEED=...#MAX_SPEED=...> */
  CMD_TYPE_SAVE_CONFIG,  /* <CMD:SAVE_CONFIG> */
  CMD_TYPE_RESET_CONFIG  /* <CMD:RESET_CONFIG> */
} scara_cmd_type_t;

/**
 * @brief Parsed command payload.
 */
typedef struct {
  scara_cmd_type_t type;
  union {
    scara_waypoint_t waypoint;
    scara_pose_t set_pose;
    scara_runtime_config_t config;
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
