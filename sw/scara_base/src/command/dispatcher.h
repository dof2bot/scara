/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * dispatcher.h
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

#include "parser.h"
#include "trajectory_queue.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Robot system operational states (subset of scara_core_state).
 */
typedef enum {
  SYSTEM_STATE_INIT = 0,
  SYSTEM_STATE_IDLE,
  SYSTEM_STATE_RUNNING,
  SYSTEM_STATE_HOLD,
  SYSTEM_STATE_ESTOP,
  SYSTEM_STATE_HOMING
} scara_system_state_t;

/**
 * @brief Initializes dispatcher and connects to trajectory queue.
 * @param queue Pointer to shared trajectory queue.
 */
void dispatcher_init(trajectory_queue_t *queue);

/**
 * @brief Dispatches a parsed command and handles response generation.
 * @param cmd Pointer to command received from parser.
 */
void dispatcher_dispatch(const scara_command_t *cmd);

/**
 * @brief Periodic dispatcher tick for state transitions and status management.
 */
void dispatcher_tick(void);

/**
 * @brief Returns current system operational state.
 * @return scara_system_state_t Current state.
 */
scara_system_state_t dispatcher_get_state(void);

/**
 * @brief Sets current system operational state.
 * @param state New state.
 */
void dispatcher_set_state(scara_system_state_t state);

/**
 * @brief Returns shared trajectory queue pointer.
 * @return trajectory_queue_t* Queue pointer or NULL.
 */
trajectory_queue_t *dispatcher_get_queue(void);

/**
 * @brief Returns string representation of operational state.
 * @param state Operational state.
 * @return const char* State string.
 */
const char *dispatcher_get_state_str(scara_system_state_t state);

#ifdef __cplusplus
}
#endif
