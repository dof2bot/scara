/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * dispatcher.c
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
#include "dispatcher.h"
#include "handlers/cmd_config.h"
#include "handlers/cmd_motion.h"
#include "handlers/cmd_system.h"
#include "handlers/cmd_tool.h"
#include "homing/homing_controller.h"
#include "stepper/stepper_driver.h"
#include <stdio.h>

/* State string representations */
static const char STATE_STR_IDLE[] = "IDLE";
static const char STATE_STR_RUNNING[] = "RUNNING";
static const char STATE_STR_HOLD[] = "HOLD";
static const char STATE_STR_ESTOP[] = "ESTOP";
static const char STATE_STR_HOMING[] = "HOMING";
static const char STATE_STR_UNKNOWN[] = "UNKNOWN";

/* Response literals */
static const char RESP_INIT_OK[] = "<RESP:INIT_OK#STATE=IDLE>\n";
static const char RESP_NACK_UNKNOWN_CMD[] = "<RESP:NACK_UNKNOWN_CMD>\n";

static trajectory_queue_t *traj_queue = NULL;
static scara_system_state_t system_state = SYSTEM_STATE_INIT;

const char *dispatcher_get_state_str(scara_system_state_t state) {
  switch (state) {
  case SYSTEM_STATE_IDLE:
    return STATE_STR_IDLE;
  case SYSTEM_STATE_RUNNING:
    return STATE_STR_RUNNING;
  case SYSTEM_STATE_HOLD:
    return STATE_STR_HOLD;
  case SYSTEM_STATE_ESTOP:
    return STATE_STR_ESTOP;
  case SYSTEM_STATE_HOMING:
    return STATE_STR_HOMING;
  default:
    return STATE_STR_UNKNOWN;
  }
}

void dispatcher_init(trajectory_queue_t *queue) {
  traj_queue = queue;
  system_state = SYSTEM_STATE_IDLE;
  printf("%s", RESP_INIT_OK);
}

scara_system_state_t dispatcher_get_state(void) {
  return system_state;
}

void dispatcher_set_state(scara_system_state_t state) {
  system_state = state;
}

trajectory_queue_t *dispatcher_get_queue(void) {
  return traj_queue;
}

void dispatcher_dispatch(const scara_command_t *cmd) {
  if (!cmd) {
    return;
  }

  if (cmd_motion_handle(cmd)) {
    return;
  }
  if (cmd_system_handle(cmd)) {
    return;
  }
  if (cmd_config_handle(cmd)) {
    return;
  }
  if (cmd_tool_handle(cmd)) {
    return;
  }

  printf("%s", RESP_NACK_UNKNOWN_CMD);
}

void dispatcher_tick(void) {
  if (system_state == SYSTEM_STATE_ESTOP || system_state == SYSTEM_STATE_HOLD) {
    return;
  }

  if (system_state == SYSTEM_STATE_HOMING) {
    if (!homing_controller_is_requested()) {
      if (homing_controller_get_state() == HOMING_STATE_SUCCESS) {
        system_state = SYSTEM_STATE_IDLE;
      } else if (homing_controller_get_state() == HOMING_STATE_FAILED) {
        system_state = SYSTEM_STATE_ESTOP;
      }
    }
    return;
  }

  bool busy = stepper_driver_is_busy();
  bool has_queued_points =
      traj_queue && !trajectory_queue_is_empty(traj_queue);

  if (busy || has_queued_points) {
    system_state = SYSTEM_STATE_RUNNING;
  } else {
    system_state = SYSTEM_STATE_IDLE;
  }
}
