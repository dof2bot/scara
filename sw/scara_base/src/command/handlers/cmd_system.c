/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * cmd_system.c
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
#include "cmd_system.h"
#include "dispatcher.h"
#include "homing/homing_controller.h"
#include "motion/motion_planner.h"
#include "stepper/stepper_driver.h"
#include "trajectory_queue.h"
#include <stdio.h>

/* System response literals */
static const char RESP_ACK_MOTORS_ENABLED[] = "<RESP:ACK#MOTORS_ENABLED>\n";
static const char RESP_ACK_MOTORS_DISABLED[] = "<RESP:ACK#MOTORS_DISABLED>\n";
static const char RESP_ACK_ESTOP_TRIGGERED[] = "<RESP:ACK#ESTOP_TRIGGERED>\n";
static const char RESP_ACK_SETPOS_DONE[] = "<RESP:ACK#SETPOS_DONE>\n";
static const char RESP_ACK_HOMING_STARTED[] = "<RESP:ACK#HOMING_STARTED>\n";
static const char RESP_NACK_ESTOP_ACTIVE[] = "<RESP:NACK_ESTOP_ACTIVE>\n";

static const char RESP_STATUS_FMT[] =
    "<STATUS#STATE=%s#BUSY=%d#Q=%u#X=%.2f#Y=%.2f#Z=%.2f#PHI=%.2f>\n";
static const char RESP_POS_FMT[] =
    "<POS#X=%.2f#Y=%.2f#Z=%.2f#PHI=%.2f#J1=%ld#J2=%ld#Z_STEP=%ld#J4=%ld>\n";

static void handle_enable(void) {
  stepper_driver_set_enabled(true);
  if (dispatcher_get_state() == SYSTEM_STATE_ESTOP) {
    dispatcher_set_state(SYSTEM_STATE_IDLE);
  }
  printf("%s", RESP_ACK_MOTORS_ENABLED);
}

static void handle_disable(void) {
  stepper_driver_set_enabled(false);
  printf("%s", RESP_ACK_MOTORS_DISABLED);
}

static void handle_estop(void) {
  dispatcher_set_state(SYSTEM_STATE_ESTOP);
  stepper_driver_emergency_stop();

  trajectory_queue_t *traj_queue = dispatcher_get_queue();
  if (traj_queue) {
    trajectory_queue_clear(traj_queue);
  }

  printf("%s", RESP_ACK_ESTOP_TRIGGERED);
}

static void handle_status(void) {
  scara_pose_t pose = motion_planner_get_current_pose();
  bool busy = stepper_driver_is_busy();
  trajectory_queue_t *traj_queue = dispatcher_get_queue();
  size_t q_count = traj_queue ? trajectory_queue_count(traj_queue) : 0;
  printf(
      RESP_STATUS_FMT,
      dispatcher_get_state_str(dispatcher_get_state()), busy ? 1 : 0,
      (unsigned int)q_count, pose.x, pose.y, pose.z, pose.phi
  );
}

static void handle_getpos(void) {
  scara_pose_t pose = motion_planner_get_current_pose();
  scara_step_coords_t steps = stepper_driver_get_position();
  printf(
      RESP_POS_FMT,
      pose.x, pose.y, pose.z, pose.phi, (long)steps.j1_steps,
      (long)steps.j2_steps, (long)steps.z_steps, (long)steps.j4_steps
  );
}

static void handle_setpos(const scara_command_t *cmd) {
  motion_planner_set_current_pose(&cmd->data.set_pose);
  printf("%s", RESP_ACK_SETPOS_DONE);
}

static void handle_home(void) {
  if (dispatcher_get_state() == SYSTEM_STATE_ESTOP) {
    printf("%s", RESP_NACK_ESTOP_ACTIVE);
    return;
  }
  dispatcher_set_state(SYSTEM_STATE_HOMING);
  homing_controller_request();
  printf("%s", RESP_ACK_HOMING_STARTED);
}

bool cmd_system_handle(const scara_command_t *cmd) {
  if (!cmd) {
    return false;
  }

  switch (cmd->type) {
  case CMD_TYPE_ENABLE:
    handle_enable();
    return true;

  case CMD_TYPE_DISABLE:
    handle_disable();
    return true;

  case CMD_TYPE_ESTOP:
    handle_estop();
    return true;

  case CMD_TYPE_STATUS:
    handle_status();
    return true;

  case CMD_TYPE_GETPOS:
    handle_getpos();
    return true;

  case CMD_TYPE_SETPOS:
    handle_setpos(cmd);
    return true;

  case CMD_TYPE_HOME:
    handle_home();
    return true;

  default:
    return false;
  }
}
