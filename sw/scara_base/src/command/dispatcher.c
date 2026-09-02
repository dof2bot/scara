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
#include "config/config_storage.h"
#include "kinematics/scara_ik.h"
#include "motion/motion_planner.h"
#include "stepper/stepper_driver.h"
#include <stdio.h>

static trajectory_queue_t *traj_queue = NULL;
static scara_system_state_t system_state = SYSTEM_STATE_INIT;

void dispatcher_init(trajectory_queue_t *queue) {
  traj_queue = queue;
  system_state = SYSTEM_STATE_IDLE;
  printf("<RESP:INIT_OK#STATE=IDLE>\n");
}

scara_system_state_t dispatcher_get_state(void) {
  return system_state;
}

void dispatcher_dispatch(const scara_command_t *cmd) {
  if (!cmd) {
    return;
  }

  switch (cmd->type) {
  case CMD_TYPE_WAYPOINT: {
    if (system_state == SYSTEM_STATE_ESTOP) {
      printf("<RESP:NACK_ESTOP_ACTIVE>\n");
      return;
    }

    /* Validate reachability dynamically before queueing */
    const scara_runtime_config_t *cfg = config_storage_get();
    scara_geometry_t geo = {cfg->l1, cfg->l2};

    if (!scara_ik_is_reachable(
            &geo, cmd->data.waypoint.x, cmd->data.waypoint.y
        )) {
      printf("<RESP:NACK_OUT_OF_REACH>\n");
      return;
    }

    if (cmd->data.waypoint.z < cfg->z_min || cmd->data.waypoint.z > cfg->z_max) {
      printf("<RESP:NACK_Z_OUT_OF_BOUNDS>\n");
      return;
    }

    if (traj_queue && trajectory_queue_push(traj_queue, &cmd->data.waypoint)) {
      printf(
          "<RESP:ACK#QUEUE=%u>\n",
          (unsigned int)trajectory_queue_count(traj_queue)
      );
    } else {
      printf("<RESP:NACK_BUFFER_FULL>\n");
    }
    break;
  }

  case CMD_TYPE_ENABLE: {
    stepper_driver_set_enabled(true);

    if (system_state == SYSTEM_STATE_ESTOP) {
      system_state = SYSTEM_STATE_IDLE;
    }

    printf("<RESP:ACK#MOTORS_ENABLED>\n");
    break;
  }

  case CMD_TYPE_DISABLE: {
    stepper_driver_set_enabled(false);
    printf("<RESP:ACK#MOTORS_DISABLED>\n");
    break;
  }

  case CMD_TYPE_ESTOP: {
    system_state = SYSTEM_STATE_ESTOP;
    stepper_driver_emergency_stop();

    if (traj_queue) {
      trajectory_queue_clear(traj_queue);
    }

    printf("<RESP:ACK#ESTOP_TRIGGERED>\n");
    break;
  }

  case CMD_TYPE_STATUS: {
    const char *state_str = "UNKNOWN";

    switch (system_state) {
    case SYSTEM_STATE_IDLE:
      state_str = "IDLE";
      break;
    case SYSTEM_STATE_RUNNING:
      state_str = "RUNNING";
      break;
    case SYSTEM_STATE_ESTOP:
      state_str = "ESTOP";
      break;
    case SYSTEM_STATE_HOMING:
      state_str = "HOMING";
      break;
    default:
      break;
    }

    scara_pose_t pose = motion_planner_get_current_pose();
    bool busy = stepper_driver_is_busy();
    size_t q_count = traj_queue ? trajectory_queue_count(traj_queue) : 0;
    printf(
        "<STATUS#STATE=%s#BUSY=%d#Q=%u#X=%.2f#Y=%.2f#Z=%.2f#PHI=%.2f>\n",
        state_str, busy ? 1 : 0, (unsigned int)q_count, pose.x, pose.y, pose.z,
        pose.phi
    );
    break;
  }

  case CMD_TYPE_GETPOS: {
    scara_pose_t pose = motion_planner_get_current_pose();
    scara_step_coords_t steps = stepper_driver_get_position();
    printf(
        "<POS#X=%.2f#Y=%.2f#Z=%.2f#PHI=%.2f#J1=%ld#J2=%ld#Z_STEP=%ld#J4=%ld>\n",
        pose.x, pose.y, pose.z, pose.phi, (long)steps.j1_steps,
        (long)steps.j2_steps, (long)steps.z_steps, (long)steps.j4_steps
    );
    break;
  }

  case CMD_TYPE_SETPOS: {
    motion_planner_set_current_pose(&cmd->data.set_pose);
    printf("<RESP:ACK#SETPOS_DONE>\n");
    break;
  }

  case CMD_TYPE_HOME: {
    const scara_runtime_config_t *cfg = config_storage_get();
    scara_waypoint_t home_wp = {
        .x = (cfg->l1 + cfg->l2) * 0.65f,
        .y = 0.0f,
        .z = cfg->z_min + 20.0f,
        .phi = 0.0f,
        .speed = SCARA_DEFAULT_SPEED_MM_S
    };

    if (traj_queue && trajectory_queue_push(traj_queue, &home_wp)) {
      printf("<RESP:ACK#HOMING_STARTED>\n");
    } else {
      printf("<RESP:ACK#HOMED>\n");
    }

    break;
  }

  case CMD_TYPE_GET_CONFIG: {
    const scara_runtime_config_t *cfg = config_storage_get();
    printf(
        "<RESP:CONFIG#L1=%.2f#L2=%.2f#Z_MIN=%.2f#Z_MAX=%.2f#MIN_SPEED=%.2f#MAX_SPEED=%.2f>\n",
        cfg->l1, cfg->l2, cfg->z_min, cfg->z_max, cfg->min_speed, cfg->max_speed
    );
    break;
  }

  case CMD_TYPE_SET_CONFIG: {
    config_storage_set(&cmd->data.config);
    printf("<RESP:ACK#CONFIG_UPDATED>\n");
    break;
  }

  case CMD_TYPE_SAVE_CONFIG: {
    if (config_storage_save_flash()) {
      printf("<RESP:ACK#CONFIG_SAVED>\n");
    } else {
      printf("<RESP:NACK_CONFIG_SAVE_FAILED>\n");
    }
    break;
  }

  case CMD_TYPE_RESET_CONFIG: {
    config_storage_reset_defaults();
    printf("<RESP:ACK#CONFIG_RESET>\n");
    break;
  }

  default:
    printf("<RESP:NACK_UNKNOWN_CMD>\n");
    break;
  }
}

void dispatcher_tick(void) {
  if (system_state == SYSTEM_STATE_ESTOP) {
    return;
  }

  bool busy = stepper_driver_is_busy();
  bool has_queued_points = traj_queue && !trajectory_queue_is_empty(traj_queue);

  if (busy || has_queued_points) {
    system_state = SYSTEM_STATE_RUNNING;
  } else {
    system_state = SYSTEM_STATE_IDLE;
  }
}
