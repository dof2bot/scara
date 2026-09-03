/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * cmd_motion.c
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
#include "cmd_motion.h"
#include "config/config_storage.h"
#include "dispatcher.h"
#include "kinematics/safety_guard.h"
#include "motion/motion_planner.h"
#include "trajectory_queue.h"
#include <stdio.h>

/* Safety NACK response literals */
static const char RESP_NACK_PATH_CROSSES_DEADZONE[] =
    "<RESP:NACK_PATH_CROSSES_DEADZONE>\n";
static const char RESP_NACK_SINGULARITY_LIMIT[] =
    "<RESP:NACK_SINGULARITY_LIMIT>\n";
static const char RESP_NACK_JOINT_LIMIT[] = "<RESP:NACK_JOINT_LIMIT>\n";
static const char RESP_NACK_Z_OUT_OF_BOUNDS[] =
    "<RESP:NACK_Z_OUT_OF_BOUNDS>\n";
static const char RESP_NACK_OUT_OF_REACH[] = "<RESP:NACK_OUT_OF_REACH>\n";

/* General motion response literals */
static const char RESP_NACK_ESTOP_ACTIVE[] = "<RESP:NACK_ESTOP_ACTIVE>\n";
static const char RESP_NACK_BUFFER_FULL[] = "<RESP:NACK_BUFFER_FULL>\n";
static const char RESP_NACK_INVALID_AXIS[] = "<RESP:NACK_INVALID_AXIS>\n";
static const char RESP_NACK_NOT_HELD[] = "<RESP:NACK_NOT_HELD>\n";

static const char RESP_ACK_QUEUE_FMT[] = "<RESP:ACK#QUEUE=%u>\n";
static const char RESP_ACK_JOG_QUEUED_FMT[] =
    "<RESP:ACK#JOG_QUEUED#QUEUE=%u>\n";
static const char RESP_ACK_FEED_HOLD_ACTIVE[] = "<RESP:ACK#FEED_HOLD_ACTIVE>\n";
static const char RESP_ACK_ALREADY_IDLE[] = "<RESP:ACK#ALREADY_IDLE>\n";
static const char RESP_ACK_MOTION_RESUMED[] = "<RESP:ACK#MOTION_RESUMED>\n";

/* Numerical constants */
static const float DEFAULT_JOG_SPEED = 25.0f;
static const float MIN_JOG_SPEED_THRESHOLD = 20.0f;

/* Jog axis identifiers */
static const char JOG_AXIS_X = 'X';
static const char JOG_AXIS_Y = 'Y';
static const char JOG_AXIS_Z = 'Z';
static const char JOG_AXIS_PHI = 'P';

static void print_safety_nack(safety_status_t s_stat) {
  if (s_stat == SAFETY_ERR_PATH_CROSSES_DEADZONE) {
    printf("%s", RESP_NACK_PATH_CROSSES_DEADZONE);
  } else if (s_stat == SAFETY_ERR_SINGULARITY_OUTER ||
             s_stat == SAFETY_ERR_SINGULARITY_INNER) {
    printf("%s", RESP_NACK_SINGULARITY_LIMIT);
  } else if (s_stat == SAFETY_ERR_JOINT_LIMIT_J1 ||
             s_stat == SAFETY_ERR_JOINT_LIMIT_J2) {
    printf("%s", RESP_NACK_JOINT_LIMIT);
  } else if (s_stat == SAFETY_ERR_Z_LIMIT) {
    printf("%s", RESP_NACK_Z_OUT_OF_BOUNDS);
  } else {
    printf("%s", RESP_NACK_OUT_OF_REACH);
  }
}

static bool validate_pose_and_path(
    const scara_pose_t *curr, const scara_pose_t *target
) {
  const scara_runtime_config_t *cfg = config_storage_get();
  scara_geometry_t geo = {cfg->l1, cfg->l2};

  scara_elbow_config_t elbow = motion_planner_get_elbow_config();
  safety_status_t s_stat =
      safety_guard_check_pose(&geo, target, elbow, NULL);
  if (s_stat != SAFETY_STATUS_OK) {
    print_safety_nack(s_stat);
    return false;
  }

  safety_status_t path_stat = safety_guard_check_path(&geo, curr, target);
  if (path_stat != SAFETY_STATUS_OK) {
    print_safety_nack(path_stat);
    return false;
  }

  return true;
}

static void handle_waypoint(const scara_command_t *cmd) {
  if (dispatcher_get_state() == SYSTEM_STATE_ESTOP) {
    printf("%s", RESP_NACK_ESTOP_ACTIVE);
    return;
  }

  scara_pose_t target_pose = {
      .x = cmd->data.waypoint.x,
      .y = cmd->data.waypoint.y,
      .z = cmd->data.waypoint.z,
      .phi = cmd->data.waypoint.phi
  };

  scara_pose_t curr_p = motion_planner_get_current_pose();
  if (!validate_pose_and_path(&curr_p, &target_pose)) {
    return;
  }

  trajectory_queue_t *traj_queue = dispatcher_get_queue();
  if (traj_queue && trajectory_queue_push(traj_queue, &cmd->data.waypoint)) {
    printf(
        RESP_ACK_QUEUE_FMT,
        (unsigned int)trajectory_queue_count(traj_queue)
    );
  } else {
    printf("%s", RESP_NACK_BUFFER_FULL);
  }
}

static void handle_jog(const scara_command_t *cmd) {
  if (dispatcher_get_state() == SYSTEM_STATE_ESTOP) {
    printf("%s", RESP_NACK_ESTOP_ACTIVE);
    return;
  }

  scara_pose_t curr = motion_planner_get_current_pose();
  scara_pose_t target = curr;
  char axis = cmd->data.jog.axis;
  float step = cmd->data.jog.step;

  if (axis == JOG_AXIS_X) {
    target.x += step;
  } else if (axis == JOG_AXIS_Y) {
    target.y += step;
  } else if (axis == JOG_AXIS_Z) {
    target.z += step;
  } else if (axis == JOG_AXIS_PHI) {
    target.phi += step;
  } else {
    printf("%s", RESP_NACK_INVALID_AXIS);
    return;
  }

  if (!validate_pose_and_path(&curr, &target)) {
    return;
  }

  const scara_runtime_config_t *cfg = config_storage_get();
  scara_waypoint_t wp = {
      .x = target.x,
      .y = target.y,
      .z = target.z,
      .phi = target.phi,
      .speed = cfg->min_speed > MIN_JOG_SPEED_THRESHOLD
                   ? cfg->min_speed
                   : DEFAULT_JOG_SPEED
  };

  trajectory_queue_t *traj_queue = dispatcher_get_queue();
  if (traj_queue && trajectory_queue_push(traj_queue, &wp)) {
    printf(
        RESP_ACK_JOG_QUEUED_FMT,
        (unsigned int)trajectory_queue_count(traj_queue)
    );
  } else {
    printf("%s", RESP_NACK_BUFFER_FULL);
  }
}

static void handle_hold(void) {
  if (dispatcher_get_state() == SYSTEM_STATE_RUNNING) {
    dispatcher_set_state(SYSTEM_STATE_HOLD);
    printf("%s", RESP_ACK_FEED_HOLD_ACTIVE);
  } else {
    printf("%s", RESP_ACK_ALREADY_IDLE);
  }
}

static void handle_resume(void) {
  if (dispatcher_get_state() == SYSTEM_STATE_HOLD) {
    dispatcher_set_state(SYSTEM_STATE_RUNNING);
    printf("%s", RESP_ACK_MOTION_RESUMED);
  } else {
    printf("%s", RESP_NACK_NOT_HELD);
  }
}

bool cmd_motion_handle(const scara_command_t *cmd) {
  if (!cmd) {
    return false;
  }

  switch (cmd->type) {
  case CMD_TYPE_WAYPOINT:
    handle_waypoint(cmd);
    return true;

  case CMD_TYPE_JOG:
    handle_jog(cmd);
    return true;

  case CMD_TYPE_HOLD:
    handle_hold();
    return true;

  case CMD_TYPE_RESUME:
    handle_resume();
    return true;

  default:
    return false;
  }
}
