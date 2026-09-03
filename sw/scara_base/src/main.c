/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * main.c
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
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include "command/dispatcher.h"
#include "command/parser.h"
#include "command/trajectory_queue.h"
#include "config/config_storage.h"
#include "config/scara_config.h"
#include "homing/homing_controller.h"
#include "io/io_gpio.h"
#include "kinematics/scara_ik.h"
#include "motion/motion_planner.h"
#include "stepper/stepper_driver.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static const uint32_t USB_CONN_CHECK_MS = 100;
static const uint32_t HEARTBEAT_INTERVAL_MS = 500;

/* Motion worker serial response literals */
static const char RESP_HOMING_IN_PROGRESS[] = "<RESP:HOMING_IN_PROGRESS>\n";
static const char RESP_HOMED_SUCCESS_FMT[] =
    "<RESP:HOMED_SUCCESS#X=%.2f#Y=%.2f#Z=%.2f#PHI=%.2f>\n";
static const char RESP_HOMING_FAILED[] = "<RESP:HOMING_FAILED>\n";

static const char RESP_MOVE_START_FMT[] =
    "<RESP:MOVE_START#X=%.2f#Y=%.2f#Z=%.2f#PHI=%.2f>\n";
static const char RESP_MOVE_DONE_FMT[] =
    "<RESP:MOVE_DONE#X=%.2f#Y=%.2f#Z=%.2f#PHI=%.2f>\n";
static const char RESP_MOVE_FAILED[] = "<RESP:MOVE_FAILED>\n";

/* Shared Ring Buffer for waypoints between Core 0 and Core 1 */
static trajectory_queue_t global_trajectory_queue;

/**
 * @brief Core 1 worker thread: Dedicated mathematical co-processor & motion
 * executor.
 */
static void core1_motion_worker(void) {
  scara_waypoint_t target_wp;

  while (true) {
    /* 1. Handle Homing sequence if requested */
    if (homing_controller_is_requested()) {
      printf("%s", RESP_HOMING_IN_PROGRESS);
      bool home_ok = homing_controller_run();
      if (home_ok) {
        scara_pose_t home_p = motion_planner_get_current_pose();
        printf(
            RESP_HOMED_SUCCESS_FMT, home_p.x, home_p.y, home_p.z, home_p.phi
        );
      } else {
        printf("%s", RESP_HOMING_FAILED);
      }
    }

    /* 2. Process waypoints if system state allows motion */
    scara_system_state_t cur_state = dispatcher_get_state();
    if (cur_state != SYSTEM_STATE_ESTOP && cur_state != SYSTEM_STATE_HOLD &&
        cur_state != SYSTEM_STATE_HOMING) {
      if (trajectory_queue_pop(&global_trajectory_queue, &target_wp)) {
        /* Enable steppers automatically on motion if not already enabled */
        if (!stepper_driver_is_enabled()) {
          stepper_driver_set_enabled(true);
        }

        printf(
            RESP_MOVE_START_FMT, target_wp.x, target_wp.y, target_wp.z,
            target_wp.phi
        );

        /* Execute linear Cartesian motion with analytical IK */
        bool ok = motion_planner_move_linear(&target_wp);

        if (ok) {
          stepper_driver_wait_idle();
          scara_pose_t final_p = motion_planner_get_current_pose();
          printf(
              RESP_MOVE_DONE_FMT, final_p.x, final_p.y, final_p.z,
              final_p.phi
          );
        } else {
          printf("%s", RESP_MOVE_FAILED);
        }
      }
    }

    tight_loop_contents();
  }
}

/**
 * @brief Initializes all hardware peripherals and software subsystems.
 * @return true on success, false on failure.
 */
static bool device_init(void) {
  if (!stdio_init_all()) {
    return false;
  }

  config_storage_init();

  if (!io_gpio_init()) {
    return false;
  }

  if (!stepper_driver_init()) {
    return false;
  }

  const scara_runtime_config_t *cfg = config_storage_get();
  motion_planner_init(cfg->l1, cfg->l2);
  trajectory_queue_init(&global_trajectory_queue);
  parser_init();
  dispatcher_init(&global_trajectory_queue);

  return true;
}

/**
 * @brief Main entry point for scara-base running on Core 0.
 * @return 0 on exit, 1 on failure.
 */
int main(void) {
  /* Initialize devices and subsystems */
  if (!device_init()) {
    return 1;
  }

  /* Launch Core 1 dedicated motion task */
  multicore_launch_core1(core1_motion_worker);

  /* Optional: Wait for USB serial connection if desired */
  uint32_t last_heartbeat = to_ms_since_boot(get_absolute_time());
  scara_command_t received_cmd;

  while (true) {
    /* 1. Poll and process incoming serial commands */
    if (parser_poll(&received_cmd)) {
      dispatcher_dispatch(&received_cmd);
    }

    /* 2. Dispatcher periodic state check */
    dispatcher_tick();

    /* 3. Non-blocking heartbeat LED */
    uint32_t now = to_ms_since_boot(get_absolute_time());

    if (now - last_heartbeat >= HEARTBEAT_INTERVAL_MS) {
      last_heartbeat = now;
      io_gpio_toggle_led();
    }

    tight_loop_contents();
  }

  return 0;
}
