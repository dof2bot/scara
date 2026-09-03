/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * homing_controller.h
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

#include "scara_config.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  HOMING_STATE_IDLE = 0,
  HOMING_STATE_Z,
  HOMING_STATE_J2,
  HOMING_STATE_J1,
  HOMING_STATE_SUCCESS,
  HOMING_STATE_FAILED
} homing_state_t;

/**
 * @brief Requests homing procedure to be executed by motion worker.
 */
void homing_controller_request(void);

/**
 * @brief Checks if a homing sequence has been requested or is in progress.
 * @return true if requested/busy, false if idle.
 */
bool homing_controller_is_requested(void);

/**
 * @brief Executes the sequential SCARA homing routine (Z -> J2 -> J1).
 *
 * 1. Drives Z axis towards top limit switch (Z safety travel).
 * 2. Drives J2 (elbow) towards Y limit switch.
 * 3. Drives J1 (shoulder) towards X limit switch.
 * 4. Calibrates step counts and sets absolute Cartesian pose.
 *
 * @return true on successful calibration, false if a switch failed to trigger.
 */
bool homing_controller_run(void);

/**
 * @brief Returns the current state of the homing routine.
 * @return homing_state_t Current homing state.
 */
homing_state_t homing_controller_get_state(void);

#ifdef __cplusplus
}
#endif
