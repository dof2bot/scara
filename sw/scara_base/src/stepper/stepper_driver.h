/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * stepper_driver.h
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
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Discrete motor step coordinates.
 */
typedef struct {
  int32_t j1_steps; /* Shoulder step position */
  int32_t j2_steps; /* Elbow step position */
  int32_t z_steps;  /* Z-axis step position */
  int32_t j4_steps; /* Wrist step position */
} scara_step_coords_t;

/**
 * @brief Initializes GPIOs, PIO and DMA channels for stepper driver.
 * @return true on success, false on error.
 */
bool stepper_driver_init(void);

/**
 * @brief Enables or disables motor driver power stages (via ENABLE pin).
 * @param enable true to enable motors, false to freewheel (save power/heat).
 */
void stepper_driver_set_enabled(bool enable);

/**
 * @brief Checks if stepper motor power stage is currently enabled.
 * @return true if enabled, false otherwise.
 */
bool stepper_driver_is_enabled(void);

/**
 * @brief Returns the current step position counters for all 4 axes.
 * @return scara_step_coords_t Current axis steps.
 */
scara_step_coords_t stepper_driver_get_position(void);

/**
 * @brief Overwrites internal step position counters (e.g. after homing).
 * @param pos New step coordinates.
 */
void stepper_driver_set_position(const scara_step_coords_t *pos);

/**
 * @brief Immediately stops any active motion and aborts execution.
 */
void stepper_driver_emergency_stop(void);

/**
 * @brief Checks if a motion sequence is currently in progress.
 * @return true if motors are moving, false if idle.
 */
bool stepper_driver_is_busy(void);

/**
 * @brief Submits a chunk of 32-bit PIO step/dir command words to the hardware
 * queue.
 *
 * @param commands Pointer to array of formatted PIO command words.
 * @param count Number of command words.
 * @return true if successfully queued/started, false if busy.
 */
bool stepper_driver_submit_chunk(const uint32_t *commands, size_t count);

/**
 * @brief Checks if the next ping-pong DMA buffer is free to receive new data.
 * @return true if ready, false if busy.
 */
bool stepper_driver_is_buffer_available(void);

/**
 * @brief Blocks until the next ping-pong buffer is ready and queues the chunk.
 *
 * @param commands Pointer to array of formatted PIO command words.
 * @param count Number of command words.
 * @return true if successfully queued, false on error.
 */
bool stepper_driver_stream_chunk(const uint32_t *commands, size_t count);

/**
 * @brief Blocks until all queued DMA chunks and PIO pulses complete.
 */
void stepper_driver_wait_idle(void);

/**
 * @brief Executes a single synchronized multi-axis move (blocking / polling
 * mode).
 *
 * @param target_steps Target step coordinates for all 4 axes.
 * @param step_rate_hz Base step pulse frequency (Hz).
 */
void stepper_driver_move_steps_sync(
    const scara_step_coords_t *target_steps, uint32_t step_rate_hz
);

#ifdef __cplusplus
}
#endif
