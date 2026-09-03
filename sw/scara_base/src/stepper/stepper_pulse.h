/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * stepper_pulse.h
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

#include "stepper_driver.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Executes a synchronized multi-axis Bresenham stepping move.
 *
 * @param target_steps Target step coordinates for all 4 axes.
 * @param current_pos Pointer to current position coordinates to be updated.
 * @param step_rate_hz Base step pulse frequency (Hz).
 */
void stepper_pulse_move_sync(
    const scara_step_coords_t *target_steps, scara_step_coords_t *current_pos,
    uint32_t step_rate_hz
);

#ifdef __cplusplus
}
#endif
