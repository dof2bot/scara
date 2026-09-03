/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * trajectory_chunk.h
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

#include "kinematics/scara_ik.h"
#include "scara_config.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generates a DMA command chunk for PIO from segment [p1 -> p2].
 *
 * @param geometry Pointer to arm geometry (L1, L2).
 * @param elbow_config Active elbow configuration.
 * @param p1 Starting pose.
 * @param p2 Ending pose.
 * @param speed Feedrate (mm/s).
 * @param out_buffer Output buffer for 32-bit PIO words.
 * @param max_words Capacity of out_buffer.
 * @param out_count Pointer to store generated word count.
 * @return true on success, false if unreachable or buffer overflow.
 */
bool trajectory_chunk_generate(
    const scara_geometry_t *geometry, scara_elbow_config_t elbow_config,
    const scara_pose_t *p1, const scara_pose_t *p2, float speed,
    uint32_t *out_buffer, size_t max_words, size_t *out_count
);

#ifdef __cplusplus
}
#endif
