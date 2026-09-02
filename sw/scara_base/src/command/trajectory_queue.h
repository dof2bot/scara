/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * trajectory_queue.h
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

#include "motion/motion_planner.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TRAJECTORY_QUEUE_CAPACITY 32

/**
 * @brief Thread-safe circular queue for trajectory waypoints between Core 0 and
 *        Core 1. Data memory barriers are used to ensure thread safety, since
 *        atomic operations are not available on the RP2040 micro-controller.
 */
typedef struct {
  scara_waypoint_t waypoints[TRAJECTORY_QUEUE_CAPACITY];
  volatile size_t head;
  volatile size_t tail;
  volatile size_t count;
} trajectory_queue_t;

/**
 * @brief Initializes circular queue.
 * @param q Pointer to queue structure.
 */
void trajectory_queue_init(trajectory_queue_t *q);

/**
 * @brief Checks if queue is full.
 * @param q Pointer to queue.
 * @return true if full, false otherwise.
 */
bool trajectory_queue_is_full(const trajectory_queue_t *q);

/**
 * @brief Checks if queue is empty.
 * @param q Pointer to queue.
 * @return true if empty, false otherwise.
 */
bool trajectory_queue_is_empty(const trajectory_queue_t *q);

/**
 * @brief Pushes a new waypoint to the queue.
 * @param q Pointer to queue.
 * @param wp Waypoint to push.
 * @return true on success, false if queue was full.
 */
bool trajectory_queue_push(trajectory_queue_t *q, const scara_waypoint_t *wp);

/**
 * @brief Pops the next waypoint from the queue.
 * @param q Pointer to queue.
 * @param wp Pointer to store popped waypoint.
 * @return true on success, false if queue was empty.
 */
bool trajectory_queue_pop(trajectory_queue_t *q, scara_waypoint_t *wp);

/**
 * @brief Clears all pending waypoints in the queue.
 * @param q Pointer to queue.
 */
void trajectory_queue_clear(trajectory_queue_t *q);

/**
 * @brief Returns the number of items currently in the queue.
 * @param q Pointer to queue.
 * @return size_t Count of items.
 */
size_t trajectory_queue_count(const trajectory_queue_t *q);

#ifdef __cplusplus
}
#endif
