/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * trajectory_queue.c
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

#include "trajectory_queue.h"
#include "pico/sync.h"
#include <string.h>

void trajectory_queue_init(trajectory_queue_t *q) {
  if (!q) {
    return;
  }

  q->head = 0;
  q->tail = 0;
  q->count = 0;
  memset(q->waypoints, 0, sizeof(q->waypoints));
}

bool trajectory_queue_is_full(const trajectory_queue_t *q) {
  if (!q) {
    return true;
  }

  return q->count >= TRAJECTORY_QUEUE_CAPACITY;
}

bool trajectory_queue_is_empty(const trajectory_queue_t *q) {
  if (!q) {
    return true;
  }

  return q->count == 0;
}

bool trajectory_queue_push(trajectory_queue_t *q, const scara_waypoint_t *wp) {
  if (!q || !wp) {
    return false;
  }

  if (q->count >= TRAJECTORY_QUEUE_CAPACITY) {
    return false;
  }

  q->waypoints[q->head] = *wp;
  q->head = (q->head + 1) % TRAJECTORY_QUEUE_CAPACITY;

  /*
   * Data memory barrier ensures that the data is written to memory before the
   * count is incremented (q->count++); otherwise the other core may read
   * the new head index but the data at that index may not be written yet.
   * This is required because the queue is shared between two cores.
   */
  __dmb();
  q->count++;

  return true;
}

bool trajectory_queue_pop(trajectory_queue_t *q, scara_waypoint_t *wp) {
  if (!q || !wp) {
    return false;
  }

  if (q->count == 0) {
    return false;
  }

  *wp = q->waypoints[q->tail];
  q->tail = (q->tail + 1) % TRAJECTORY_QUEUE_CAPACITY;

  /*
   * Data memory barrier ensures that the data is written to memory before the
   * count is decremented (q->count--); otherwise the other core may read
   * the new tail index but the data at that index may not be written yet.
   * This is required because the queue is shared between two cores.
   */
  __dmb();
  q->count--;

  return true;
}

void trajectory_queue_clear(trajectory_queue_t *q) {
  if (!q) {
    return;
  }

  q->head = 0;
  q->tail = 0;
  q->count = 0;
}

size_t trajectory_queue_count(const trajectory_queue_t *q) {
  return q ? q->count : 0;
}
