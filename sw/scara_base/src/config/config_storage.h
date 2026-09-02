/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * config_storage.h
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

#include <stdbool.h>
#include <stdint.h>

#define SCARA_CONFIG_MAGIC 0x53434152 /* "SCAR" */
#define SCARA_CONFIG_VERSION 1

typedef struct {
  uint32_t magic;
  uint32_t version;
  float l1;
  float l2;
  float z_min;
  float z_max;
  float min_speed;
  float max_speed;
  uint32_t checksum;
} scara_runtime_config_t;

void config_storage_init(void);
const scara_runtime_config_t *config_storage_get(void);
void config_storage_set(const scara_runtime_config_t *new_cfg);
bool config_storage_save_flash(void);
void config_storage_reset_defaults(void);
