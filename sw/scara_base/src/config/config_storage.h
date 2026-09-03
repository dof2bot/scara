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
#define SCARA_CONFIG_VERSION 2

typedef struct {
  uint32_t magic;
  uint32_t version;

  /* 1. Arm Physical Geometry (mm) */
  float l1;
  float l2;
  float z_min;
  float z_max;

  /* 2. Speeds & Acceleration Dynamics (mm/s, mm/s^2) */
  float min_speed;
  float max_speed;
  float default_speed;
  float default_accel;
  float max_accel;

  /* 3. Homing Calibration & Offsets (rad, Hz) */
  float home_offset_j1;
  float home_offset_j2;
  uint32_t homing_rate_hz;

  /* 4. Software Joint Angular Limits (rad) */
  float j1_min_rad;
  float j1_max_rad;
  float j2_min_rad;
  float j2_max_rad;

  /* 5. Stepper Transmission & Lead Parameters */
  float gear_ratio_j1;
  float gear_ratio_j2;
  float gear_ratio_j4;
  float leadscrew_pitch_z;

  uint32_t checksum;
} scara_runtime_config_t;

void config_storage_init(void);
const scara_runtime_config_t *config_storage_get(void);
void config_storage_set(const scara_runtime_config_t *new_cfg);
bool config_storage_save_flash(void);
void config_storage_reset_defaults(void);

float config_storage_get_steps_per_rad_j1(void);
float config_storage_get_steps_per_rad_j2(void);
float config_storage_get_steps_per_rad_j4(void);
float config_storage_get_steps_per_mm_z(void);
