/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * config_storage.c
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
#include "config_storage.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "scara_config.h"
#include <math.h>
#include <string.h>

#define FLASH_CONFIG_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

static scara_runtime_config_t active_config;

static uint32_t calculate_checksum(const scara_runtime_config_t *cfg) {
  const uint8_t *p = (const uint8_t *)cfg;
  size_t len = sizeof(scara_runtime_config_t) - sizeof(uint32_t);
  uint32_t sum = 0;

  for (size_t i = 0; i < len; ++i) {
    sum = (sum * 31) + p[i];
  }

  return sum;
}

static void load_hardcoded_defaults(scara_runtime_config_t *cfg) {
  cfg->magic = SCARA_CONFIG_MAGIC;
  cfg->version = SCARA_CONFIG_VERSION;

  /* 1. Physical Geometry */
  cfg->l1 = SCARA_ARM_L1_MM;
  cfg->l2 = SCARA_ARM_L2_MM;
  cfg->z_min = SCARA_Z_MIN_MM;
  cfg->z_max = SCARA_Z_MAX_MM;

  /* 2. Speeds & Acceleration Dynamics */
  cfg->min_speed = SCARA_MIN_SPEED_MM_S;
  cfg->max_speed = SCARA_MAX_SPEED_MM_S;
  cfg->default_speed = SCARA_DEFAULT_SPEED_MM_S;
  cfg->default_accel = SCARA_DEFAULT_ACCEL_MM_S2;
  cfg->max_accel = SCARA_MAX_ACCEL_MM_S2;

  /* 3. Homing Calibration & Offsets */
  cfg->home_offset_j1 = SCARA_HOME_OFFSET_J1_RAD;
  cfg->home_offset_j2 = SCARA_HOME_OFFSET_J2_RAD;
  cfg->homing_rate_hz = SCARA_HOMING_STEP_RATE_HZ;

  /* 4. Software Joint Angular Limits */
  cfg->j1_min_rad = SCARA_J1_MIN_RAD;
  cfg->j1_max_rad = SCARA_J1_MAX_RAD;
  cfg->j2_min_rad = SCARA_J2_MIN_RAD;
  cfg->j2_max_rad = SCARA_J2_MAX_RAD;

  /* 5. Stepper Transmission & Lead Parameters */
  cfg->gear_ratio_j1 = SCARA_GEAR_RATIO_J1;
  cfg->gear_ratio_j2 = SCARA_GEAR_RATIO_J2;
  cfg->gear_ratio_j4 = SCARA_GEAR_RATIO_J4;
  cfg->leadscrew_pitch_z = SCARA_LEADSCREW_PITCH_Z;

  cfg->checksum = calculate_checksum(cfg);
}

void config_storage_init(void) {
  const scara_runtime_config_t *flash_cfg =
      (const scara_runtime_config_t *)(XIP_BASE + FLASH_CONFIG_OFFSET);

  if (flash_cfg->magic == SCARA_CONFIG_MAGIC &&
      flash_cfg->version == SCARA_CONFIG_VERSION) {
    uint32_t expected = calculate_checksum(flash_cfg);

    if (flash_cfg->checksum == expected) {
      memcpy(&active_config, flash_cfg, sizeof(scara_runtime_config_t));
      return;
    }
  }

  load_hardcoded_defaults(&active_config);
}

const scara_runtime_config_t *config_storage_get(void) {
  return &active_config;
}

void config_storage_set(const scara_runtime_config_t *new_cfg) {
  if (!new_cfg) {
    return;
  }

  active_config = *new_cfg;
  active_config.magic = SCARA_CONFIG_MAGIC;
  active_config.version = SCARA_CONFIG_VERSION;
  active_config.checksum = calculate_checksum(&active_config);
}

bool config_storage_save_flash(void) {
  active_config.checksum = calculate_checksum(&active_config);

  uint8_t page_buf[FLASH_PAGE_SIZE];
  memset(page_buf, 0xFF, sizeof(page_buf));
  memcpy(page_buf, &active_config, sizeof(scara_runtime_config_t));

  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(FLASH_CONFIG_OFFSET, FLASH_SECTOR_SIZE);
  flash_range_program(FLASH_CONFIG_OFFSET, page_buf, FLASH_PAGE_SIZE);
  restore_interrupts(ints);

  return true;
}

void config_storage_reset_defaults(void) {
  load_hardcoded_defaults(&active_config);
  config_storage_save_flash();
}

float config_storage_get_steps_per_rad_j1(void) {
  float gr = (active_config.gear_ratio_j1 > 0.01f) ? active_config.gear_ratio_j1
                                                   : SCARA_GEAR_RATIO_J1;
  return (SCARA_STEPS_PER_REV * SCARA_MICROSTEPPING * gr) /
         (2.0f * (float)M_PI);
}

float config_storage_get_steps_per_rad_j2(void) {
  float gr = (active_config.gear_ratio_j2 > 0.01f) ? active_config.gear_ratio_j2
                                                   : SCARA_GEAR_RATIO_J2;
  return (SCARA_STEPS_PER_REV * SCARA_MICROSTEPPING * gr) /
         (2.0f * (float)M_PI);
}

float config_storage_get_steps_per_rad_j4(void) {
  float gr = (active_config.gear_ratio_j4 > 0.01f) ? active_config.gear_ratio_j4
                                                   : SCARA_GEAR_RATIO_J4;
  return (SCARA_STEPS_PER_REV * SCARA_MICROSTEPPING * gr) /
         (2.0f * (float)M_PI);
}

float config_storage_get_steps_per_mm_z(void) {
  float pitch = (active_config.leadscrew_pitch_z > 0.01f)
                    ? active_config.leadscrew_pitch_z
                    : SCARA_LEADSCREW_PITCH_Z;
  return (SCARA_STEPS_PER_REV * SCARA_MICROSTEPPING) / pitch;
}
