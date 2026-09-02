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
#include "scara_config.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
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
  cfg->l1 = SCARA_ARM_L1_MM;
  cfg->l2 = SCARA_ARM_L2_MM;
  cfg->z_min = SCARA_Z_MIN_MM;
  cfg->z_max = SCARA_Z_MAX_MM;
  cfg->min_speed = 1.0f;
  cfg->max_speed = SCARA_MAX_SPEED_MM_S;
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
