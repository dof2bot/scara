/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * stepper_driver.c
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
#include "stepper_driver.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "stepper_multi.pio.h"
#include <stdlib.h>
#include <string.h>

#define PING_PONG_BUFFER_SIZE SCARA_MAX_SEGMENTS_CHUNK

static PIO stepper_pio = pio0;
static uint stepper_sm = 0;
static uint stepper_offset = 0;

static int dma_chan_ping = -1;
static int dma_chan_pong = -1;

static uint32_t ping_buffer[PING_PONG_BUFFER_SIZE];
static uint32_t pong_buffer[PING_PONG_BUFFER_SIZE];

static volatile scara_step_coords_t current_pos = {0, 0, 0, 0};
static volatile bool is_enabled = false;
static volatile bool is_busy = false;

bool stepper_driver_init(void) {
  /* Initialize Enable pin */
  gpio_init(SCARA_ENABLE_PIN);
  gpio_set_dir(SCARA_ENABLE_PIN, GPIO_OUT);
  stepper_driver_set_enabled(false);

  /* Initialize GPIOs for step/dir */
  const uint step_pins[] = {
      SCARA_J1_STEP_PIN, SCARA_J2_STEP_PIN, SCARA_Z_STEP_PIN, SCARA_J4_STEP_PIN
  };
  const uint dir_pins[] = {
      SCARA_J1_DIR_PIN, SCARA_J2_DIR_PIN, SCARA_Z_DIR_PIN, SCARA_J4_DIR_PIN
  };

  for (size_t i = 0; i < 4; i++) {
    gpio_init(step_pins[i]);
    gpio_set_dir(step_pins[i], GPIO_OUT);
    gpio_put(step_pins[i], 0);

    gpio_init(dir_pins[i]);
    gpio_set_dir(dir_pins[i], GPIO_OUT);
    gpio_put(dir_pins[i], 0);
  }

  /* Load PIO program */
  if (!pio_can_add_program(stepper_pio, &stepper_multi_program)) {
    return false;
  }

  stepper_offset = pio_add_program(stepper_pio, &stepper_multi_program);
  stepper_multi_program_init(
      stepper_pio, stepper_sm, stepper_offset, SCARA_J1_DIR_PIN,
      SCARA_J1_STEP_PIN
  );

  /* Configure Chained DMA for Ping-Pong Double Buffering */
  dma_chan_ping = dma_claim_unused_channel(true);
  dma_chan_pong = dma_claim_unused_channel(true);

  if (dma_chan_ping < 0 || dma_chan_pong < 0) {
    return false;
  }

  dma_channel_config cfg_ping = dma_channel_get_default_config(dma_chan_ping);
  channel_config_set_transfer_data_size(&cfg_ping, DMA_SIZE_32);
  channel_config_set_read_increment(&cfg_ping, true);
  channel_config_set_write_increment(&cfg_ping, false);
  channel_config_set_dreq(
      &cfg_ping, pio_get_dreq(stepper_pio, stepper_sm, true)
  );
  channel_config_set_chain_to(&cfg_ping, (uint)dma_chan_pong);

  dma_channel_configure(
      dma_chan_ping, &cfg_ping, &stepper_pio->txf[stepper_sm], ping_buffer,
      PING_PONG_BUFFER_SIZE, false
  );

  dma_channel_config cfg_pong = dma_channel_get_default_config(dma_chan_pong);
  channel_config_set_transfer_data_size(&cfg_pong, DMA_SIZE_32);
  channel_config_set_read_increment(&cfg_pong, true);
  channel_config_set_write_increment(&cfg_pong, false);
  channel_config_set_dreq(
      &cfg_pong, pio_get_dreq(stepper_pio, stepper_sm, true)
  );
  channel_config_set_chain_to(&cfg_pong, (uint)dma_chan_ping);

  dma_channel_configure(
      dma_chan_pong, &cfg_pong, &stepper_pio->txf[stepper_sm], pong_buffer,
      PING_PONG_BUFFER_SIZE, false
  );

  return true;
}

void stepper_driver_set_enabled(bool enable) {
  is_enabled = enable;
  /* Active LOW: 0 enables motor coils, 1 disables (freewheel) */
  gpio_put(SCARA_ENABLE_PIN, enable ? 0 : 1);
}

bool stepper_driver_is_enabled(void) {
  return is_enabled;
}

scara_step_coords_t stepper_driver_get_position(void) {
  return current_pos;
}

void stepper_driver_set_position(const scara_step_coords_t *pos) {
  if (pos) {
    current_pos = *pos;
  }
}

void stepper_driver_emergency_stop(void) {
  /* Abort any active DMA transfers */
  if (dma_chan_ping >= 0) {
    dma_channel_abort(dma_chan_ping);
  }

  if (dma_chan_pong >= 0) {
    dma_channel_abort(dma_chan_pong);
  }

  /* Clear PIO FIFO */
  pio_sm_set_enabled(stepper_pio, stepper_sm, false);
  pio_sm_clear_fifos(stepper_pio, stepper_sm);
  pio_sm_restart(stepper_pio, stepper_sm);
  pio_sm_set_enabled(stepper_pio, stepper_sm, true);

  /* Reset all step pins to LOW */
  gpio_put(SCARA_J1_STEP_PIN, 0);
  gpio_put(SCARA_J2_STEP_PIN, 0);
  gpio_put(SCARA_Z_STEP_PIN, 0);
  gpio_put(SCARA_J4_STEP_PIN, 0);

  is_busy = false;
}

bool stepper_driver_is_busy(void) {
  if (dma_chan_ping >= 0 && dma_channel_is_busy(dma_chan_ping)) {
    return true;
  }

  if (dma_chan_pong >= 0 && dma_channel_is_busy(dma_chan_pong)) {
    return true;
  }

  return is_busy;
}

bool stepper_driver_submit_chunk(const uint32_t *commands, size_t count) {
  if (!commands || count == 0 || count > PING_PONG_BUFFER_SIZE) {
    return false;
  }

  /* If ping channel is free, use ping */
  if (!dma_channel_is_busy(dma_chan_ping)) {
    memcpy(ping_buffer, commands, count * sizeof(uint32_t));
    dma_channel_set_read_addr(dma_chan_ping, ping_buffer, false);
    dma_channel_set_trans_count(dma_chan_ping, count, true);
    is_busy = true;

    return true;
  }

  /* If pong channel is free, use pong */
  if (!dma_channel_is_busy(dma_chan_pong)) {
    memcpy(pong_buffer, commands, count * sizeof(uint32_t));
    dma_channel_set_read_addr(dma_chan_pong, pong_buffer, false);
    dma_channel_set_trans_count(dma_chan_pong, count, true);
    is_busy = true;

    return true;
  }

  return false;
}

void stepper_driver_move_steps_sync(
    const scara_step_coords_t *target_steps, uint32_t step_rate_hz
) {
  if (!target_steps || step_rate_hz == 0) {
    return;
  }

  int32_t d_j1 = target_steps->j1_steps - current_pos.j1_steps;
  int32_t d_j2 = target_steps->j2_steps - current_pos.j2_steps;
  int32_t d_z = target_steps->z_steps - current_pos.z_steps;
  int32_t d_j4 = target_steps->j4_steps - current_pos.j4_steps;

  /* Set direction pins */
  gpio_put(SCARA_J1_DIR_PIN, d_j1 >= 0 ? 1 : 0);
  gpio_put(SCARA_J2_DIR_PIN, d_j2 >= 0 ? 1 : 0);
  gpio_put(SCARA_Z_DIR_PIN, d_z >= 0 ? 1 : 0);
  gpio_put(SCARA_J4_DIR_PIN, d_j4 >= 0 ? 1 : 0);

  int32_t abs_j1 = abs(d_j1);
  int32_t abs_j2 = abs(d_j2);
  int32_t abs_z = abs(d_z);
  int32_t abs_j4 = abs(d_j4);

  int32_t max_steps = abs_j1;
  if (abs_j2 > max_steps) {
    max_steps = abs_j2;
  }
  if (abs_z > max_steps) {
    max_steps = abs_z;
  }
  if (abs_j4 > max_steps) {
    max_steps = abs_j4;
  }

  if (max_steps == 0) {
    return;
  }

  uint32_t half_period_us = 1000000 / (2 * step_rate_hz);

  if (half_period_us < 2) {
    half_period_us = 2;
  }

  int32_t err_j1 = 0, err_j2 = 0, err_z = 0, err_j4 = 0;
  is_busy = true;

  for (int32_t i = 0; i < max_steps; i++) {
    bool step_j1 = false, step_j2 = false, step_z = false, step_j4 = false;

    err_j1 += abs_j1;

    if (err_j1 >= max_steps) {
      err_j1 -= max_steps;
      step_j1 = true;
      current_pos.j1_steps += (d_j1 >= 0) ? 1 : -1;
    }

    err_j2 += abs_j2;

    if (err_j2 >= max_steps) {
      err_j2 -= max_steps;
      step_j2 = true;
      current_pos.j2_steps += (d_j2 >= 0) ? 1 : -1;
    }

    err_z += abs_z;

    if (err_z >= max_steps) {
      err_z -= max_steps;
      step_z = true;
      current_pos.z_steps += (d_z >= 0) ? 1 : -1;
    }

    err_j4 += abs_j4;

    if (err_j4 >= max_steps) {
      err_j4 -= max_steps;
      step_j4 = true;
      current_pos.j4_steps += (d_j4 >= 0) ? 1 : -1;
    }

    /* Pulse active pins HIGH */
    if (step_j1) {
      gpio_put(SCARA_J1_STEP_PIN, 1);
    }

    if (step_j2) {
      gpio_put(SCARA_J2_STEP_PIN, 1);
    }

    if (step_z) {
      gpio_put(SCARA_Z_STEP_PIN, 1);
    }

    if (step_j4) {
      gpio_put(SCARA_J4_STEP_PIN, 1);
    }

    sleep_us(half_period_us);

    /* Reset pins LOW */
    if (step_j1) {
      gpio_put(SCARA_J1_STEP_PIN, 0);
    }

    if (step_j2) {
      gpio_put(SCARA_J2_STEP_PIN, 0);
    }

    if (step_z) {
      gpio_put(SCARA_Z_STEP_PIN, 0);
    }

    if (step_j4) {
      gpio_put(SCARA_J4_STEP_PIN, 0);
    }

    sleep_us(half_period_us);
  }

  is_busy = false;
}
