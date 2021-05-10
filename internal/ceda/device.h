// +build arm

#ifndef EDA_DEVICE_H
#define EDA_DEVICE_H 1

/*
 * Copyright 2020 Baptiste Joly <baptiste.joly@clermont.in2p3.fr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 */

/**
 * EDA board (Arrow SocKit) software library
 * I/O functions for communication with FPGA processes
 * via memory mapping
 */

#include "fpga.h"
#include <stdint.h>

typedef struct Device Device_t;

Device_t *new_device();

int device_configure(Device_t *ctx, uint32_t thresh, uint32_t rshaper,
                     uint32_t rfm, const char *ip, int run);
int device_initialize(Device_t *ctx);
int device_start(Device_t *ctx);
int device_wait(Device_t *ctx);
int device_stop(Device_t *ctx);

void device_free(Device_t *ctx);

#endif // !EDA_DEVICE_H
