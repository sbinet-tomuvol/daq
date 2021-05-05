// Copyright 2021 The tomuvol-daq Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

//  MIM EDA board data acquisition program
//
// for 1 to 4 RFMs connected

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <time.h>

#include "device.h"
#include "logger.h"

/** TO DO LIST
 *
 * change HR_id numbering (1 to 8 instead of 0 to 7) (+ processing script)
 * change trigger_id numbering (start 1) (+ processing script)
 *
 */

int main(int argc, char *argv[]) {

  if (argc != 6) {
    fprintf(stderr,
            "usage : %s <thresh> <Rshaper> <rfm_on[3:0]> <ip> <run id>\n",
            argv[0]);
    exit(1);
  }

  // run-dependant settings
  alt_u32 thresh_delta = atoi(argv[1]);
  alt_u32 Rshaper = atoi(argv[2]);
  alt_u32 rfm_on = atoi(argv[3]);
  char *ip_addr = argv[4];
  int run_cnt = atoi(argv[5]);

  int err = 0;
  Device_t *ctx = new_device();

  err = device_configure(ctx, thresh_delta, Rshaper, rfm_on, ip_addr, run_cnt);
  if (err != 0) {
    device_free(ctx);
    return err;
  }

  err = device_initialize(ctx);
  if (err != 0) {
    device_free(ctx);
    return err;
  }

  err = device_start(ctx);
  if (err != 0) {
    device_free(ctx);
    return err;
  }

  err = device_wait(ctx);
  if (err != 0) {
    device_free(ctx);
    return err;
  }

  err = device_stop(ctx);
  if (err != 0) {
    device_free(ctx);
    return err;
  }

  device_free(ctx);
  return (0);
}
