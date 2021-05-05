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

#include <signal.h>
#include <time.h>

#include "device.h"
#include "logger.h"

#define NB_READOUTS_PER_FILE 10000

/** TO DO LIST
 *
 * change HR_id numbering (1 to 8 instead of 0 to 7) (+ processing script)
 * change trigger_id numbering (start 1) (+ processing script)
 *
 */

int g_state = 1;

void handle_sigint(int sig) {
  log_printf("Caught signal %d\n", sig);
  log_flush();
  g_state = 0;
}

void give_file_to_server(char *filename, int sock) {
  log_printf("send copy request to eda-srv\n");
  log_flush();
  alt_u32 length;
  alt_u8 length_litend[4] = {0};
  char sock_read_buf[4] = {0};
  int valread;
  // send lenght of filename (uint32 little endian) to server
  length = strlen(filename);
  length_litend[0] = length; // length < 128 < 256
  send(sock, length_litend, 4, 0);
  // send filename to server
  send(sock, filename, length, 0);
  // wait server ack
  valread = read(sock, sock_read_buf, 3);
  if ((valread != 3) || (strcmp(sock_read_buf, "ACK") != 0)) {
    log_printf("instead of ACK, received :%s\n", sock_read_buf);
    log_flush();
  }
  return;
}

int main(int argc, char *argv[]) {

  signal(SIGINT, handle_sigint);

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

  // save run-dependant settings
  log_printf("thresh_delta=%lu, Rshaper=%lu, rfm_on[3:0]=%lu\n", thresh_delta,
             Rshaper, rfm_on);
  log_flush();
  char settings_filename[128];
  sprintf(settings_filename, "/home/root/run/settings_%03d.csv", run_cnt);
  FILE *settings_file = fopen(settings_filename, "w");
  if (!settings_file) {
    log_printf("could not open file %s\n", settings_filename);
    log_flush();
    device_free(ctx);
    return -1;
  }
  fprintf(
      settings_file,
      "thresh_delta=%lu; Rshaper=%lu; rfm_on[3:0]=%lu; ip_addr=%s; run_id=%d",
      thresh_delta, Rshaper, rfm_on, ip_addr, run_cnt);
  fclose(settings_file);
  give_file_to_server(settings_filename, ctx->sock_cp);

  log_printf("-----------------RUN NB %d-----------------\n", run_cnt);
  log_flush();

  // init run----------------------------------------------------------
  // prepare run file
  char daq_filename[128];
  sprintf(daq_filename, "/dev/shm/eda_%03d.000.raw",
          run_cnt); // use tmpfs for daq (to reduce writings on µSD flash mem)
  FILE *daq_file = fopen(daq_filename, "w");
  if (!daq_file) {
    log_printf("unable to open file %s\n", daq_filename);
    log_flush();
    device_free(ctx);
    return -1;
  }
  // init run counters
  alt_u32 cycle_id = 0;

  SYNC_reset_hr();

  // wait for reset BCID
  log_printf("waiting for reset_BCID command\n");
  log_flush();
  send(ctx->sock_ctl, "eda-ready", 9, 0);

  int dcc_cmd = 0xE;
  while (dcc_cmd != CMD_RESET_BCID) {
    while (SYNC_dcc_cmd_mem() == dcc_cmd) {
      if (g_state == 0)
        break;
    }
    dcc_cmd = SYNC_dcc_cmd_mem();
    // log_printf("sDCC command = %d\n",dcc_cmd);
    if (g_state == 0)
      break;
  }
  if (g_state == 0) {
    device_free(ctx);
    fclose(daq_file);
    return -1;
  }
  log_printf("SYNC_state()=%d\n", SYNC_state());
  log_printf("reset_BCID done\n");
  log_flush();

  CNT_reset();
  CNT_start();
  for (int rfm_index = 0; rfm_index < NB_RFM; rfm_index++) {
    if (((rfm_on >> rfm_index) & 1) == 1) {
      DAQ_fifo_init(rfm_index);
    }
  }

  cycle_id = 0;
  int file_cnt = 0;
  SYNC_fifo_arming();

  //----------- run loop -----------------
  while (g_state == 1) {
    // wait until new readout is started
    log_printf("trigger %07lu\n\tacq\n", cycle_id);
    log_flush();
    while ((SYNC_fpga_ro() == 0) && (g_state == 1)) {
      ;
    }
    if (g_state == 0)
      break;
    log_printf("\treadout\n");
    log_flush();
    // wait until readout is done
    while ((SYNC_fifo_ready() == 0) && (g_state == 1)) {
      ;
    }
    if (g_state == 0)
      break;
    // read hardroc data
    log_printf("\tbuffering\n");
    log_flush();
    for (int rfm_index = 0; rfm_index < NB_RFM; rfm_index++) {
      if (((rfm_on >> rfm_index) & 1) == 1) {
        log_printf("\t\trfm %d\n", rfm_index);
        log_flush();
        DAQ_bufferize_data_DIF(rfm_index);
        if (g_state == 0)
          break;
      }
    }
    SYNC_fifo_ack();
    // write data file
    log_printf("\tfwrite\n");
    log_flush();
    DAQ_write_buffer(daq_file);
    log_printf("\tdone\n");
    log_flush();
    cycle_id++;
    if ((cycle_id % NB_READOUTS_PER_FILE) == 0) {
      fclose(daq_file);
      give_file_to_server(daq_filename, ctx->sock_cp);
      // prepare new file
      memset(daq_filename, 0, 128);
      file_cnt++;
      sprintf(daq_filename, "/dev/shm/eda_%03d.%03d.raw", run_cnt, file_cnt);
      daq_file = fopen(daq_filename, "w");
    }
  }
  CNT_stop();

  // close current daq file
  fclose(daq_file);
  give_file_to_server(daq_filename, ctx->sock_cp);

  device_free(ctx);
  return (0);
}
