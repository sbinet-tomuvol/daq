// +build arm

/*
 * Copyright 2020
 * Baptiste Joly <baptiste.joly@clermont.in2p3.fr>
 * Guillaume Blanchard <guillaume.blanchard@clermont.in2p3.fr>
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

#include "device.h"
#include "config.h"
#include "logger.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

struct Device {
  // run-dependant settings
  alt_u32 thresh_delta;
  alt_u32 rshaper;
  alt_u32 rfm_on;

  char *ip_addr;
  int run_cnt;

  // baseline settings
  alt_u32 dac_floor_table[NB_RFM * NB_HR * 3];
  alt_u32 pa_gain_table[NB_RFM * NB_HR * 64];
  alt_u32 mask_table[NB_RFM * NB_HR * 64];

  int sock_cp;
  int sock_ctl;

  int mem_fd;

  char daq_filename[128];
  FILE *daq_file;
  alt_u32 cycle_id; // run counters

  struct {
    pthread_t id;
    int rc;
    FILE *daq;
  } task[NB_RFM]; // run-loop status
};

#define PORT 8877
#define NB_READOUTS_PER_FILE 10000

int g_state = 1;

#include <signal.h>
void handle_sigint(int sig) {
  log_printf("Caught signal %d\n", sig);
  log_flush();
  g_state = 0;
}

int device_init_mmap(Device_t *ctx);
int device_init_fpga(Device_t *ctx);
int device_init_hrsc(Device_t *ctx);

int device_init_run(Device_t *ctx);
void *device_loop(void *ctx);

void give_file_to_server(char *filename, int sock);

Device_t *new_device() {
  Device_t *ctx = (Device_t *)calloc(1, sizeof(Device_t));
  signal(SIGINT, handle_sigint);

  return ctx;
}

int device_configure(Device_t *ctx, uint32_t thresh, uint32_t rshaper,
                     uint32_t rfm, const char *ip, int run) {
  ctx->thresh_delta = thresh;
  ctx->rshaper = rshaper;
  ctx->rfm_on = rfm;
  ctx->ip_addr = strdup(ip);
  ctx->run_cnt = run;

  // fixed-location, temporary, line-buffered log file
  log_init();

  // copy base settings files from clrtodaq0 (using ssh keys)
  char command[128];
  sprintf(
      command,
      "scp -P 1122 -r mim@193.48.81.203:/mim/soft/eda/config_base /dev/shm/");

  int err = 0;
  err = system(command);
  if (err != 0) {
    log_printf("could not copy base settings from clrtodaq\n");
    return err;
  }

  // load files to tables
  // single-HR configuration file
  FILE *conf_base_file = fopen("/dev/shm/config_base/conf_base.csv", "r");
  if (!conf_base_file)
    return -1;
  if (HRSC_read_conf_singl(conf_base_file, 0) < 0) {
    fclose(conf_base_file);
    return -1;
  }
  fclose(conf_base_file);

  // floor thresholds
  FILE *dac_floor_file = fopen("/dev/shm/config_base/dac_floor_4rfm.csv", "r");
  if (!dac_floor_file)
    return -1;
  if (read_th_offset(dac_floor_file, ctx->dac_floor_table) < 0) {
    fclose(dac_floor_file);
    return -1;
  }
  fclose(dac_floor_file);

  // preamplifier gains
  FILE *pa_gain_file = fopen("/dev/shm/config_base/pa_gain_4rfm.csv", "r");
  if (!pa_gain_file)
    return -1;
  if (read_pa_gain(pa_gain_file, ctx->pa_gain_table) < 0) {
    fclose(pa_gain_file);
    return -1;
  }
  fclose(pa_gain_file);

  // masks
  FILE *mask_file = fopen("/dev/shm/config_base/mask_4rfm.csv", "r");
  if (!mask_file)
    return -1;
  if (read_mask(mask_file, ctx->mask_table) < 0) {
    fclose(mask_file);
    return -1;
  }
  fclose(mask_file);

  // create socket (for DAQ file copy)---------------------------------

  // for copy server (on clrtodaq x)
  ctx->sock_cp = 0;
  struct sockaddr_in serv_addr_cp;

  if ((ctx->sock_cp = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    log_printf("\n Socket creation error \n");
    log_flush();
    return -1;
  }
  serv_addr_cp.sin_family = AF_INET;
  serv_addr_cp.sin_port = htons(PORT);

  // Convert IPv4 and IPv6 addresses from text to binary form
  if (inet_pton(AF_INET, ctx->ip_addr, &serv_addr_cp.sin_addr) <= 0) {
    log_printf("\nInvalid address/ Address %s not supported \n", ctx->ip_addr);
    log_flush();
    return -1;
  }

  if (connect(ctx->sock_cp, (struct sockaddr *)&serv_addr_cp,
              sizeof(serv_addr_cp)) < 0) {
    log_printf("\nSocket Connection Failed \n");
    log_flush();
    return -1;
  }

  // for eda-ctl (on localhost)
  ctx->sock_ctl = 0;
  struct sockaddr_in serv_addr_ctl;

  if ((ctx->sock_ctl = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    log_printf("\n Socket creation error \n");
    log_flush();
    return -1;
  }
  serv_addr_ctl.sin_family = AF_INET;
  serv_addr_ctl.sin_port = htons(PORT);

  // Convert IPv4 and IPv6 addresses from text to binary form
  if (inet_pton(AF_INET, "127.0.0.1", &serv_addr_ctl.sin_addr) <= 0) {
    log_printf("\nInvalid address/ Address %s not supported \n", "127.0.0.1");
    log_flush();
    return -1;
  }

  if (connect(ctx->sock_ctl, (struct sockaddr *)&serv_addr_ctl,
              sizeof(serv_addr_ctl)) < 0) {
    log_printf("\nSocket Connection Failed \n");
    log_flush();
    return -1;
  }

  return 0;
}

int device_initialize(Device_t *ctx) {
  // FPGA-HPS memory mapping-------------------------------------------
  if (!device_init_mmap(ctx)) {
    return -1;
  }

  // Init FPGA---------------------------------------------------------
  if (!device_init_fpga(ctx)) {
    return -1;
  }

  // HR configuration--------------------------------------------------
  if (!device_init_hrsc(ctx)) {
    return -1;
  }

  return 0;
}

void device_free(Device_t *ctx) {
  free(ctx->ip_addr);

  if (ctx->sock_cp != 0) {
    close(ctx->sock_cp);
  }
  if (ctx->sock_ctl != 0) {
    close(ctx->sock_ctl);
  }
  if (ctx->mem_fd != 0) {
    munmap_lw_h2f(ctx->mem_fd);
    munmap_h2f(ctx->mem_fd);
    close(ctx->mem_fd);
  }

  if (ctx->daq_file != 0) {
    fclose(ctx->daq_file);
  }

  free(ctx);
  ctx = NULL;
}

int device_init_mmap(Device_t *ctx) {
  ctx->mem_fd = 0;
  if ((ctx->mem_fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
    log_printf("ERROR: could not open \"/dev/mem\"...\n");
    log_flush();
    ctx->mem_fd = 0;
    return -1;
  }
  // lightweight HPS to FPGA bus
  if (!mmap_lw_h2f(ctx->mem_fd)) {
    close(ctx->mem_fd);
    ctx->mem_fd = 0;
    return -1;
  }
  // HPS to FPGA bus
  if (!mmap_h2f(ctx->mem_fd)) {
    munmap_lw_h2f(ctx->mem_fd);
    close(ctx->mem_fd);
    ctx->mem_fd = 0;
    return -1;
  }

  return 0;
}

int device_init_fpga(Device_t *ctx) {
  // reset fpga and set clock
  SYNC_reset_fpga();
  usleep(2);
  // make sure the pll is locked
  int cnt_poll = 0;
  while ((!SYNC_pll_lck()) && (cnt_poll < 100)) {
    usleep(10000);
    cnt_poll++;
  }
  if (cnt_poll >= 100) {
    log_printf("the PLL is not locked\n");
    log_flush();
    return -1;
  }
  log_printf("the PLL is locked\n");
  log_printf("pll lock=%d\n", SYNC_pll_lck());
  log_flush();

  // activate RFMs
  int rfm_index;
  for (rfm_index = 0; rfm_index < NB_RFM; rfm_index++) {
    if (((ctx->rfm_on >> rfm_index) & 1) == 1) {
      RFM_on(rfm_index);
      RFM_enable(rfm_index);
    }
  }
  sleep(1);
  log_printf("control pio=%lx\n", PIO_ctrl_get());
  log_flush();

  SYNC_select_command_dcc();
  SYNC_enable_dcc_busy();
  SYNC_enable_dcc_ramfull();

  return 0;
}

int device_init_hrsc(Device_t *ctx) {
  int err = 0;
  HRSC_set_bit(0, 854,
               0); // disable trig_out output pin (RFM v1 coupling problem)
  HRSC_set_shaper_resis(0, ctx->rshaper);
  HRSC_set_shaper_capa(0, 3);

  HRSC_copy_conf(0, 1);
  HRSC_copy_conf(0, 2);
  HRSC_copy_conf(0, 3);
  HRSC_copy_conf(0, 4);
  HRSC_copy_conf(0, 5);
  HRSC_copy_conf(0, 6);
  HRSC_copy_conf(0, 7);

  // set chip ids
  alt_u32 hr_addr, chan;
  for (hr_addr = 0; hr_addr < 8; hr_addr++)
    HRSC_set_chip_id(hr_addr, hr_addr);

  // prepare config file (for history)
  char sc_filename[128];
  sprintf(sc_filename, "/home/root/run/hr_sc_%03d.csv", ctx->run_cnt);
  FILE *sc_file = fopen(sc_filename, "w");
  if (!sc_file) {
    log_printf("could not open file %s\n", sc_filename);
    log_flush();
    return -1;
  }

  // for each active RFM, tune the configuration and send it
  for (int rfm_index = 0; rfm_index < NB_RFM; rfm_index++) {
    if (((ctx->rfm_on >> rfm_index) & 1) == 1) {
      // set mask
      alt_u32 mask;
      for (hr_addr = 0; hr_addr < 8; hr_addr++) {
        for (chan = 0; chan < 64; chan++) {
          mask = ctx->mask_table[64 * (NB_HR * rfm_index + hr_addr) + chan];
          log_printf("%u      %u      %u\n", (uint32_t)hr_addr, (uint32_t)chan,
                     (uint32_t)mask);
          log_flush();
          HRSC_set_mask(hr_addr, chan, mask);
        }
      }
      // set DAC thresholds
      log_printf("HR      thresh0     thresh1     thresh2\n");
      log_flush();
      alt_u32 th0, th1, th2;
      for (hr_addr = 0; hr_addr < 8; hr_addr++) {
        th0 = ctx->dac_floor_table[3 * (NB_HR * rfm_index + hr_addr)] +
              ctx->thresh_delta;
        th1 = ctx->dac_floor_table[3 * (NB_HR * rfm_index + hr_addr) + 1] +
              ctx->thresh_delta;
        th2 = ctx->dac_floor_table[3 * (NB_HR * rfm_index + hr_addr) + 2] +
              ctx->thresh_delta;
        log_printf("%u      %u      %u      %u\n", (uint32_t)hr_addr,
                   (uint32_t)th0, (uint32_t)th1, (uint32_t)th2);
        log_flush();
        HRSC_set_DAC0(hr_addr, th0);
        HRSC_set_DAC1(hr_addr, th1);
        HRSC_set_DAC2(hr_addr, th2);
      }
      // set preamplifier gain
      log_printf("HR      chan        pa_gain\n");
      log_flush();
      alt_u32 pa_gain;
      for (hr_addr = 0; hr_addr < 8; hr_addr++) {
        for (chan = 0; chan < 64; chan++) {
          pa_gain =
              ctx->pa_gain_table[64 * (NB_HR * rfm_index + hr_addr) + chan];
          log_printf("%u      %u      %u\n", (uint32_t)hr_addr, (uint32_t)chan,
                     (uint32_t)pa_gain);
          log_flush();
          HRSC_set_preamp(hr_addr, chan, pa_gain);
        }
      }
      // send to HRs
      if (HRSC_set_config(rfm_index) < 0) {
        PRINT_config(stderr, rfm_index);
        return -1;
      }
      log_printf("Hardroc configuration done\n");
      log_flush();
      if (HRSC_reset_read_registers(rfm_index) < 0) {
        PRINT_config(stderr, rfm_index);
        return -1;
      }
      fprintf(sc_file, "#RFM_INDEX= %d ------------------------\n", rfm_index);
      HRSC_write_conf_mult(sc_file);
    }
  }
  fclose(sc_file);
  char command[128];
  sprintf(command,
          "scp -P 1122 %s mim@193.48.81.203:/mim/soft/eda/config_history/",
          sc_filename);
  err = system(command);
  if (err != 0) {
    log_printf("could not send config to history store: err=%d\n", err);
    log_flush();
    return err;
  }

  log_printf("read register reset done\n");
  log_flush();
  sleep(1); // let DACs stabilize

  return 0;
}

int device_start(Device_t *ctx) {
  int err = 0;

  // init run----------------------------------------------------------
  err = device_init_run(ctx);
  if (err != 0) {
    return err;
  }

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
    return -1;
  }
  log_printf("SYNC_state()=%d\n", SYNC_state());
  log_printf("reset_BCID done\n");
  log_flush();

  CNT_reset();
  CNT_start();
  for (int rfm_index = 0; rfm_index < NB_RFM; rfm_index++) {
    if (((ctx->rfm_on >> rfm_index) & 1) == 1) {
      DAQ_fifo_init(rfm_index);
    }
  }

  ctx->cycle_id = 0;
  SYNC_fifo_arming();

  //  for (int i = 0; i < NB_RFM; i++) {
  //    if (((ctx->rfm_on >> i) & 1) == 0) {
  //      continue;
  //    }
  //    err = pthread_create(&ctx->task[i].id, NULL, device_loop, (void *)ctx);
  //    if (err != 0) {
  //      log_printf("could not create worker for RFM=%d: err=%d\n", i, err);
  //      log_flush();
  //      return err;
  //    }
  //  }

  err = pthread_create(&ctx->task[0].id, NULL, device_loop, (void *)ctx);
  if (err != 0) {
    log_printf("could not create worker: err=%d\n", err);
    log_flush();
    return err;
  }

  return 0;
}

int device_wait(Device_t *ctx) {
  int err = 0;
  //  for (int i = 0; i < NB_RFM; i++) {
  //    if (((ctx->rfm_on >> i) & 1) == 0) {
  //      continue;
  //    }
  //    pthread_join(ctx->task[i].id, (void **)(&ctx->task[i].rc));
  //    if (ctx->task[i].rc != 0) {
  //      err = ctx->task[i].rc;
  //      log_printf("error joining worker for RFM=%d: err=%d\n", i, err);
  //      log_flush();
  //    }
  //  }

  err = pthread_join(ctx->task[0].id, (void **)(&ctx->task[0].rc));
  if (err != 0) {
    log_printf("could not join worker: err=%d\n", err);
    log_flush();
    return err;
  }

  if (ctx->task[0].rc != 0) {
    err = ctx->task[0].rc;
    log_printf("error during worker loop: err=%d\n", err);
    log_flush();
    return err;
  }

  return 0;
}

int device_stop(Device_t *ctx) {
  // close current daq file
  fclose(ctx->daq_file);
  ctx->daq_file = NULL;

  give_file_to_server(ctx->daq_filename, ctx->sock_cp);
  return 0;
}

int device_init_run(Device_t *ctx) {
  // save run-dependant settings
  log_printf("thresh_delta=%lu, Rshaper=%lu, rfm_on[3:0]=%lu\n",
             ctx->thresh_delta, ctx->rshaper, ctx->rfm_on);
  log_flush();
  char settings_filename[128];
  sprintf(settings_filename, "/home/root/run/settings_%03d.csv", ctx->run_cnt);
  FILE *settings_file = fopen(settings_filename, "w");
  if (!settings_file) {
    log_printf("could not open file %s\n", settings_filename);
    log_flush();
    return -1;
  }
  fprintf(
      settings_file,
      "thresh_delta=%lu; Rshaper=%lu; rfm_on[3:0]=%lu; ip_addr=%s; run_id=%d",
      ctx->thresh_delta, ctx->rshaper, ctx->rfm_on, ctx->ip_addr, ctx->run_cnt);
  fclose(settings_file);
  give_file_to_server(settings_filename, ctx->sock_cp);

  log_printf("-----------------RUN NB %d-----------------\n", ctx->run_cnt);
  log_flush();

  // FIXME(sbinet): replace with a pair of open_memstream(3).
  // prepare run file
  sprintf(
      ctx->daq_filename, "/dev/shm/eda_%03d.000.raw",
      ctx->run_cnt); // use tmpfs for daq (to reduce writings on µSD flash mem)
  ctx->daq_file = fopen(ctx->daq_filename, "w");
  if (!ctx->daq_file) {
    log_printf("unable to open file %s\n", ctx->daq_filename);
    log_flush();
    return -1;
  }
  // init run counters
  ctx->cycle_id = 0;

  SYNC_reset_hr();

  return 0;
}

void *device_loop(void *argp) {

  Device_t *ctx = (Device_t *)argp;
  int id = pthread_self();
  ctx->task[id].rc = 0;

  int file_cnt = 0;
  //----------- run loop -----------------
  while (g_state == 1) {
    // wait until new readout is started
    log_printf("trigger %07lu\n\tacq\n", ctx->cycle_id);
    log_flush();
    while ((SYNC_fpga_ro() == 0) && (g_state == 1)) {
      ;
    }
    if (g_state == 0) {
      break;
    }

    log_printf("\treadout\n");
    log_flush();
    // wait until readout is done
    while ((SYNC_fifo_ready() == 0) && (g_state == 1)) {
      ;
    }
    if (g_state == 0) {
      break;
    }

    // read hardroc data
    log_printf("\tbuffering\n");
    log_flush();
    for (int rfm_index = 0; rfm_index < NB_RFM; rfm_index++) {
      if (((ctx->rfm_on >> rfm_index) & 1) == 0) {
        continue;
      }
      log_printf("\t\trfm %d\n", rfm_index);
      log_flush();
      DAQ_bufferize_data_DIF(rfm_index);
      if (g_state == 0) {
        break;
      }
    }
    SYNC_fifo_ack();

    // write data file
    log_printf("\tfwrite\n");
    log_flush();
    DAQ_write_buffer(ctx->daq_file);
    log_printf("\tdone\n");
    log_flush();

    ctx->cycle_id++;
    if ((ctx->cycle_id % NB_READOUTS_PER_FILE) == 0) {
      fclose(ctx->daq_file);
      give_file_to_server(ctx->daq_filename, ctx->sock_cp);
      // prepare new file
      memset(ctx->daq_filename, 0, 128);
      file_cnt++;
      sprintf(ctx->daq_filename, "/dev/shm/eda_%03d.%03d.raw", ctx->run_cnt,
              file_cnt);
      ctx->daq_file = fopen(ctx->daq_filename, "w");
    }
  }
  CNT_stop();

  pthread_exit((void *)(&ctx->task[id].rc));
  return NULL;
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
