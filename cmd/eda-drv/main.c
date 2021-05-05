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

#include "config.h"
#include "device.h"
#include "logger.h"

#define PORT 8877
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
  // fixed-location, temporary, line-buffered log file
  log_init();

  // run-dependant settings
  alt_u32 thresh_delta = atoi(argv[1]);
  alt_u32 Rshaper = atoi(argv[2]);
  alt_u32 rfm_on = atoi(argv[3]);
  char *ip_addr = argv[4];
  int run_cnt = atoi(argv[5]);

  // baseline settings-------------------------------------------------
  alt_u32 dac_floor_table[NB_RFM * NB_HR * 3];
  alt_u32 pa_gain_table[NB_RFM * NB_HR * 64];
  alt_u32 mask_table[NB_RFM * NB_HR * 64];

  // copy base settings files from clrtodaq0 (using ssh keys)
  char command[128];
  sprintf(
      command,
      "scp -P 1122 -r mim@193.48.81.203:/mim/soft/eda/config_base /dev/shm/");
  system(command);

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
  if (read_th_offset(dac_floor_file, dac_floor_table) < 0) {
    fclose(dac_floor_file);
    return -1;
  }
  fclose(dac_floor_file);
  // preamplifier gains
  FILE *pa_gain_file = fopen("/dev/shm/config_base/pa_gain_4rfm.csv", "r");
  if (!pa_gain_file)
    return -1;
  if (read_pa_gain(pa_gain_file, pa_gain_table) < 0) {
    fclose(pa_gain_file);
    return -1;
  }
  fclose(pa_gain_file);
  // masks
  FILE *mask_file = fopen("/dev/shm/config_base/mask_4rfm.csv", "r");
  if (!mask_file)
    return -1;
  if (read_mask(mask_file, mask_table) < 0) {
    fclose(mask_file);
    return -1;
  }
  fclose(mask_file);

  // create socket (for DAQ file copy)---------------------------------

  // for copy server (on clrtodaq x)
  int sock_cp = 0;
  struct sockaddr_in serv_addr_cp;

  if ((sock_cp = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    log_printf("\n Socket creation error \n");
    log_flush();
    return -1;
  }
  serv_addr_cp.sin_family = AF_INET;
  serv_addr_cp.sin_port = htons(PORT);

  // Convert IPv4 and IPv6 addresses from text to binary form
  if (inet_pton(AF_INET, ip_addr, &serv_addr_cp.sin_addr) <= 0) {
    log_printf("\nInvalid address/ Address %s not supported \n", ip_addr);
    log_flush();
    return -1;
  }

  if (connect(sock_cp, (struct sockaddr *)&serv_addr_cp, sizeof(serv_addr_cp)) <
      0) {
    log_printf("\nSocket Connection Failed \n");
    log_flush();
    return -1;
  }

  // for eda-ctl (on localhost)
  int sock_ctl = 0;
  struct sockaddr_in serv_addr_ctl;

  if ((sock_ctl = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
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

  if (connect(sock_ctl, (struct sockaddr *)&serv_addr_ctl,
              sizeof(serv_addr_ctl)) < 0) {
    log_printf("\nSocket Connection Failed \n");
    log_flush();
    return -1;
  }

  // FPGA-HPS memory mapping-------------------------------------------
  int mem_fd;
  if ((mem_fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
    log_printf("ERROR: could not open \"/dev/mem\"...\n");
    log_flush();
    close(sock_cp);
    close(sock_ctl);
    return -1;
  }
  // lightweight HPS to FPGA bus
  if (!mmap_lw_h2f(mem_fd)) {
    close(sock_cp);
    close(sock_ctl);
    close(mem_fd);
    return -1;
  }
  // HPS to FPGA bus
  if (!mmap_h2f(mem_fd)) {
    close(sock_cp);
    close(sock_ctl);
    close(mem_fd);
    return -1;
  }

  // Init FPGA---------------------------------------------------------

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
    munmap_lw_h2f(mem_fd);
    munmap_h2f(mem_fd);
    close(mem_fd);
    close(sock_cp);
    close(sock_ctl);
    return -1;
  }
  log_printf("the PLL is locked\n");
  log_printf("pll lock=%d\n", SYNC_pll_lck());
  log_flush();

  // activate RFMs
  int rfm_index;
  for (rfm_index = 0; rfm_index < NB_RFM; rfm_index++) {
    if (((rfm_on >> rfm_index) & 1) == 1) {
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

  // HR configuration--------------------------------------------------

  HRSC_set_bit(0, 854,
               0); // disable trig_out output pin (RFM v1 coupling problem)
  HRSC_set_shaper_resis(0, Rshaper);
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
  sprintf(sc_filename, "/home/root/run/hr_sc_%03d.csv", run_cnt);
  FILE *sc_file = fopen(sc_filename, "w");
  if (!sc_file) {
    log_printf("could not open file %s\n", sc_filename);
    log_flush();
    munmap_lw_h2f(mem_fd);
    munmap_h2f(mem_fd);
    close(mem_fd);
    close(sock_cp);
    close(sock_ctl);
    return -1;
  }

  // for each active RFM, tune the configuration and send it
  for (rfm_index = 0; rfm_index < NB_RFM; rfm_index++) {
    if (((rfm_on >> rfm_index) & 1) == 1) {
      // set mask
      alt_u32 mask;
      for (hr_addr = 0; hr_addr < 8; hr_addr++) {
        for (chan = 0; chan < 64; chan++) {
          mask = mask_table[64 * (NB_HR * rfm_index + hr_addr) + chan];
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
        th0 = dac_floor_table[3 * (NB_HR * rfm_index + hr_addr)] + thresh_delta;
        th1 = dac_floor_table[3 * (NB_HR * rfm_index + hr_addr) + 1] +
              thresh_delta;
        th2 = dac_floor_table[3 * (NB_HR * rfm_index + hr_addr) + 2] +
              thresh_delta;
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
          pa_gain = pa_gain_table[64 * (NB_HR * rfm_index + hr_addr) + chan];
          log_printf("%u      %u      %u\n", (uint32_t)hr_addr, (uint32_t)chan,
                     (uint32_t)pa_gain);
          log_flush();
          HRSC_set_preamp(hr_addr, chan, pa_gain);
        }
      }
      // send to HRs
      if (HRSC_set_config(rfm_index) < 0) {
        PRINT_config(stderr, rfm_index);
        munmap_lw_h2f(mem_fd);
        munmap_h2f(mem_fd);
        close(mem_fd);
        close(sock_cp);
        close(sock_ctl);
        return -1;
      }
      log_printf("Hardroc configuration done\n");
      log_flush();
      if (HRSC_reset_read_registers(rfm_index) < 0) {
        PRINT_config(stderr, rfm_index);
        munmap_lw_h2f(mem_fd);
        munmap_h2f(mem_fd);
        close(mem_fd);
        close(sock_cp);
        close(sock_ctl);
        return -1;
      }
      fprintf(sc_file, "#RFM_INDEX= %d ------------------------\n", rfm_index);
      HRSC_write_conf_mult(sc_file);
    }
  }
  fclose(sc_file);
  memset(command, 0, 128);
  sprintf(command,
          "scp -P 1122 %s mim@193.48.81.203:/mim/soft/eda/config_history/",
          sc_filename);
  system(command);

  log_printf("read register reset done\n");
  log_flush();
  sleep(1); // let DACs stabilize

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
    munmap_lw_h2f(mem_fd);
    munmap_h2f(mem_fd);
    close(mem_fd);
    close(sock_cp);
    close(sock_ctl);
    return -1;
  }
  fprintf(
      settings_file,
      "thresh_delta=%lu; Rshaper=%lu; rfm_on[3:0]=%lu; ip_addr=%s; run_id=%d",
      thresh_delta, Rshaper, rfm_on, ip_addr, run_cnt);
  fclose(settings_file);
  give_file_to_server(settings_filename, sock_cp);

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
    munmap_lw_h2f(mem_fd);
    munmap_h2f(mem_fd);
    close(mem_fd);
    close(sock_cp);
    close(sock_ctl);
    return -1;
  }
  // init run counters
  alt_u32 cycle_id = 0;

  SYNC_reset_hr();

  // wait for reset BCID
  log_printf("waiting for reset_BCID command\n");
  log_flush();
  send(sock_ctl, "eda-ready", 9, 0);

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
    munmap_lw_h2f(mem_fd);
    munmap_h2f(mem_fd);
    close(mem_fd);
    close(sock_cp);
    close(sock_ctl);
    fclose(daq_file);
    return -1;
  }
  log_printf("SYNC_state()=%d\n", SYNC_state());
  log_printf("reset_BCID done\n");
  log_flush();

  CNT_reset();
  CNT_start();
  for (rfm_index = 0; rfm_index < NB_RFM; rfm_index++) {
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
    for (rfm_index = 0; rfm_index < NB_RFM; rfm_index++) {
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
      give_file_to_server(daq_filename, sock_cp);
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
  give_file_to_server(daq_filename, sock_cp);
  close(sock_cp);
  close(sock_ctl);
  log_close();

  munmap_lw_h2f(mem_fd);
  munmap_h2f(mem_fd);
  close(mem_fd);

  return (0);
}
