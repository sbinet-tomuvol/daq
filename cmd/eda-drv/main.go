// Copyright 2021 The tomuvol-daq Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

/** TO DO LIST
 *
 * change HR_id numbering (1 to 8 instead of 0 to 7) (+ processing script)
 * change trigger_id numbering (start 1) (+ processing script)
 *
 */

//#cgo CFLAGS: -g -Wall -std=c99 -D_GNU_SOURCE=1 -I/build/soc_eds/ip/altera/hps/altera_hps/hwlib/include -I.
//#cgo LDFLAGS: -pthread -static
//
//#include <stdlib.h>
//#include <string.h>
//#include "device.h"
import "C"

import (
	"flag"
	"log"
	"unsafe"
)

func main() {
	log.SetPrefix("eda-drv: ")
	log.SetFlags(0)

	var (
		thresh  = flag.Int("thresh", 0, "trigger delta threshold")
		rshaper = flag.Int("rshaper", 0, "RFM r-shaper")
		rfm     = flag.Int("rfm", 0, "RFM-slot on/off mask")
		ip      = flag.String("ip", "", "ip of eda-svc")
		run     = flag.Int("run", 0, "run number to use for data taking")
	)

	flag.Parse()

	ctx := C.new_device()
	if ctx == nil {
		log.Fatal("could not create EDA device")
	}
	defer C.device_free(ctx)

	c_ip := C.CString(*ip)
	defer C.free(unsafe.Pointer(c_ip))

	err := C.device_configure(ctx, C.uint32_t(*thresh), C.uint32_t(*rshaper), C.uint32_t(*rfm), c_ip, C.int(*run))
	if err != 0 {
		log.Fatalf("could not configure EDA: err=%d", err)
	}

	err = C.device_initialize(ctx)
	if err != 0 {
		log.Fatalf("could not configure EDA: err=%d", err)
	}

	err = C.device_start(ctx)
	if err != 0 {
		log.Fatalf("could not configure EDA: err=%d", err)
	}

	err = C.device_wait(ctx)
	if err != 0 {
		log.Fatalf("could not configure EDA: err=%d", err)
	}

	err = C.device_stop(ctx)
	if err != 0 {
		log.Fatalf("could not stop EDA: err=%d", err)
	}

}
