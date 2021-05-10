// Copyright 2021 The tomuvol-daq Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

//go:build arm
// +build arm

package ceda

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
	"fmt"
	"log"
	"time"
)

const (
	nRFM        = 4
	difIDOffset = 0x00

	nHR         = 8
	nBitsCfgHR  = 872
	nBytesCfgHR = 109
	szCfgHR     = 4 + nHR*nBytesCfgHR
	nChans      = 64

	daqBufferSize = nRFM * (26 + nHR*(2+128*20))
)

type Device struct {
	ctx *C.Device_t

	msg  *log.Logger
	id   uint32        // [0,8)
	rfms []int         // list of enabled RFMs
	difs map[int]uint8 // map of EDA-slot->DIF/RFM-id

	dir string
	run uint32

	cfg struct {
		mode string // csv or db
		ctl  struct {
			addr string // addr+port to eda-ctl
		}

		hr struct {
			fname   string
			rshaper uint32 // resistance shaper
			cshaper uint32 // capacity shaper
		}

		daq struct {
			fname string
			floor [nRFM * nHR * 3]uint32
			delta uint32 // delta threshold
			rfm   uint32 // RFM ON mask

			addrs []string // [addr:port]s for sending DIF data

			timeout time.Duration // timeout for reset-BCID
		}
	}
}

func newDevice(devmem, odir, devshm, cfgdir string) (*Device, error) {
	ctx := C.new_device()
	if ctx == nil {
		return nil, fmt.Errorf("ceda: could not create EDA device")
	}

	return &Device{ctx}, nil
}

func (dev *Device) Close() error {
	if dev.ctx == nil {
		return nil
	}
	C.device_free(dev.ctx)
	dev.ctx = nil
	return nil
}

func (dev *Device) Printf(format string, args ...interface{}) {
	panic("not implemented")
}

func (dev *Device) Initialize() error {
	rc := C.device_initialize(dev.ctx)
	if rc != 0 {
		return fmt.Errorf("ceda: could not initialize EDA device rc=%d", rc)
	}
	return nil
}

func (dev *Device) Start() error {
	rc := C.device_start(dev.ctx)
	if rc != 0 {
		return fmt.Errorf("ceda: could not start EDA device: rc=%d", rc)
	}
	return nil
}

func (dev *Device) Stop() error {
	rc := C.device_stop(dev.ctx)
	if rc != 0 {
		return fmt.Errorf("ceda: could not stop EDA device: rc=%d", rc)
	}
	return nil
}
