// Copyright 2021 The tomuvol-daq Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"flag"
	"log"

	"github.com/sbinet-tomuvol/daq/internal/ceda"
)

func main() {
	log.SetPrefix("eda-drv: ")
	log.SetFlags(0)

	var (
		addr = flag.String("addr", ":9999", "eda-ctl [addr]:port")
		odir = flag.String("o", "/home/root/run", "output dir")

		devmem = flag.String("dev-mem", "/dev/mem", "")
		devshm = flag.String("dev-shm", "/dev/shm", "")
		cfgdir = flag.String("cfg-dir", "/dev/shm/config_base", "")
	)

	flag.Parse()

	err := ceda.Serve(*addr, *odir, *devmem, *devshm, *cfgdir)
	if err != nil {
		log.Fatalf("could not run eda-drv: %+v", err)
	}
}
