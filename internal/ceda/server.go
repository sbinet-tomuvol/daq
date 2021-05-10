// Copyright 2021 The tomuvol-daq Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

//go:build arm
// +build arm

package ceda

import (
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"log"
	"net"
	"os"
	"strconv"
	"strings"

	"github.com/go-lpc/mim/conddb"
)

type server struct {
	ctl net.Listener

	msg    *log.Logger
	odir   string
	devmem string
	devshm string
	cfgdir string

	dev *Device
}

func Serve(addr, odir, devmem, devshm, cfgdir string) error {
	srv, err := newServer(addr, odir, devmem, devshm, cfgdir)
	if err != nil {
		return fmt.Errorf("could not create eda driver: %w", err)
	}
	return srv.serve()
}

func newServer(addr, odir, devmem, devshm, cfgdir string) (*server, error) {
	ctl, err := net.Listen("tcp", addr)
	if err != nil {
		return nil, fmt.Errorf("could not create eda-drv server on %q: %w", addr, err)
	}

	srv := &server{
		ctl: ctl,

		msg: log.New(os.Stdout, "eda-drv: ", 0),

		odir:   odir,
		devmem: devmem,
		devshm: devshm,
		cfgdir: cfgdir,
	}
	return srv, nil
}

func (srv *server) serve() error {
	defer srv.close()

	for {
		conn, err := srv.ctl.Accept()
		if err != nil {
			return fmt.Errorf("could not accept connection: %w", err)
		}

		err = srv.handle(conn)
		if err != nil {
			srv.msg.Printf("could not run EDA board: %+v", err)
			continue
		}
	}
}

func (srv *server) handle(conn net.Conn) error {
	defer conn.Close()
	srv.msg.Printf("serving %v...", conn.RemoteAddr())
	defer srv.msg.Printf("serving %v... [done]", conn.RemoteAddr())

	srv.dev = nil
	dev, err := newDevice(
		srv.devmem, srv.odir, srv.devshm, srv.cfgdir,
	)
	if err != nil {
		return fmt.Errorf("could not create EDA device: %w", err)
	}
	defer dev.Close()
	srv.dev = dev

	// FIXME(sbinet): use DIM hooks to configure those
	dev.id = 1
	dev.cfg.hr.rshaper = 3
	dim, _, err := net.SplitHostPort(conn.RemoteAddr().String())
	if err != nil {
		return fmt.Errorf("could not extract dim-host ip from %q: %+v", conn.RemoteAddr().String(), err)
	}
	//  dev.cfg.daq.addrs = make([]string, len(dev.rfms))
	//	for i, rfm := range dev.rfms {
	//		difid := difIDFrom(dev.id, i)
	//		dev.cfg.daq.addrs[i] = fmt.Sprintf("%s:%d", dim, 10000+difid)
	//	}

loop:
	for {
		var req struct {
			Name string           `json:"name"`
			Args *json.RawMessage `json:"args"`
		}

		err = json.NewDecoder(conn).Decode(&req)
		if err != nil {
			srv.msg.Printf("could not decode command request: %+v", err)
			srv.reply(conn, err)
			if errors.Is(err, io.EOF) {
				break loop
			}
			continue
		}
		dev.Printf("received request: name=%q", req.Name)

		switch strings.ToLower(req.Name) {
		case "scan":
			var args []struct {
				RFM  int `json:"rfm"`
				EDA  int `json:"eda"`
				Slot int `json:"slot"`
				DAQ  struct {
					RShaper     int `json:"rshaper"`
					TriggerMode int `json:"trigger_type"`
				} `json:"daq_state"`
			}
			err = json.Unmarshal(*req.Args, &args)
			if err != nil {
				srv.msg.Printf("could not decode %q payload: %+v",
					req.Name, err,
				)
				srv.reply(conn, err)
				continue
			}
			dev.rfms = nil
			dev.difs = make(map[int]uint8, nRFM)
			dev.cfg.daq.rfm = 0
			for _, arg := range args {
				dev.msg.Printf(
					"scan: rfm=%d, eda-id=%d, slot-id=%d",
					arg.RFM, arg.EDA, arg.Slot,
				)
				dev.rfms = append(dev.rfms, arg.Slot)
				dev.difs[arg.Slot] = uint8(arg.RFM)
				dev.id = uint32(arg.EDA)
				dev.cfg.daq.rfm |= (1 << arg.Slot)
				dev.cfg.hr.rshaper = uint32(arg.DAQ.RShaper)
			}
			err = nil
			srv.reply(conn, err)
			// FIXME(sbinet): compare expected scan-result with
			// EDA introspection functions.
			// if err != nil {
			// 	srv.msg.Printf("could not scan EDA device: %+v", err)
			// 	continue
			// }

		case "configure":
			var args []struct {
				DIF   uint8         `json:"dif"`
				ASICS []conddb.ASIC `json:"asics"`
			}
			err = json.Unmarshal(*req.Args, &args)
			if err != nil {
				srv.msg.Printf("could not decode %q payload: %+v",
					req.Name, err,
				)
				srv.reply(conn, err)
				continue
			}

			for _, arg := range args {
				// FIXME(sbinet): handle hysteresis, make sure addrs are unique.
				dev.cfg.daq.addrs = append(dev.cfg.daq.addrs, fmt.Sprintf(
					"%s:%d", dim, 10000+int(arg.DIF),
				))
				srv.msg.Printf("addrs: %q", dev.cfg.daq.addrs)

				dev.setDBConfig(arg.DIF, arg.ASICS)

				err = dev.configASICs(arg.DIF)
				if err != nil {
					srv.msg.Printf("could not configure EDA device: %+v", err)
					srv.reply(conn, err)
					continue
				}
			}
			srv.reply(conn, nil)

		case "initialize":
			err = dev.Initialize()
			srv.reply(conn, err)
			if err != nil {
				srv.msg.Printf("could not initialize EDA device: %+v", err)
				continue
			}

		case "start":
			var args []string
			err = json.Unmarshal(*req.Args, &args)
			if err != nil {
				srv.msg.Printf("could not decode %q payload: %+v",
					req.Name, err,
				)
				srv.reply(conn, err)
				continue
			}

			run, err := strconv.Atoi(args[0])
			if err != nil {
				srv.msg.Printf("could not decode run-nbr for start-run (args=%v): %+v",
					req.Args, err,
				)
				srv.reply(conn, err)
				continue
			}
			dev.run = uint32(run)

			err = dev.Start()
			srv.reply(conn, err)
			if err != nil {
				srv.msg.Printf("could not start EDA device: %+v", err)
				continue
			}

		case "stop":
			err = dev.Stop()
			srv.reply(conn, err)
			if err != nil {
				srv.msg.Printf("could not stop EDA device: %+v", err)
				return fmt.Errorf("could not stop EDA device: %w", err)
			}
			break loop

		default:
			srv.msg.Printf("unknown command name=%q, args=%q", req.Name, req.Args)
			err = fmt.Errorf("unknown command %q", req.Name)
			srv.reply(conn, err)
			continue
		}
	}

	return nil
}

func (srv *server) reply(conn net.Conn, err error) {
	rep := struct {
		Msg string `json:"msg"`
	}{"ok"}
	if err != nil {
		rep.Msg = fmt.Sprintf("%+v", err)
	}

	_ = json.NewEncoder(conn).Encode(rep)
}

func (srv *server) close() {
	_ = srv.ctl.Close()
}
