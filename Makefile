.PHONY: build

CROSS_COMPILE = arm-linux-gnueabihf-
CFLAGS = -static -g -Wall -std=c99 -D_GNU_SOURCE=1 -I${SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/include -I.
LDFLAGS =  -g -Wall
CC = $(CROSS_COMPILE)gcc
ARCH= arm


all: build

bin/eda-drv: bin cmd/eda-drv/*.c
	$(CC) $(LDFLAGS) $(CFLAGS) -I./cmd/eda-drv -o $@ cmd/eda-drv/*.c

clean:
	/bin/rm -fr ./bin

bin:
	mkdir ./bin

build: bin/eda-drv

