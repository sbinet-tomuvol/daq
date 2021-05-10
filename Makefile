.PHONY: build

CROSS_COMPILE = arm-linux-gnueabihf-
CFLAGS = -g -Wall -Wunused-result -std=c99 -D_GNU_SOURCE=1 -I${SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/include -I.
LDFLAGS = -pthread -static
CC = $(CROSS_COMPILE)gcc
ARCH = arm


all: build

bin/eda-drv: bin
	GOARCH=arm CC=$(CC) CC_FOR_TARGET=$(CC) CGO_ENABLED=1 \
		   go build -o ./bin/eda-drv -v -tags=netgo ./cmd/eda-drv

clean:
	/bin/rm -fr ./bin

bin:
	mkdir ./bin

build: bin/eda-drv

