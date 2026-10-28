CC      ?= cc
CC_FW   := arm-none-eabi-gcc

CPUFLAGS := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
FW_CFLAGS := $(CPUFLAGS) -ffreestanding -nostdlib -O2 -Wall -Wextra \
             -I common/include -I firmware
HOST_CFLAGS := -O2 -Wall -Wextra -I common/include

FW_SRCS := firmware/startup.c firmware/main.c firmware/uart.c \
           common/src/ins_packet.c

.PHONY: all firmware host clean qemu qemu-tcp replay-local

all: firmware host

build:
	mkdir -p build

firmware: build/firmware.elf

build/firmware.elf: $(FW_SRCS) firmware/linker.ld | build
	$(CC_FW) $(FW_CFLAGS) -T firmware/linker.ld -Wl,-Map=build/firmware.map \
	  -o $@ $(FW_SRCS)

host: build/replay

build/replay: host/replay.c common/src/ins_packet.c | build
	$(CC) $(HOST_CFLAGS) -o $@ host/replay.c common/src/ins_packet.c

replay-local: build/replay
	./build/replay

# -nographic already muxes the monitor onto stdio, so -serial stdio
# fails with "cannot use stdio by multiple character devices".
qemu: build/firmware.elf
	qemu-system-arm -machine mps2-an386 -cpu cortex-m4 -display none \
	  -serial stdio -kernel build/firmware.elf

qemu-tcp: build/firmware.elf
	qemu-system-arm -machine mps2-an386 -cpu cortex-m4 -display none \
	  -serial tcp:127.0.0.1:1234,server,nowait -kernel build/firmware.elf

clean:
	rm -rf build
