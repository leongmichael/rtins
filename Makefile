CC      ?= cc
CC_FW   := arm-none-eabi-gcc

CPUFLAGS := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
FW_CFLAGS := $(CPUFLAGS) -ffreestanding -nostdlib -O2 -Wall -Wextra \
             -I common/include -I firmware
HOST_CFLAGS := -O2 -Wall -Wextra -I common/include
HOST_LDLIBS := -lm

COMMON_SRCS := common/src/ins_packet.c common/src/ins_math.c \
               common/src/ins_mech.c
EKF_SRCS := $(COMMON_SRCS) common/src/ins_ekf.c

FW_SRCS := firmware/startup.c firmware/main.c firmware/uart.c \
           $(COMMON_SRCS)

.PHONY: all firmware host clean qemu qemu-tcp replay-local test-mech test-ekf

all: firmware host test-mech test-ekf

build:
	mkdir -p build

firmware: build/firmware.elf

build/firmware.elf: $(FW_SRCS) firmware/linker.ld | build
	$(CC_FW) $(FW_CFLAGS) -T firmware/linker.ld -Wl,-Map=build/firmware.map \
	  -o $@ $(FW_SRCS)

host: build/replay build/test_mech build/test_ekf

build/replay: host/replay.c common/src/ins_packet.c | build
	$(CC) $(HOST_CFLAGS) -o $@ host/replay.c common/src/ins_packet.c

build/test_mech: host/test_mech.c $(COMMON_SRCS) | build
	$(CC) $(HOST_CFLAGS) -o $@ host/test_mech.c $(COMMON_SRCS) $(HOST_LDLIBS)

replay-local: build/replay
	./build/replay

test-mech: build/test_mech
	./build/test_mech

build/test_ekf: host/test_ekf.c $(EKF_SRCS) | build
	$(CC) $(HOST_CFLAGS) -o $@ host/test_ekf.c $(EKF_SRCS) $(HOST_LDLIBS)

test-ekf: build/test_ekf
	./build/test_ekf

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
