# Dual-target skeleton. Rules are not written yet.
#
# Firmware (arm-none-eabi-gcc), once you implement the image:
#   -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
#   -ffreestanding -nostdlib          # if you keep firmware libc-free
#   -T firmware/linker.ld
#
# Host (cc / clang):
#   -I common/include
#   common/src/ins_packet.c host/replay.c
#
# QEMU hello (stdio):
#   qemu-system-arm -machine mps2-an386 -cpu cortex-m4 -nographic \
#     -kernel build/firmware.elf -serial stdio
#
# QEMU data plane (TCP):
#   qemu-system-arm -machine mps2-an386 -cpu cortex-m4 -nographic \
#     -kernel build/firmware.elf -serial tcp:127.0.0.1:1234,server,nowait

.PHONY: all firmware host
all:
	@echo "Build rules not written yet."
