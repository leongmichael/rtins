# rtins

Bare-metal **strapdown INS** with a loosely coupled **15-state error-state EKF**, written in C for ARM Cortex-M4F. v1 runs in **QEMU** (`mps2-an386`) with a host program injecting IMU and GNSS over UART. There is no physical board yet.

Aided sensors in v1: **6-DoF IMU + GNSS position**. No magnetometer, barometer, or GNSS velocity.

## Layout

```
common/     HAL-free C: packets, math, strapdown, EKF
            (compiled for the Mac and for the MCU)
firmware/   startup, linker, UART, main
host/       replay tool and unit tests
datasets/   reserved for logs
```

`common/` does not call `malloc`, libc stdio, or MCU registers.

## Requirements

- `arm-none-eabi-gcc`
- `qemu-system-arm` with machine `mps2-an386`
- host `cc` (clang or gcc)

## Build and test

```bash
make all          # firmware + host + mech/EKF tests
make test-mech
make test-ekf
make replay-local # encode/decode IMU, GNSS, NAV frames
```

## Run in QEMU

Bring-up (one `A` on UART0, then idle):

```bash
make qemu         # Ctrl-C to quit
```

Aided stream (two terminals):

```bash
make qemu-tcp
./build/replay --tcp
```

Replay sends 3 s of still IMU at 100 Hz. The first GNSS fix sets the NED origin; later fixes are 10 m north. Output is TSV you can redirect and plot:

```text
# t_us p_n p_e p_d
0 0.000 0.000 0.000
...
3000000 10.838 0.000 0.003
```

## Frames

UART bytes, little-endian:

```
A5 5A | type | len | payload[len] | crc8
```

CRC-8 (poly `0x07`, init 0) covers `type`, `len`, and payload.

| type | direction | payload |
|------|-----------|---------|
| `0x01` IMU | host → MCU | `t_us`, `f` xyz (m/s²), `ω` xyz (rad/s) — 28 B |
| `0x02` GNSS | host → MCU | `t_us`, lat, lon, alt, fix — 17 B |
| `0x81` NAV | MCU → host | `t_us`, `q_nb[4]`, `v_ned[3]`, `p_ned[3]` — 44 B |

IMU specific force is **body**, x forward, y right, **z down**. Level and still: `f ≈ [0, 0, −g]`.

## Navigation

- Frame: **NED**, position in metres from the first valid GNSS fix
- Attitude: unit quaternion `q_nb` (scalar first, body → NED)
- Mechanization: flat Earth (no Earth rate, no transport rate)
- EKF state: `ψ(3), δv(3), δp(3), b_g(3), b_a(3)`
- GNSS update: **position only**. `fix == 0` is ignored

Q/R in `common/include/ins_ekf.h` is frozen for emulator still+GNSS, not a flight-tuned set.

## Target

QEMU MPS2 AN386, Cortex-M4F, hard-float (`-mfloat-abi=hard -mfpu=fpv4-sp-d16`). Firmware is freestanding (`-nostdlib`). The FPU is enabled in `Reset_Handler` (CPACR).
