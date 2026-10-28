#include "ins_ekf.h"
#include "ins_packet.h"
#include "uart.h"

int main(void)
{
    static ins_ekf_t ekf;
    uint8_t b;
    ins_imu_t imu;
    ins_gnss_t gnss;

    uart_init();
    uart_putc((uint8_t)'A');

    ins_packet_reset();
    ins_ekf_init(&ekf);

    for (;;) {
        if (!uart_try_getc(&b)) {
            continue;
        }

        switch (ins_packet_feed(b)) {
        case INS_PACKET_FRAME_READY:
            if (ins_packet_type() == INS_PKT_TYPE_IMU) {
                if (ins_imu_unpack(ins_packet_payload(), ins_packet_len(),
                                   &imu) == INS_OK) {
                    ins_ekf_imu(&ekf, &imu);
                }
                uart_putc((uint8_t)'I');
            } else if (ins_packet_type() == INS_PKT_TYPE_GNSS) {
                if (ins_gnss_unpack(ins_packet_payload(), ins_packet_len(),
                                    &gnss) == INS_OK) {
                    ins_ekf_gnss(&ekf, &gnss);
                }
                uart_putc((uint8_t)'G');
            } else {
                uart_putc((uint8_t)'?');
            }
            break;
        case INS_PACKET_BAD:
            uart_putc((uint8_t)'!');
            break;
        case INS_PACKET_NEED_MORE:
        default:
            break;
        }
    }
}
