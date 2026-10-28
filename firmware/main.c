#include "ins_ekf.h"
#include "ins_packet.h"
#include "uart.h"

static void nav_downlink(const ins_ekf_t *ekf)
{
    ins_nav_t nav;
    uint8_t payload[INS_PKT_NAV_LEN];
    uint8_t frame[5u + INS_PKT_NAV_LEN];
    size_t n;
    int i;

    nav.t_us = ekf->mech.t_us;
    for (i = 0; i < 4; i++) {
        nav.q_nb[i] = ekf->mech.q_nb[i];
    }
    for (i = 0; i < 3; i++) {
        nav.v_ned[i] = ekf->mech.v_ned[i];
        nav.p_ned[i] = ekf->mech.p_ned[i];
    }
    ins_nav_pack(&nav, payload);
    n = ins_packet_encode(INS_PKT_TYPE_NAV, payload, INS_PKT_NAV_LEN,
                          frame, sizeof(frame));
    if (n > 0u) {
        uart_write(frame, n);
    }
}

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
                    nav_downlink(&ekf);
                }
            } else if (ins_packet_type() == INS_PKT_TYPE_GNSS) {
                if (ins_gnss_unpack(ins_packet_payload(), ins_packet_len(),
                                    &gnss) == INS_OK) {
                    ins_ekf_gnss(&ekf, &gnss);
                }
            }
            break;
        case INS_PACKET_BAD:
        case INS_PACKET_NEED_MORE:
        default:
            break;
        }
    }
}
