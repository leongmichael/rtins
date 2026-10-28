#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "ins_packet.h"

static int encode_demo_frames(uint8_t *imu_frame, size_t imu_cap, size_t *imu_n,
                              uint8_t *gnss_frame, size_t gnss_cap, size_t *gnss_n)
{
    ins_imu_t imu;
    ins_gnss_t gnss;
    uint8_t imu_pl[INS_PKT_IMU_LEN];
    uint8_t gnss_pl[INS_PKT_GNSS_LEN];

    memset(&imu, 0, sizeof(imu));
    imu.t_us = 1000;
    imu.acc_mps2[0] = 0.0f;
    imu.acc_mps2[1] = 0.0f;
    imu.acc_mps2[2] = -9.80665f; /* NED body-z-down, still */
    imu.gyr_rads[0] = 0.0f;
    imu.gyr_rads[1] = 0.0f;
    imu.gyr_rads[2] = 0.0f;
    ins_imu_pack(&imu, imu_pl);
    *imu_n = ins_packet_encode(INS_PKT_TYPE_IMU, imu_pl, INS_PKT_IMU_LEN,
                               imu_frame, imu_cap);

    memset(&gnss, 0, sizeof(gnss));
    gnss.t_us = 1000;
    gnss.lat_deg = 37.4275f;
    gnss.lon_deg = -122.1697f;
    gnss.alt_m = 30.0f;
    gnss.fix = 1;
    ins_gnss_pack(&gnss, gnss_pl);
    *gnss_n = ins_packet_encode(INS_PKT_TYPE_GNSS, gnss_pl, INS_PKT_GNSS_LEN,
                                gnss_frame, gnss_cap);

    if (*imu_n == 0u || *gnss_n == 0u) {
        return -1;
    }
    return 0;
}

static int local_self_test(const uint8_t *imu_frame, size_t imu_n,
                           const uint8_t *gnss_frame, size_t gnss_n)
{
    size_t i;
    ins_packet_result_t r;
    ins_imu_t imu;
    ins_gnss_t gnss;
    int ready;

    ins_packet_reset();
    ready = 0;
    for (i = 0; i < imu_n; i++) {
        r = ins_packet_feed(imu_frame[i]);
        if (r == INS_PACKET_BAD) {
            fprintf(stderr, "local: IMU frame rejected\n");
            return -1;
        }
        if (r == INS_PACKET_FRAME_READY) {
            ready++;
        }
    }
    if (ready != 1 || ins_packet_type() != INS_PKT_TYPE_IMU) {
        fprintf(stderr, "local: expected one IMU frame\n");
        return -1;
    }
    if (ins_imu_unpack(ins_packet_payload(), ins_packet_len(), &imu) != INS_OK) {
        fprintf(stderr, "local: IMU unpack failed\n");
        return -1;
    }
    printf("local IMU t=%u az=%.2f gx=%.3f\n",
           (unsigned)imu.t_us, (double)imu.acc_mps2[2], (double)imu.gyr_rads[0]);

    ins_packet_reset();
    ready = 0;
    for (i = 0; i < gnss_n; i++) {
        r = ins_packet_feed(gnss_frame[i]);
        if (r == INS_PACKET_BAD) {
            fprintf(stderr, "local: GNSS frame rejected\n");
            return -1;
        }
        if (r == INS_PACKET_FRAME_READY) {
            ready++;
        }
    }
    if (ready != 1 || ins_packet_type() != INS_PKT_TYPE_GNSS) {
        fprintf(stderr, "local: expected one GNSS frame\n");
        return -1;
    }
    if (ins_gnss_unpack(ins_packet_payload(), ins_packet_len(), &gnss) != INS_OK) {
        fprintf(stderr, "local: GNSS unpack failed\n");
        return -1;
    }
    printf("local GNSS lat=%.4f lon=%.4f fix=%u\n",
           (double)gnss.lat_deg, (double)gnss.lon_deg, (unsigned)gnss.fix);
    return 0;
}

static int send_all(int fd, const uint8_t *buf, size_t n)
{
    size_t off = 0;

    while (off < n) {
        ssize_t w = write(fd, buf + off, n - off);
        if (w <= 0) {
            return -1;
        }
        off += (size_t)w;
    }
    return 0;
}

static int tcp_send(const char *host, uint16_t port,
                    const uint8_t *imu_frame, size_t imu_n,
                    const uint8_t *gnss_frame, size_t gnss_n)
{
    int fd;
    struct sockaddr_in addr;
    char reply[8];
    ssize_t n;

    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("socket");
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
        fprintf(stderr, "bad host %s\n", host);
        close(fd);
        return -1;
    }

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        perror("connect");
        close(fd);
        return -1;
    }

    if (send_all(fd, imu_frame, imu_n) != 0 ||
        send_all(fd, gnss_frame, gnss_n) != 0) {
        perror("write");
        close(fd);
        return -1;
    }

    /* Firmware ACKs IMU with 'I' and GNSS with 'G'. The boot 'A' may
     * already have been dropped if we connected after reset. */
    {
        size_t got = 0;
        int waits;

        for (waits = 0; waits < 20 && got < sizeof(reply); waits++) {
            fd_set rfds;
            struct timeval tv;
            ssize_t r;

            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);
            tv.tv_sec = 0;
            tv.tv_usec = 50000;
            if (select(fd + 1, &rfds, NULL, NULL, &tv) <= 0) {
                if (got > 0) {
                    break;
                }
                continue;
            }
            r = read(fd, reply + got, sizeof(reply) - got);
            if (r <= 0) {
                break;
            }
            got += (size_t)r;
        }
        n = (ssize_t)got;
    }
    if (n > 0) {
        fwrite(reply, 1, (size_t)n, stdout);
        fputc('\n', stdout);
    }

    close(fd);
    return 0;
}

int main(int argc, char **argv)
{
    uint8_t imu_frame[64];
    uint8_t gnss_frame[64];
    size_t imu_n;
    size_t gnss_n;
    const char *host = "127.0.0.1";
    uint16_t port = 1234;
    int use_tcp = 0;

    if (argc >= 2 && strcmp(argv[1], "--tcp") == 0) {
        use_tcp = 1;
        if (argc >= 3) {
            host = argv[2];
        }
        if (argc >= 4) {
            port = (uint16_t)atoi(argv[3]);
        }
    }

    if (encode_demo_frames(imu_frame, sizeof(imu_frame), &imu_n,
                           gnss_frame, sizeof(gnss_frame), &gnss_n) != 0) {
        fprintf(stderr, "encode failed\n");
        return 1;
    }

    if (local_self_test(imu_frame, imu_n, gnss_frame, gnss_n) != 0) {
        return 1;
    }

    if (!use_tcp) {
        return 0;
    }

    printf("sending IMU+GNSS to %s:%u\n", host, (unsigned)port);
    if (tcp_send(host, port, imu_frame, imu_n, gnss_frame, gnss_n) != 0) {
        return 1;
    }
    return 0;
}
