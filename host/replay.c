#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "ins_packet.h"

#define LAT0 37.4275f
#define LON0 (-122.1697f)
#define ALT0 30.0f
#define RE_M 6378137.0f
#define IMU_HZ 100u
#define STREAM_N 301u /* 3 s including t=0 */

static void still_imu(ins_imu_t *imu, ins_time_t t_us)
{
    memset(imu, 0, sizeof(*imu));
    imu->t_us = t_us;
    imu->acc_mps2[2] = -9.80665f;
}

static int encode_imu(const ins_imu_t *imu, uint8_t *frame, size_t cap, size_t *n)
{
    uint8_t pl[INS_PKT_IMU_LEN];

    ins_imu_pack(imu, pl);
    *n = ins_packet_encode(INS_PKT_TYPE_IMU, pl, INS_PKT_IMU_LEN, frame, cap);
    return (*n == 0u) ? -1 : 0;
}

static int encode_gnss(const ins_gnss_t *gnss, uint8_t *frame, size_t cap, size_t *n)
{
    uint8_t pl[INS_PKT_GNSS_LEN];

    ins_gnss_pack(gnss, pl);
    *n = ins_packet_encode(INS_PKT_TYPE_GNSS, pl, INS_PKT_GNSS_LEN, frame, cap);
    return (*n == 0u) ? -1 : 0;
}

static int local_self_test(void)
{
    ins_imu_t imu;
    ins_gnss_t gnss;
    ins_nav_t nav, nav_out;
    uint8_t frame[64];
    uint8_t pl[INS_PKT_NAV_LEN];
    size_t n, i;
    int ready;

    still_imu(&imu, 1000);
    if (encode_imu(&imu, frame, sizeof(frame), &n) != 0) {
        return -1;
    }
    ins_packet_reset();
    ready = 0;
    for (i = 0; i < n; i++) {
        if (ins_packet_feed(frame[i]) == INS_PACKET_FRAME_READY) {
            ready++;
        }
    }
    if (ready != 1 || ins_imu_unpack(ins_packet_payload(), ins_packet_len(),
                                     &imu) != INS_OK) {
        fprintf(stderr, "local: IMU failed\n");
        return -1;
    }
    printf("local IMU t=%u az=%.2f\n", (unsigned)imu.t_us,
           (double)imu.acc_mps2[2]);

    memset(&gnss, 0, sizeof(gnss));
    gnss.t_us = 1000;
    gnss.lat_deg = LAT0;
    gnss.lon_deg = LON0;
    gnss.alt_m = ALT0;
    gnss.fix = 1;
    if (encode_gnss(&gnss, frame, sizeof(frame), &n) != 0) {
        return -1;
    }
    ins_packet_reset();
    ready = 0;
    for (i = 0; i < n; i++) {
        if (ins_packet_feed(frame[i]) == INS_PACKET_FRAME_READY) {
            ready++;
        }
    }
    if (ready != 1 || ins_gnss_unpack(ins_packet_payload(), ins_packet_len(),
                                      &gnss) != INS_OK) {
        fprintf(stderr, "local: GNSS failed\n");
        return -1;
    }
    printf("local GNSS lat=%.4f fix=%u\n", (double)gnss.lat_deg,
           (unsigned)gnss.fix);

    memset(&nav, 0, sizeof(nav));
    nav.t_us = 2000;
    nav.q_nb[0] = 1.0f;
    nav.p_ned[0] = 1.25f;
    ins_nav_pack(&nav, pl);
    n = ins_packet_encode(INS_PKT_TYPE_NAV, pl, INS_PKT_NAV_LEN, frame,
                          sizeof(frame));
    ins_packet_reset();
    ready = 0;
    for (i = 0; i < n; i++) {
        if (ins_packet_feed(frame[i]) == INS_PACKET_FRAME_READY) {
            ready++;
        }
    }
    if (ready != 1 || ins_packet_type() != INS_PKT_TYPE_NAV ||
        ins_nav_unpack(ins_packet_payload(), ins_packet_len(), &nav_out)
            != INS_OK ||
        fabsf(nav_out.p_ned[0] - 1.25f) > 1.0e-5f) {
        fprintf(stderr, "local: NAV failed\n");
        return -1;
    }
    printf("local NAV p_n=%.2f\n", (double)nav_out.p_ned[0]);
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

static int wait_nav(int fd, ins_nav_t *nav, int timeout_ms)
{
    uint8_t b;
    int elapsed = 0;

    while (elapsed < timeout_ms) {
        fd_set rfds;
        struct timeval tv;
        ssize_t r;

        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 20000;
        if (select(fd + 1, &rfds, NULL, NULL, &tv) <= 0) {
            elapsed += 20;
            continue;
        }
        r = read(fd, &b, 1);
        if (r <= 0) {
            return -1;
        }
        if (ins_packet_feed(b) == INS_PACKET_FRAME_READY &&
            ins_packet_type() == INS_PKT_TYPE_NAV) {
            return ins_nav_unpack(ins_packet_payload(), ins_packet_len(), nav)
                           == INS_OK
                       ? 0
                       : -1;
        }
        elapsed = 0;
    }
    return -1;
}

static int tcp_stream(const char *host, uint16_t port)
{
    int fd;
    struct sockaddr_in addr;
    ins_imu_t imu;
    ins_gnss_t gnss;
    ins_nav_t nav;
    uint8_t frame[64];
    size_t n;
    unsigned i;
    float lat_pull;
    ins_nav_t last;

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

    ins_packet_reset();
    memset(&last, 0, sizeof(last));
    lat_pull = LAT0 + (float)((10.0 / (double)RE_M) * (180.0 / (double)M_PI));

    printf("# t_us p_n p_e p_d\n");

    for (i = 0; i < STREAM_N; i++) {
        still_imu(&imu, i * (1000000u / IMU_HZ));
        if (encode_imu(&imu, frame, sizeof(frame), &n) != 0 ||
            send_all(fd, frame, n) != 0) {
            fprintf(stderr, "send IMU failed at %u\n", i);
            close(fd);
            return -1;
        }
        if (wait_nav(fd, &nav, 500) != 0) {
            fprintf(stderr, "no NAV after IMU %u\n", i);
            close(fd);
            return -1;
        }
        last = nav;
        if ((i % 50u) == 0u) {
            printf("%u %.3f %.3f %.3f\n", (unsigned)nav.t_us,
                   (double)nav.p_ned[0], (double)nav.p_ned[1],
                   (double)nav.p_ned[2]);
        }

        /* First GNSS sets origin. Later fixes are 10 m north so p_n should
         * walk toward ~10. */
        if (i == 50u || i == 100u || i == 200u || i == 300u) {
            memset(&gnss, 0, sizeof(gnss));
            gnss.t_us = imu.t_us;
            gnss.lat_deg = (i == 50u) ? LAT0 : lat_pull;
            gnss.lon_deg = LON0;
            gnss.alt_m = ALT0;
            gnss.fix = 1;
            if (encode_gnss(&gnss, frame, sizeof(frame), &n) != 0 ||
                send_all(fd, frame, n) != 0) {
                fprintf(stderr, "send GNSS failed\n");
                close(fd);
                return -1;
            }
        }
    }

    printf("# last p_n=%.3f m (GNSS pull toward ~10 after origin fix)\n",
           (double)last.p_ned[0]);
    close(fd);
    return 0;
}

int main(int argc, char **argv)
{
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

    if (local_self_test() != 0) {
        return 1;
    }
    if (!use_tcp) {
        return 0;
    }
    printf("streaming still IMU + GNSS to %s:%u\n", host, (unsigned)port);
    return tcp_stream(host, port) != 0 ? 1 : 0;
}
