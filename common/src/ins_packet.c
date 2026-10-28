#include "ins_packet.h"

typedef enum {
    ST_SYNC0 = 0,
    ST_SYNC1,
    ST_TYPE,
    ST_LEN,
    ST_PAYLOAD,
    ST_CRC
} parse_state_t;

static parse_state_t s_state;
static uint8_t s_type;
static uint8_t s_len;
static uint8_t s_idx;
static uint8_t s_crc;
static uint8_t s_payload[INS_PKT_MAX_PAYLOAD];

static uint8_t crc8_update(uint8_t crc, uint8_t byte)
{
    uint8_t i;

    crc ^= byte;
    for (i = 0; i < 8u; i++) {
        if ((crc & 0x80u) != 0u) {
            crc = (uint8_t)((crc << 1) ^ 0x07u);
        } else {
            crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static uint32_t rd_u32le(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static float rd_f32le(const uint8_t *p)
{
    union {
        uint32_t u;
        float f;
    } x;

    x.u = rd_u32le(p);
    return x.f;
}

static void wr_u32le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void wr_f32le(uint8_t *p, float f)
{
    union {
        uint32_t u;
        float f;
    } x;

    x.f = f;
    wr_u32le(p, x.u);
}

void ins_packet_reset(void)
{
    s_state = ST_SYNC0;
    s_type = 0;
    s_len = 0;
    s_idx = 0;
    s_crc = 0;
}

ins_packet_result_t ins_packet_feed(uint8_t byte)
{
    switch (s_state) {
    case ST_SYNC0:
        if (byte == INS_PKT_SYNC0) {
            s_state = ST_SYNC1;
        }
        return INS_PACKET_NEED_MORE;

    case ST_SYNC1:
        if (byte == INS_PKT_SYNC1) {
            s_state = ST_TYPE;
            s_crc = 0;
        } else if (byte != INS_PKT_SYNC0) {
            s_state = ST_SYNC0;
        }
        return INS_PACKET_NEED_MORE;

    case ST_TYPE:
        s_type = byte;
        s_crc = crc8_update(0, byte);
        s_state = ST_LEN;
        return INS_PACKET_NEED_MORE;

    case ST_LEN:
        s_len = byte;
        s_crc = crc8_update(s_crc, byte);
        s_idx = 0;
        if (s_len > INS_PKT_MAX_PAYLOAD) {
            ins_packet_reset();
            return INS_PACKET_BAD;
        }
        if (s_len == 0u) {
            s_state = ST_CRC;
        } else {
            s_state = ST_PAYLOAD;
        }
        return INS_PACKET_NEED_MORE;

    case ST_PAYLOAD:
        s_payload[s_idx] = byte;
        s_crc = crc8_update(s_crc, byte);
        s_idx++;
        if (s_idx >= s_len) {
            s_state = ST_CRC;
        }
        return INS_PACKET_NEED_MORE;

    case ST_CRC:
        if (byte == s_crc) {
            s_state = ST_SYNC0;
            return INS_PACKET_FRAME_READY;
        }
        ins_packet_reset();
        return INS_PACKET_BAD;

    default:
        ins_packet_reset();
        return INS_PACKET_BAD;
    }
}

uint8_t ins_packet_type(void)
{
    return s_type;
}

uint8_t ins_packet_len(void)
{
    return s_len;
}

const uint8_t *ins_packet_payload(void)
{
    return s_payload;
}

size_t ins_packet_encode(uint8_t type, const uint8_t *payload, uint8_t len,
                         uint8_t *out, size_t out_cap)
{
    uint8_t crc;
    uint8_t i;
    size_t need;

    need = (size_t)5u + (size_t)len;
    if (len > INS_PKT_MAX_PAYLOAD || out == NULL || out_cap < need) {
        return 0;
    }
    if (len > 0u && payload == NULL) {
        return 0;
    }

    out[0] = INS_PKT_SYNC0;
    out[1] = INS_PKT_SYNC1;
    out[2] = type;
    out[3] = len;

    crc = crc8_update(0, type);
    crc = crc8_update(crc, len);
    for (i = 0; i < len; i++) {
        out[4u + i] = payload[i];
        crc = crc8_update(crc, payload[i]);
    }
    out[4u + len] = crc;
    return need;
}

void ins_imu_pack(const ins_imu_t *in, uint8_t payload[INS_PKT_IMU_LEN])
{
    wr_u32le(&payload[0], in->t_us);
    wr_f32le(&payload[4], in->acc_mps2[0]);
    wr_f32le(&payload[8], in->acc_mps2[1]);
    wr_f32le(&payload[12], in->acc_mps2[2]);
    wr_f32le(&payload[16], in->gyr_rads[0]);
    wr_f32le(&payload[20], in->gyr_rads[1]);
    wr_f32le(&payload[24], in->gyr_rads[2]);
}

void ins_gnss_pack(const ins_gnss_t *in, uint8_t payload[INS_PKT_GNSS_LEN])
{
    wr_u32le(&payload[0], in->t_us);
    wr_f32le(&payload[4], in->lat_deg);
    wr_f32le(&payload[8], in->lon_deg);
    wr_f32le(&payload[12], in->alt_m);
    payload[16] = in->fix;
}

ins_status_t ins_imu_unpack(const uint8_t *payload, uint8_t len, ins_imu_t *out)
{
    if (payload == NULL || out == NULL || len != INS_PKT_IMU_LEN) {
        return INS_ERR_INVALID;
    }
    out->t_us = rd_u32le(&payload[0]);
    out->acc_mps2[0] = rd_f32le(&payload[4]);
    out->acc_mps2[1] = rd_f32le(&payload[8]);
    out->acc_mps2[2] = rd_f32le(&payload[12]);
    out->gyr_rads[0] = rd_f32le(&payload[16]);
    out->gyr_rads[1] = rd_f32le(&payload[20]);
    out->gyr_rads[2] = rd_f32le(&payload[24]);
    return INS_OK;
}

ins_status_t ins_gnss_unpack(const uint8_t *payload, uint8_t len, ins_gnss_t *out)
{
    if (payload == NULL || out == NULL || len != INS_PKT_GNSS_LEN) {
        return INS_ERR_INVALID;
    }
    out->t_us = rd_u32le(&payload[0]);
    out->lat_deg = rd_f32le(&payload[4]);
    out->lon_deg = rd_f32le(&payload[8]);
    out->alt_m = rd_f32le(&payload[12]);
    out->fix = payload[16];
    return INS_OK;
}

void ins_nav_pack(const ins_nav_t *in, uint8_t payload[INS_PKT_NAV_LEN])
{
    int i;

    wr_u32le(&payload[0], in->t_us);
    for (i = 0; i < 4; i++) {
        wr_f32le(&payload[4 + (i * 4)], in->q_nb[i]);
    }
    for (i = 0; i < 3; i++) {
        wr_f32le(&payload[20 + (i * 4)], in->v_ned[i]);
        wr_f32le(&payload[32 + (i * 4)], in->p_ned[i]);
    }
}

ins_status_t ins_nav_unpack(const uint8_t *payload, uint8_t len, ins_nav_t *out)
{
    int i;

    if (payload == NULL || out == NULL || len != INS_PKT_NAV_LEN) {
        return INS_ERR_INVALID;
    }
    out->t_us = rd_u32le(&payload[0]);
    for (i = 0; i < 4; i++) {
        out->q_nb[i] = rd_f32le(&payload[4 + (i * 4)]);
    }
    for (i = 0; i < 3; i++) {
        out->v_ned[i] = rd_f32le(&payload[20 + (i * 4)]);
        out->p_ned[i] = rd_f32le(&payload[32 + (i * 4)]);
    }
    return INS_OK;
}
