/**
 * @file mx_protocol.c
 * @brief 相机 <-> 主控 串口协议 v2.0 —— 解析与组帧实现（详见 mx_protocol.h）
 */

#include "mx_protocol.h"
#include <string.h>

static const uint8_t UP_HDR[4]   = { 0xAAU, 0xAAU, 0xBBU, 0xBBU };
static const uint8_t UP_TAIL[4]  = { 0xFFU, 0xFFU, 0xEEU, 0xEEU };
static const uint8_t DOWN_HDR[4] = { 0xCCU, 0xCCU, 0xDDU, 0xDDU };
static const uint8_t DOWN_TAIL[4] = { 0xFFU, 0xFFU, 0xEEU, 0xEEU };

/* ---------------------------------------------------------------- 校验和 */

uint8_t mx_uplink_checksum(const uint8_t *payload8)
{
    uint8_t sum = 0U;
    for (uint8_t i = 0U; i < 8U; i++) {
        sum = (uint8_t)(sum + payload8[i]);
    }
    return sum;
}

/* ---------------------------------------------------------------- 上行解析 */

static bool mx_validate_uplink(const uint8_t *b, mx_uplink_t *out)
{
    /* 帧尾 */
    for (uint8_t i = 0U; i < 4U; i++) {
        if (b[MX_UP_FRAME_LEN - 4U + i] != UP_TAIL[i]) {
            return false;
        }
    }
    /* 校验和：偏移 4-11 共 8 字节 */
    if (mx_uplink_checksum(&b[MX_OFF_SEQ]) != b[MX_OFF_CHECKSUM]) {
        return false;
    }
    if (out != NULL) {
        out->seq      = b[MX_OFF_SEQ];
        out->exists   = b[MX_OFF_EXISTS];
        out->color    = b[MX_OFF_COLOR];
        out->state    = b[MX_OFF_STATE];
        out->x_n      = b[MX_OFF_XN];
        out->y_n      = b[MX_OFF_YN];
        out->w_n      = b[MX_OFF_WN];
        out->h_n      = b[MX_OFF_HN];
    }
    return true;
}

void mx_parser_init(mx_parser_t *p)
{
    if (p == NULL) {
        return;
    }
    memset(p, 0, sizeof(*p));
    p->state = 0U;
}

/* allow_resync=false 时不再递归重同步，避免深层嵌套 */
static bool mx_feed_byte(mx_parser_t *p, uint8_t byte, mx_uplink_t *out, bool allow_resync)
{
    /* ---- 阶段一：匹配 4 字节帧头 ---- */
    if (p->state < 4U) {
        if (byte == UP_HDR[p->state]) {
            p->buf[p->state] = byte;
            p->state++;
        } else if (byte == UP_HDR[0]) {
            /* 可能是新帧头的第一个字节（例如 AA AA BB AA AA BB BB） */
            p->buf[0] = byte;
            p->state = 1U;
        } else {
            p->state = 0U;
        }
        return false;
    }

    /* ---- 阶段二：收集其余字节，凑满 17 字节后校验 ---- */
    p->buf[p->state] = byte;
    p->state++;
    if (p->state < MX_UP_FRAME_LEN) {
        return false;
    }
    p->state = 0U;

    if (mx_validate_uplink(p->buf, out)) {
        p->ok_frames++;
        return true;
    }

    p->bad_frames++;

    if (allow_resync) {
        /* 帧尾或校验和不合法：把这一帧里除首字节外的内容重新过一遍状态机。
           串口是流式传输，一次错位可能让后续所有帧都错位，重同步能自己爬回来。 */
        uint8_t tmp[MX_UP_FRAME_LEN];
        memcpy(tmp, p->buf, sizeof(tmp));
        for (uint8_t i = 1U; i < MX_UP_FRAME_LEN; i++) {
            mx_uplink_t candidate;
            if (mx_feed_byte(p, tmp[i], &candidate, false)) {
                if (out != NULL) {
                    *out = candidate;
                }
                return true;
            }
        }
    }
    return false;
}

bool mx_parser_feed(mx_parser_t *p, uint8_t byte, mx_uplink_t *out)
{
    if (p == NULL) {
        return false;
    }
    return mx_feed_byte(p, byte, out, true);
}

/* ---------------------------------------------------------------- 下行组帧 */

void mx_build_downlink(uint8_t cmd, uint8_t arg, uint8_t *out)
{
    if (out == NULL) {
        return;
    }
    memcpy(out, DOWN_HDR, 4U);
    out[4] = cmd;
    out[5] = arg;
    out[6] = (uint8_t)(cmd + arg);
    memcpy(&out[7], DOWN_TAIL, 4U);
}

void mx_build_ack(uint8_t cmd, uint8_t status, uint8_t *out)
{
    if (out == NULL) {
        return;
    }
    uint8_t ack_cmd = (uint8_t)(cmd + 0x80U);
    memcpy(out, DOWN_HDR, 4U);
    out[4] = ack_cmd;
    out[5] = status;
    out[6] = (uint8_t)(ack_cmd + status);
    memcpy(&out[7], DOWN_TAIL, 4U);
}

bool mx_parse_downlink(const uint8_t *frame, uint8_t *cmd, uint8_t *arg)
{
    if (frame == NULL) {
        return false;
    }
    for (uint8_t i = 0U; i < 4U; i++) {
        if (frame[i] != DOWN_HDR[i]) {
            return false;
        }
        if (frame[7U + i] != DOWN_TAIL[i]) {
            return false;
        }
    }
    if ((uint8_t)(frame[4] + frame[5]) != frame[6]) {
        return false;
    }
    if (cmd != NULL) {
        *cmd = frame[4];
    }
    if (arg != NULL) {
        *arg = frame[5];
    }
    return true;
}
