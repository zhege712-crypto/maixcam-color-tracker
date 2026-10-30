/**
 * @file mx_protocol.c
 * @brief 相机 <-> 主控 串口协议 v4.0 —— 解析与组帧实现（接口见 mx_protocol.h）
 */

#include "mx_protocol.h"

#include <string.h>

/* 帧头/帧尾集中放这里，解析和组帧共用，避免两处各写一份 */
static const uint8_t s_up_hdr[MX_UPLINK_HEADER_LEN] = {
    MX_UPLINK_HDR0, MX_UPLINK_HDR1, MX_UPLINK_HDR2, MX_UPLINK_HDR3
};
static const uint8_t s_up_tail[MX_UPLINK_TAIL_LEN] = {
    MX_UPLINK_TAIL0, MX_UPLINK_TAIL1, MX_UPLINK_TAIL2, MX_UPLINK_TAIL3
};

/* ---------------------------------------------------------------- 小端工具 */

/* 手写小端而不是用结构体/联合体：协议规定小端，主机端字节序不可信 */
static void mx_put_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static uint16_t mx_get_u16le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

/* ---------------------------------------------------------------- 校验和 */

uint8_t mx_uplink_checksum(const uint8_t *payload)
{
    uint8_t sum = 0U;

    if (payload == NULL) {
        return 0U;
    }
    /* 从偏移 4 开始、共 11 字节（到偏移 14 为止）。帧头不参与累加，
       因为帧头是固定常量，算进去只会平白抬高校验和，文档示例也证明
       校验 0x94 只由偏移 4-14 得到。 */
    for (uint8_t i = 0U; i < MX_UPLINK_SUM_LEN; i++) {
        sum = (uint8_t)(sum + payload[i]);
    }
    return sum;
}

/* ---------------------------------------------------------------- 角度/色环 */

uint8_t mx_rotation_encode(int angle_deg, bool is_ring)
{
    const uint8_t ring = is_ring ? (uint8_t)MX_ROTATION_RING_BIT : 0U;
    int steps;

    /* 先夹紧再乘，避免 angle_deg 很大时 int 溢出 */
    if (angle_deg <= 0) {
        return ring;
    }
    if (angle_deg > 360) {
        return (uint8_t)(ring | MX_ROTATION_ANGLE_STEPS);
    }
    /* 四舍五入到最近的 1/127 圈。用整数运算，避免引入 libm 的 round()，
       同时保证和 protocol.py 的 round() 结果一致（0..360 内没有会造成
       银行家舍入分歧的 .5 点，只有 180 度，两边都进到 64）。 */
    steps = (angle_deg * (int)MX_ROTATION_ANGLE_STEPS + 180) / 360;
    if (steps > (int)MX_ROTATION_ANGLE_STEPS) {
        steps = (int)MX_ROTATION_ANGLE_STEPS;
    }
    return (uint8_t)(ring | (uint8_t)steps);
}

void mx_rotation_decode(uint8_t rotation, uint8_t *angle_steps, bool *is_ring)
{
    if (angle_steps != NULL) {
        *angle_steps = (uint8_t)(rotation & MX_ROTATION_ANGLE_MASK);
    }
    if (is_ring != NULL) {
        *is_ring = (rotation & MX_ROTATION_RING_BIT) != 0U;
    }
}

/* ---------------------------------------------------------------- 上行组帧 */

int mx_build_uplink(const mx_uplink_t *f, uint8_t *out)
{
    if (f == NULL || out == NULL) {
        return MX_ERR_ARG;
    }

    memcpy(out, s_up_hdr, MX_UPLINK_HEADER_LEN);
    out[MX_OFF_EXISTS]   = f->exists;
    out[MX_OFF_COLOR]    = f->color;
    out[MX_OFF_ROTATION] = f->rotation;
    mx_put_u16le(&out[MX_OFF_X], f->x);
    mx_put_u16le(&out[MX_OFF_Y], f->y);
    mx_put_u16le(&out[MX_OFF_W], f->w);
    mx_put_u16le(&out[MX_OFF_H], f->h);
    /* 校验和必须在载荷全部写完之后算 */
    out[MX_OFF_CHECKSUM] = mx_uplink_checksum(&out[MX_OFF_EXISTS]);
    memcpy(&out[MX_OFF_TAIL], s_up_tail, MX_UPLINK_TAIL_LEN);

    return MX_OK;
}

/* ---------------------------------------------------------------- 上行解析 */

int mx_parse_uplink(const uint8_t *frame, mx_uplink_t *out)
{
    if (frame == NULL) {
        return MX_ERR_ARG;
    }

    /* 先查固定结构再算校验和：坏帧大多是帧头/帧尾错位，查这两处更快，
       也便于把具体错误码报给调试口。 */
    if (memcmp(frame, s_up_hdr, MX_UPLINK_HEADER_LEN) != 0) {
        return MX_ERR_HEADER;
    }
    if (memcmp(&frame[MX_OFF_TAIL], s_up_tail, MX_UPLINK_TAIL_LEN) != 0) {
        return MX_ERR_TAIL;
    }
    if (mx_uplink_checksum(&frame[MX_OFF_EXISTS]) != frame[MX_OFF_CHECKSUM]) {
        return MX_ERR_CHECKSUM;
    }

    if (out != NULL) {
        out->exists   = frame[MX_OFF_EXISTS];
        out->color    = frame[MX_OFF_COLOR];
        out->rotation = frame[MX_OFF_ROTATION];
        out->x = mx_get_u16le(&frame[MX_OFF_X]);
        out->y = mx_get_u16le(&frame[MX_OFF_Y]);
        out->w = mx_get_u16le(&frame[MX_OFF_W]);
        out->h = mx_get_u16le(&frame[MX_OFF_H]);
    }
    return MX_OK;
}

/* ---------------------------------------------------------------- 下行 */

bool mx_downlink_cmd_valid(uint8_t cmd)
{
    return cmd == MX_CMD_RED ||
           cmd == MX_CMD_GREEN ||
           cmd == MX_CMD_BLUE ||
           cmd == MX_CMD_PING;
}

int mx_build_downlink(uint8_t cmd, uint8_t *out)
{
    if (out == NULL) {
        return MX_ERR_ARG;
    }
    if (!mx_downlink_cmd_valid(cmd)) {
        /* 与 protocol.py 抛异常的行为对齐：非法指令一个字节都不写，
           避免总线上冒出相机不认识的 4 字节。 */
        return MX_ERR_CMD;
    }
    out[0] = MX_DOWNLINK_HDR0;
    out[1] = MX_DOWNLINK_HDR1;
    out[2] = cmd;
    out[3] = MX_DOWNLINK_TAIL;
    return MX_OK;
}

int mx_parse_downlink(const uint8_t *frame, uint8_t *cmd)
{
    if (frame == NULL) {
        return MX_ERR_ARG;
    }
    if (frame[0] != MX_DOWNLINK_HDR0 || frame[1] != MX_DOWNLINK_HDR1) {
        return MX_ERR_HEADER;
    }
    if (frame[3] != MX_DOWNLINK_TAIL) {
        return MX_ERR_TAIL;
    }
    if (!mx_downlink_cmd_valid(frame[2])) {
        return MX_ERR_CMD;
    }
    if (cmd != NULL) {
        *cmd = frame[2];
    }
    return MX_OK;
}

/* ---------------------------------------------------------------- 流式解析 */

void mx_uplink_parser_init(mx_uplink_parser_t *p)
{
    if (p == NULL) {
        return;
    }
    memset(p, 0, sizeof(*p));
}

/*
 * 核心：把窗口 buf[0..len) 当作“以候选帧头开头的 20 字节帧”。
 * 对不上帧头就丢首字节继续滑，凑满 20 字节就整帧校验，失败也只丢 1 字节。
 * 这样粘包（一次来多帧）、拆包（一帧分多次来）、错位（坏帧内部可能藏着
 * 下一帧的帧头）都用同一套逻辑处理，等价于 protocol.py 里
 * find(header) + del buf[:1] 的循环。
 *
 * 一个字节最多解出一帧：解出后窗口清零，不可能在同一字节上再凑满一帧。
 */
static int mx_parser_push(mx_uplink_parser_t *p, uint8_t byte, mx_uplink_t *out)
{
    /* 正常路径下 len 恒 < 20（满 20 立即校验并清零/减一），这里只是防御 */
    if (p->len >= (uint8_t)MX_UPLINK_FRAME_LEN) {
        p->len = 0U;
    }
    p->buf[p->len] = byte;
    p->len++;

    for (;;) {
        if (p->len < (uint8_t)MX_UPLINK_HEADER_LEN) {
            /* 还不够 4 字节，无法判断是不是帧头；最多缓存 3 字节，不会涨 */
            return 0;
        }
        if (memcmp(p->buf, s_up_hdr, MX_UPLINK_HEADER_LEN) != 0) {
            /* 当前窗口不是帧头开头：丢掉最左边 1 字节重新对齐。
               不能一次丢 4 字节，因为 AA AA BB AA AA BB BB 这种
               互相重叠的帧头要靠逐字节滑动才能找到正确起点。 */
            p->len--;
            memmove(p->buf, &p->buf[1], p->len);
            continue;
        }
        if (p->len < (uint8_t)MX_UPLINK_FRAME_LEN) {
            /* 帧头已对上，继续等剩余字节（拆包） */
            return 0;
        }
        if (mx_parse_uplink(p->buf, out) == MX_OK) {
            p->len = 0U;
            p->ok_frames++;
            return 1;
        }
        /* 帧头对上但帧尾/校验和错：只丢 1 字节。若整帧丢弃，藏在坏帧
           后半段里的下一帧帧头也会被一起扔掉，链路会一直卡住。 */
        p->bad_frames++;
        p->len--;
        memmove(p->buf, &p->buf[1], p->len);
    }
}

int mx_uplink_parser_feed_byte(mx_uplink_parser_t *p, uint8_t byte,
                               mx_uplink_t *out)
{
    if (p == NULL) {
        return MX_ERR_ARG;
    }
    return mx_parser_push(p, byte, out);
}

int mx_uplink_parser_feed(mx_uplink_parser_t *p, const uint8_t *data, size_t len,
                          mx_uplink_cb_t cb, void *user)
{
    int count = 0;

    if (p == NULL || (data == NULL && len != 0U)) {
        return MX_ERR_ARG;
    }
    for (size_t i = 0U; i < len; i++) {
        mx_uplink_t f;
        if (mx_parser_push(p, data[i], &f) == 1) {
            count++;
            if (cb != NULL) {
                cb(&f, user);
            }
        }
    }
    return count;
}
