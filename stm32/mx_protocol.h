/**
 * @file    mx_protocol.h
 * @brief   相机(MaixCAM) <-> 主控(STM32H723) 串口协议 v2.0 —— 纯 C 实现
 *
 * 对应文档：相机_主控串口通信协议_定稿.md
 *
 * 上行（相机 -> 主控）17 字节：
 *   0-3   帧头 AA AA BB BB
 *   4     帧序号 seq
 *   5     目标是否存在 01/00
 *   6     目标颜色 01红 02绿 03蓝
 *   7     旋转角度（圆柱物块恒 0）
 *   8     x_n   0-255 横向相对位置，中心约 128
 *   9     y_n   0-255 纵向相对位置，中心约 128
 *   10    w_n   0-255 长边/画面宽*255
 *   11    h_n   0-255 短边/画面宽*255
 *   12    校验和 = 偏移 4-11 共 8 字节累加 & 0xFF
 *   13-16 帧尾 FF FF EE EE
 *
 * 下行（主控 -> 相机）11 字节：
 *   0-3   CC CC DD DD
 *   4     CMD
 *   5     ARG
 *   6     校验和 = (CMD + ARG) & 0xFF
 *   7-10  FF FF EE EE
 *
 * 回执（相机 -> 主控）11 字节：
 *   4     CMD + 0x80
 *   5     状态 00 成功 / 01 失败
 *   6     校验和 = (回执命令字 + 状态) & 0xFF
 *
 * 本文件不依赖 HAL，可直接在 PC 上用 gcc 编译做单元测试。
 * 用法见 README.md 的“STM32 接入示例”。
 */

#ifndef MX_PROTOCOL_H
#define MX_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- 常量 */

#define MX_UP_FRAME_LEN     17U
#define MX_DOWN_FRAME_LEN   11U

/** 未标定时的几何中心（归一化值）。实际应用请使用标定得到的 CENTER_X / CENTER_Y。 */
#define MX_CENTER_N         128

/* 目标颜色（上行帧偏移 6 / SET_COLOR 的 ARG） */
#define MX_COLOR_RED        0x01U
#define MX_COLOR_GREEN      0x02U
#define MX_COLOR_BLUE       0x03U

/* 命令字（下行帧偏移 4） */
#define MX_CMD_SET_COLOR    0x01U
#define MX_CMD_RUN_STANDBY  0x02U
#define MX_CMD_SET_FPS      0x03U
#define MX_CMD_CALIBRATE    0x04U
#define MX_CMD_NEXT         0x05U   /* v3.0：主控"已执行完当前动作" */
#define MX_CMD_ABORT        0x06U   /* v3.0：主控放弃本次任务 */
#define MX_CMD_SAVE         0x10U
#define MX_CMD_PING         0x7FU

/* v3.0 上行 state 字段：相机当前处于哪个任务阶段 */
#define MX_STATE_SEARCH     0U
#define MX_STATE_BLOCK      1U
#define MX_STATE_GRAB       2U   /* 物块已就位，可以抓 */
#define MX_STATE_RING       3U
#define MX_STATE_DROP       4U   /* 环已就位，可以放 */
#define MX_STATE_DONE       5U

/* RUN / STANDBY 的 ARG */
#define MX_ARG_STANDBY      0x00U
#define MX_ARG_RUN          0x01U

/* 回执状态 */
#define MX_STATUS_OK        0x00U
#define MX_STATUS_FAIL      0x01U

/* 上行帧字段偏移（整帧） */
#define MX_OFF_SEQ          4U
#define MX_OFF_EXISTS       5U
#define MX_OFF_COLOR        6U
#define MX_OFF_STATE        7U
#define MX_OFF_XN           8U
#define MX_OFF_YN           9U
#define MX_OFF_WN           10U
#define MX_OFF_HN           11U
#define MX_OFF_CHECKSUM     12U

/* ---------------------------------------------------------------- 类型 */

/** 一帧解析结果 */
typedef struct {
    uint8_t seq;        /**< 帧序号，0-255 循环，用于检测丢帧 */
    uint8_t exists;     /**< 1 存在 / 0 不存在 */
    uint8_t color;      /**< MX_COLOR_xxx */
    uint8_t state;      /**< MX_STATE_xxx：相机当前的任务阶段（v3.0） */
    uint8_t x_n;        /**< 横向位置 0-255 */
    uint8_t y_n;        /**< 纵向位置 0-255 */
    uint8_t w_n;        /**< 长边归一化 0-255 */
    uint8_t h_n;        /**< 短边归一化 0-255 */
} mx_uplink_t;

/** 逐字节流式解析器（放在 UART 中断 / DMA 空闲回调里调用） */
typedef struct {
    uint8_t  buf[MX_UP_FRAME_LEN];
    uint8_t  idx;        /**< 内部：帧头匹配进度 / 已收集字节数 */
    uint8_t  state;      /**< 0-3 正在匹配帧头；>=4 正在收集帧体 */
    uint32_t ok_frames;  /**< 累计有效帧 */
    uint32_t bad_frames; /**< 累计帧尾或校验和非法的帧 */
} mx_parser_t;

/* ---------------------------------------------------------------- 接口 */

/** 初始化解析器（上电调用一次） */
void mx_parser_init(mx_parser_t *p);

/**
 * @brief 喂入一个字节
 * @param out 解析成功时写入结果，可为 NULL
 * @return true 表示这次喂入凑齐了一帧有效数据
 *
 * 线程/中断安全：本身不带锁，请在同一个上下文（UART 中断）里调用。
 */
bool mx_parser_feed(mx_parser_t *p, uint8_t byte, mx_uplink_t *out);

/** 计算偏移 4-11 共 8 字节的校验和 */
uint8_t mx_uplink_checksum(const uint8_t *payload8);

/** 组一帧下行指令（out 至少 11 字节） */
void mx_build_downlink(uint8_t cmd, uint8_t arg, uint8_t *out);

/** 组一帧回执（一般由相机侧使用；主控侧用来做单元测试或模拟相机） */
void mx_build_ack(uint8_t cmd, uint8_t status, uint8_t *out);

/** 解析一帧下行指令或回执，成功返回 true */
bool mx_parse_downlink(const uint8_t *frame, uint8_t *cmd, uint8_t *arg);

/** 相对标定中心的横向偏差：>0 目标偏右，<0 偏左 */
static inline int mx_dx(const mx_uplink_t *f, uint8_t center_x)
{
    return (int)f->x_n - (int)center_x;
}

/** 相对标定中心的纵向偏差：>0 目标偏下，<0 偏上 */
static inline int mx_dy(const mx_uplink_t *f, uint8_t center_y)
{
    return (int)f->y_n - (int)center_y;
}

/** 两帧之间丢了几帧（0 表示连续） */
static inline uint8_t mx_seq_gap(uint8_t prev, uint8_t cur)
{
    return (uint8_t)(cur - prev - 1U);
}

#ifdef __cplusplus
}
#endif

#endif /* MX_PROTOCOL_H */
