/**
 * @file    mx_protocol.h
 * @brief   相机(MaixCAM) <-> 主控(STM32H723) 串口协议 v4.0 —— 纯 C99 实现
 *
 * 唯一基准：
 *   - 《相机_主控串口通信协议.docx》原文
 *   - pc/protocol.py（v4 参考实现，行为必须一致）
 *
 * 上行（相机 -> 主控）20 字节定长：
 *   0-3    帧头 AA AA BB BB
 *   4      目标是否存在  0x01 存在 / 0x00 不存在
 *   5      目标颜色      0x01 红 / 0x02 绿 / 0x03 蓝
 *   6      旋转字节      bit7 = 是否色环，低 7 位 = 角度(0..127 对应 0..360 度)
 *   7-8    目标中心 x    uint16 小端，像素，参考系固定 640x480
 *   9-10   目标中心 y    uint16 小端，像素
 *   11-12  目标长   w    uint16 小端，像素
 *   13-14  目标宽   h    uint16 小端，像素
 *   15     校验和        偏移 4-14 共 11 字节累加 & 0xFF
 *   16-19  帧尾 FF FF EE EE
 *
 *   注意：x/y 是**目标中心**，不是左上角。相机若在低分辨率下识别，应在
 *   发送前换算回 640x480 参考系，主控端拿到的坐标含义才恒定。
 *
 * 下行（主控 -> 相机）4 字节定长，无校验和、无参数：
 *   0-1    帧头 AA BB
 *   2      指令 0x01/0x02/0x03 = 识别红/绿/蓝，0x7F = PING（调试用）
 *   3      帧尾 FF
 *
 *   文档原文：“一旦接收到一帧指令之后就按照这个指令一直识别下去。”
 *   即相机记住当前颜色持续发送，直到收到下一条指令。
 *
 * v4 协议**没有帧序号**：丢帧只能靠接收速率/时间戳估算，无法用序号推算。
 *
 * 本文件不依赖 HAL，只用 <stdint.h>/<stdbool.h>/<stddef.h>，
 * 可直接在 PC 上用 gcc 编译做单元测试（见 host_test.c）。
 *
 * 典型用法（主控侧解析相机上行帧）：
 *   static mx_uplink_parser_t s_parser;      // 全局，UART 中断里喂
 *   mx_uplink_parser_init(&s_parser);
 *   void on_uart_rx(uint8_t byte)
 *   {
 *       mx_uplink_t f;
 *       if (mx_uplink_parser_feed_byte(&s_parser, byte, &f) == 1) {
 *           // f.exists / f.color / f.x / f.y ... 在中断里只做拷贝，重活丢主循环
 *       }
 *   }
 */

#ifndef MX_PROTOCOL_H
#define MX_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- 版本 */

#define MX_PROTOCOL_VERSION       "MaixCAM<->STM32 UART v4.0"
#define MX_PROTOCOL_VERSION_MAJOR 4

/* ---------------------------------------------------------------- 帧长 */

#define MX_UPLINK_FRAME_LEN    20U  /**< 上行整帧长度 */
#define MX_UPLINK_HEADER_LEN   4U   /**< 上行帧头长度 */
#define MX_UPLINK_TAIL_LEN     4U   /**< 上行帧尾长度 */
#define MX_UPLINK_SUM_LEN      11U  /**< 参与校验和的字节数（偏移 4-14） */

#define MX_DOWNLINK_FRAME_LEN  4U   /**< 下行整帧长度 */

/* ---------------------------------------------------------------- 字节常量 */

/* 帧头 / 帧尾。写成独立字节宏，方便逐字节比对和自查表 */
#define MX_UPLINK_HDR0     0xAAU
#define MX_UPLINK_HDR1     0xAAU
#define MX_UPLINK_HDR2     0xBBU
#define MX_UPLINK_HDR3     0xBBU
#define MX_UPLINK_TAIL0    0xFFU
#define MX_UPLINK_TAIL1    0xFFU
#define MX_UPLINK_TAIL2    0xEEU
#define MX_UPLINK_TAIL3    0xEEU

#define MX_DOWNLINK_HDR0   0xAAU
#define MX_DOWNLINK_HDR1   0xBBU
#define MX_DOWNLINK_TAIL   0xFFU

/* 上行帧字段偏移（整帧内的偏移） */
#define MX_OFF_EXISTS      4U   /**< 目标是否存在 */
#define MX_OFF_COLOR       5U   /**< 目标颜色 */
#define MX_OFF_ROTATION    6U   /**< 旋转/色环字节 */
#define MX_OFF_X           7U   /**< 中心 x 低字节 */
#define MX_OFF_Y           9U   /**< 中心 y 低字节 */
#define MX_OFF_W           11U  /**< 目标长 低字节 */
#define MX_OFF_H           13U  /**< 目标宽 低字节 */
#define MX_OFF_CHECKSUM    15U  /**< 校验和 */
#define MX_OFF_TAIL        16U  /**< 帧尾 */

/* 颜色字段（上行偏移 5） */
#define MX_COLOR_RED       0x01U
#define MX_COLOR_GREEN     0x02U
#define MX_COLOR_BLUE      0x03U

/* 下行指令字。1/2/3 与颜色号同值，另有调试用的 PING */
#define MX_CMD_RED         0x01U
#define MX_CMD_GREEN       0x02U
#define MX_CMD_BLUE        0x03U
#define MX_CMD_PING        0x7FU

/* 第 6 字节：bit7 色环标志，低 7 位角度步进 */
#define MX_ROTATION_RING_BIT     0x80U
#define MX_ROTATION_ANGLE_MASK   0x7FU
#define MX_ROTATION_ANGLE_STEPS  127U

/* 坐标参考系：协议固定按 640x480 换算，主控不用关心相机内部真实分辨率 */
#define MX_REF_WIDTH       640U
#define MX_REF_HEIGHT      480U

/* ---------------------------------------------------------------- 返回码 */

/* 统一约定：0 = 成功，负数 = 具体错误。方便在 UART 调试口直接打印 */
#define MX_OK              0
#define MX_ERR_ARG        (-1)  /**< 空指针等参数错误 */
#define MX_ERR_HEADER     (-2)  /**< 帧头不匹配 */
#define MX_ERR_TAIL       (-3)  /**< 帧尾不匹配 */
#define MX_ERR_CHECKSUM   (-4)  /**< 校验和错误 */
#define MX_ERR_CMD        (-5)  /**< 未知下行指令 */

/* ---------------------------------------------------------------- 类型 */

/**
 * @brief 一帧上行数据。
 *
 * rotation 保留原始第 6 字节，方便日志/对表；角度和色环标志用下面的
 * mx_uplink_angle() / mx_uplink_is_ring() 拆，避免结构体内存两份冗余状态。
 */
typedef struct {
    uint8_t  exists;    /**< 1 存在 / 0 不存在 */
    uint8_t  color;     /**< MX_COLOR_xxx */
    uint8_t  rotation;  /**< 原始第 6 字节：bit7 色环，低 7 位角度 */
    uint16_t x;         /**< 目标中心 x，像素（640x480 参考系） */
    uint16_t y;         /**< 目标中心 y，像素 */
    uint16_t w;         /**< 目标长，像素 */
    uint16_t h;         /**< 目标宽，像素 */
} mx_uplink_t;

/** 解析出一帧时的回调；user 原样透传，可为 NULL */
typedef void (*mx_uplink_cb_t)(const mx_uplink_t *frame, void *user);

/**
 * @brief 逐字节流式解析器。
 *
 * 放在 UART 接收中断 / DMA 空闲回调里喂字节即可，内部自带 20 字节窗口，
 * 会自动处理拆包、粘包、噪声和坏帧后的重新同步，不需要调用者管缓冲。
 * 结构体本身不带锁：请固定在同一个上下文（中断或主循环）里调用。
 */
typedef struct {
    uint8_t  buf[MX_UPLINK_FRAME_LEN]; /**< 内部：以候选帧头开头的窗口 */
    uint8_t  len;                      /**< 内部：窗口内已有字节数 0..19 */
    uint32_t ok_frames;                /**< 累计有效帧数，便于统计帧率 */
    uint32_t bad_frames;               /**< 累计帧头对上但校验失败的帧数 */
} mx_uplink_parser_t;

/* ---------------------------------------------------------------- 角度/色环 */

/** 取角度步进（0..127），0 对应 0 度，127 对应 360 度 */
static inline uint8_t mx_uplink_angle(const mx_uplink_t *f)
{
    return (uint8_t)(f->rotation & MX_ROTATION_ANGLE_MASK);
}

/** 是否色环（第 6 字节 bit7） */
static inline bool mx_uplink_is_ring(const mx_uplink_t *f)
{
    return (f->rotation & MX_ROTATION_RING_BIT) != 0U;
}

/**
 * @brief 把“角度(度) + 是否色环”压成第 6 字节。
 * @param angle_deg 0..360，负数按 0，超过 360 按 360
 * @param is_ring   true 时置 bit7
 * @return 可直接写进 mx_uplink_t.rotation 的字节
 */
uint8_t mx_rotation_encode(int angle_deg, bool is_ring);

/**
 * @brief 拆开第 6 字节。
 * @param rotation    原始字节
 * @param angle_steps 输出角度步进 0..127，可为 NULL
 * @param is_ring     输出是否色环，可为 NULL
 */
void mx_rotation_decode(uint8_t rotation, uint8_t *angle_steps, bool *is_ring);

/* ---------------------------------------------------------------- 校验和 */

/**
 * @brief 计算上行校验和：偏移 4-14 共 11 字节累加后取低 8 位。
 * @param payload 指向整帧偏移 4（MX_OFF_EXISTS）的 11 字节，可为整帧+4
 * @return 校验和；payload 为 NULL 时返回 0
 */
uint8_t mx_uplink_checksum(const uint8_t *payload);

/* ---------------------------------------------------------------- 组帧 */

/**
 * @brief 组一帧 20 字节上行数据（测试、自检、模拟相机用）。
 * @param f   帧内容；rotation 请用 mx_rotation_encode() 生成
 * @param out 输出缓冲，至少 MX_UPLINK_FRAME_LEN 字节
 * @return MX_OK / MX_ERR_ARG
 */
int mx_build_uplink(const mx_uplink_t *f, uint8_t *out);

/**
 * @brief 组一帧 4 字节下行指令：AA BB cmd FF。
 * @param cmd 仅接受 MX_CMD_RED / GREEN / BLUE / PING
 * @param out 输出缓冲，至少 MX_DOWNLINK_FRAME_LEN 字节
 * @return MX_OK / MX_ERR_ARG / MX_ERR_CMD（非法指令时不写 out）
 */
int mx_build_downlink(uint8_t cmd, uint8_t *out);

/* ---------------------------------------------------------------- 解帧 */

/**
 * @brief 解析一帧完整的上行数据。
 * @param frame 指向至少 MX_UPLINK_FRAME_LEN 字节的完整帧
 * @param out   解析结果，可为 NULL（只校验不取值）
 * @return MX_OK / MX_ERR_ARG / MX_ERR_HEADER / MX_ERR_TAIL / MX_ERR_CHECKSUM
 *
 * 只校验帧头、帧尾、校验和；颜色/存在字段不做合法性判断，未知颜色也照常
 * 上报，避免相机以后扩色号时整帧被主控丢掉。
 */
int mx_parse_uplink(const uint8_t *frame, mx_uplink_t *out);

/**
 * @brief 解析一帧完整的下行指令。
 * @param frame 指向至少 MX_DOWNLINK_FRAME_LEN 字节的完整帧
 * @param cmd   输出指令字，可为 NULL
 * @return MX_OK / MX_ERR_ARG / MX_ERR_HEADER / MX_ERR_TAIL / MX_ERR_CMD
 */
int mx_parse_downlink(const uint8_t *frame, uint8_t *cmd);

/** 指令字是否属于 v4 合法集合（1/2/3/0x7F） */
bool mx_downlink_cmd_valid(uint8_t cmd);

/* ---------------------------------------------------------------- 流式解析 */

/** 初始化解析器（上电或出错恢复时调用一次） */
void mx_uplink_parser_init(mx_uplink_parser_t *p);

/**
 * @brief 喂入一个字节。
 * @param p   解析器
 * @param byte 收到的字节
 * @param out  凑齐一帧时写入结果，可为 NULL
 * @return 1 = 解出一帧有效数据；0 = 还需继续喂；MX_ERR_ARG = p 为空
 */
int mx_uplink_parser_feed_byte(mx_uplink_parser_t *p, uint8_t byte,
                               mx_uplink_t *out);

/**
 * @brief 喂入一段数据（DMA 空闲 / 环形缓冲取出一批时用）。
 * @param p    解析器
 * @param data 数据首地址；len 为 0 时可为 NULL
 * @param len  字节数
 * @param cb   每解出一帧调用一次；可为 NULL（只计数并更新统计）
 * @param user 原样传给 cb
 * @return 解出的帧数（>=0）；参数非法时返回 MX_ERR_ARG
 */
int mx_uplink_parser_feed(mx_uplink_parser_t *p, const uint8_t *data, size_t len,
                          mx_uplink_cb_t cb, void *user);

#ifdef __cplusplus
}
#endif

#endif /* MX_PROTOCOL_H */
