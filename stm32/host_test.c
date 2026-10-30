/**
 * @file host_test.c
 * @brief 在 PC 上验证 mx_protocol.c 是否符合 v4 协议文档（不依赖 STM32 / HAL）
 *
 * 编译运行（Windows / MinGW）：
 *     gcc -std=c99 -Wall -Wextra -O2 mx_protocol.c host_test.c -o host_test.exe
 *     .\host_test.exe
 *
 * 用例里的字节序列直接取自《相机_主控串口通信协议.docx》的示例，
 * 其中示例帧 aa aa bb bb ... 是最高优先级的黄金向量。
 */

#include "mx_protocol.h"

#include <stdio.h>
#include <string.h>

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (cond) {                                             \
            g_pass++;                                           \
        } else {                                                \
            g_fail++;                                           \
            printf("  [FAIL] %s:%d  ", __FILE__, __LINE__);     \
            printf(__VA_ARGS__);                                \
            printf("\n");                                       \
        }                                                       \
    } while (0)

/* 文档示例帧：存在、绿色、旋转 0x15、中心(100,200)、长50 宽30、校验 0x94 */
static const uint8_t DOC_FRAME[MX_UPLINK_FRAME_LEN] = {
    0xAA, 0xAA, 0xBB, 0xBB, 0x01, 0x02, 0x15,
    0x64, 0x00, 0xC8, 0x00, 0x32, 0x00, 0x1E, 0x00,
    0x94, 0xFF, 0xFF, 0xEE, 0xEE
};

/* ---------------------------------------------------------------- 工具 */

static void make_frame(uint8_t *out, uint8_t exists, uint8_t color,
                       uint8_t rotation, uint16_t x, uint16_t y,
                       uint16_t w, uint16_t h)
{
    mx_uplink_t f;

    memset(&f, 0, sizeof(f));
    f.exists   = exists;
    f.color    = color;
    f.rotation = rotation;
    f.x = x;
    f.y = y;
    f.w = w;
    f.h = h;
    CHECK(mx_build_uplink(&f, out) == MX_OK, "辅助函数组帧失败");
}

static void bytewise_feed(mx_uplink_parser_t *p, const uint8_t *data,
                          size_t len, int *frames, mx_uplink_t *last)
{
    for (size_t i = 0U; i < len; i++) {
        mx_uplink_t f;
        if (mx_uplink_parser_feed_byte(p, data[i], &f) == 1) {
            (*frames)++;
            if (last != NULL) {
                *last = f;
            }
        }
    }
}

/* 回调收集器 */
typedef struct {
    unsigned count;
    mx_uplink_t last;
} collect_t;

static void collect_cb(const mx_uplink_t *frame, void *user)
{
    collect_t *c = (collect_t *)user;

    c->count++;
    c->last = *frame;
}

/* ---------------------------------------------------------------- 用例 */

static void test_doc_golden(void)
{
    printf("用例1：文档示例帧逐字节一致（组帧 + 解帧）\n");

    mx_uplink_t f;
    uint8_t buf[MX_UPLINK_FRAME_LEN];

    memset(&f, 0, sizeof(f));
    f.exists   = 1U;
    f.color    = MX_COLOR_GREEN;
    f.rotation = 0x15U;
    f.x = 100U;
    f.y = 200U;
    f.w = 50U;
    f.h = 30U;

    CHECK(mx_build_uplink(&f, buf) == MX_OK, "组帧返回错误");

    /* 逐字节比对，失败时能直接指出是哪一位不对 */
    for (unsigned i = 0U; i < MX_UPLINK_FRAME_LEN; i++) {
        CHECK(buf[i] == DOC_FRAME[i],
              "第 %u 字节 = %02X，期望 %02X", i,
              (unsigned)buf[i], (unsigned)DOC_FRAME[i]);
    }

    /* 独立累加一遍，确认 0x94 确实等于偏移 4-14 的和 */
    {
        uint8_t sum = 0U;
        for (unsigned i = MX_OFF_EXISTS; i < MX_OFF_CHECKSUM; i++) {
            sum = (uint8_t)(sum + DOC_FRAME[i]);
        }
        CHECK(sum == 0x94U, "偏移 4-14 累加 = %02X，期望 94", (unsigned)sum);
        CHECK(mx_uplink_checksum(&DOC_FRAME[MX_OFF_EXISTS]) == 0x94U,
              "校验和助手 = %02X，期望 94",
              (unsigned)mx_uplink_checksum(&DOC_FRAME[MX_OFF_EXISTS]));
    }

    /* 解帧 */
    mx_uplink_t g;
    memset(&g, 0xA5, sizeof(g));
    CHECK(mx_parse_uplink(DOC_FRAME, &g) == MX_OK, "示例帧解析失败");
    CHECK(g.exists == 1U, "exists = %u", (unsigned)g.exists);
    CHECK(g.color == MX_COLOR_GREEN, "color = %u", (unsigned)g.color);
    CHECK(g.rotation == 0x15U, "rotation = %02X", (unsigned)g.rotation);
    CHECK(mx_uplink_angle(&g) == 0x15U, "angle = %u", (unsigned)mx_uplink_angle(&g));
    CHECK(!mx_uplink_is_ring(&g), "示例帧不该带色环标志");
    CHECK(g.x == 100U, "x = %u", (unsigned)g.x);
    CHECK(g.y == 200U, "y = %u", (unsigned)g.y);
    CHECK(g.w == 50U, "w = %u", (unsigned)g.w);
    CHECK(g.h == 30U, "h = %u", (unsigned)g.h);

    /* 只校验不取值也必须成功 */
    CHECK(mx_parse_uplink(DOC_FRAME, NULL) == MX_OK, "out 为 NULL 时解析失败");
    CHECK(strstr(MX_PROTOCOL_VERSION, "v4") != NULL, "版本串不是 v4");
}

static void test_roundtrip(void)
{
    printf("用例2：组帧/解帧往返（含 uint16 边界值）\n");

    static const uint16_t coords[][4] = {
        { 0U,     0U,     0U,     0U     },
        { 1U,     2U,     3U,     4U     },
        { 100U,   200U,   50U,    30U    },
        { 639U,   479U,   640U,   480U   },
        { 65535U, 65535U, 65535U, 65535U },
        { 0x1234U, 0x00FFU, 0x8000U, 0x0001U }
    };

    for (unsigned i = 0U; i < sizeof(coords) / sizeof(coords[0]); i++) {
        mx_uplink_t f;
        mx_uplink_t g;
        uint8_t buf[MX_UPLINK_FRAME_LEN];

        memset(&f, 0, sizeof(f));
        f.exists   = (uint8_t)(i & 1U);
        f.color    = (uint8_t)(i % 3U + 1U);
        f.rotation = (uint8_t)(i * 0x33U);
        f.x = coords[i][0];
        f.y = coords[i][1];
        f.w = coords[i][2];
        f.h = coords[i][3];

        CHECK(mx_build_uplink(&f, buf) == MX_OK, "组帧失败 i=%u", i);
        memset(&g, 0xA5, sizeof(g));
        CHECK(mx_parse_uplink(buf, &g) == MX_OK, "解帧失败 i=%u", i);
        CHECK(g.exists == f.exists && g.color == f.color &&
              g.rotation == f.rotation,
              "标量字段不一致 i=%u", i);
        CHECK(g.x == f.x && g.y == f.y && g.w == f.w && g.h == f.h,
              "坐标字段不一致 i=%u", i);
        CHECK(mx_uplink_angle(&g) == (uint8_t)(f.rotation & MX_ROTATION_ANGLE_MASK),
              "角度步进不一致 i=%u", i);
        CHECK(mx_uplink_is_ring(&g) ==
              ((f.rotation & MX_ROTATION_RING_BIT) != 0U),
              "色环标志不一致 i=%u", i);
    }

    /* 目标丢失：坐标必须原样是 0，不能被主控填成画面中心 */
    uint8_t buf[MX_UPLINK_FRAME_LEN];
    mx_uplink_t g;
    make_frame(buf, 0U, MX_COLOR_BLUE, 0U, 0U, 0U, 0U, 0U);
    CHECK(mx_parse_uplink(buf, &g) == MX_OK, "丢失目标帧解析失败");
    CHECK(g.exists == 0U, "exists = %u", (unsigned)g.exists);
    CHECK(g.x == 0U && g.y == 0U && g.w == 0U && g.h == 0U,
          "目标不存在时坐标必须为 0");
}

static void test_u16_le(void)
{
    printf("用例3：uint16 小端编解码（x=100 -> 64 00）\n");

    mx_uplink_t f;
    uint8_t buf[MX_UPLINK_FRAME_LEN];

    memset(&f, 0, sizeof(f));
    f.exists = 1U;
    f.color  = MX_COLOR_RED;
    f.x = 100U;
    f.y = 200U;
    f.w = 50U;
    f.h = 30U;
    CHECK(mx_build_uplink(&f, buf) == MX_OK, "组帧失败");
    CHECK(buf[MX_OFF_X] == 0x64U && buf[MX_OFF_X + 1U] == 0x00U,
          "x=100 应为 64 00，实际 %02X %02X",
          (unsigned)buf[MX_OFF_X], (unsigned)buf[MX_OFF_X + 1U]);
    CHECK(buf[MX_OFF_Y] == 0xC8U && buf[MX_OFF_Y + 1U] == 0x00U,
          "y=200 应为 C8 00");
    CHECK(buf[MX_OFF_W] == 0x32U && buf[MX_OFF_W + 1U] == 0x00U,
          "w=50 应为 32 00");
    CHECK(buf[MX_OFF_H] == 0x1EU && buf[MX_OFF_H + 1U] == 0x00U,
          "h=30 应为 1E 00");

    /* 高位不能受主机字节序影响 */
    f.x = 0x1234U;
    f.y = 0x00FFU;
    f.w = 0x8000U;
    f.h = 0xABCDU;
    CHECK(mx_build_uplink(&f, buf) == MX_OK, "组帧失败");
    CHECK(buf[MX_OFF_X] == 0x34U && buf[MX_OFF_X + 1U] == 0x12U,
          "x=0x1234 应为 34 12");
    CHECK(buf[MX_OFF_Y] == 0xFFU && buf[MX_OFF_Y + 1U] == 0x00U,
          "y=0x00FF 应为 FF 00");
    CHECK(buf[MX_OFF_W] == 0x00U && buf[MX_OFF_W + 1U] == 0x80U,
          "w=0x8000 应为 00 80");
    CHECK(buf[MX_OFF_H] == 0xCDU && buf[MX_OFF_H + 1U] == 0xABU,
          "h=0xABCD 应为 CD AB");

    /* 解析回来数值不变 */
    mx_uplink_t g;
    CHECK(mx_parse_uplink(buf, &g) == MX_OK, "解帧失败");
    CHECK(g.x == 0x1234U && g.y == 0x00FFU && g.w == 0x8000U && g.h == 0xABCDU,
          "解析结果 x=%u y=%u w=%u h=%u",
          (unsigned)g.x, (unsigned)g.y, (unsigned)g.w, (unsigned)g.h);
}

static void test_rotation_ring(void)
{
    printf("用例4：旋转角度与色环标志（bit7）\n");

    /* v4 映射：低 7 位 0..127 对应 0..360 度 */
    CHECK(mx_rotation_encode(0, false) == 0x00U, "0 度");
    CHECK(mx_rotation_encode(0, true) == 0x80U, "0 度 + 环");
    CHECK(mx_rotation_encode(360, false) == 0x7FU, "360 度应封顶 127");
    CHECK(mx_rotation_encode(720, false) == 0x7FU, "超过 360 度应封顶");
    CHECK(mx_rotation_encode(-10, true) == 0x80U, "负角度按 0 处理");
    CHECK(mx_rotation_encode(30, false) == 0x0BU, "30 度 -> 11 步");
    CHECK(mx_rotation_encode(90, false) == 0x20U, "90 度 -> 32 步");
    CHECK(mx_rotation_encode(180, true) == 0xC0U, "180 度 -> 64 步 + 环");

    uint8_t angle = 0U;
    bool ring = false;
    mx_rotation_decode(0xA5U, &angle, &ring);
    CHECK(angle == 0x25U, "decode angle = %02X", (unsigned)angle);
    CHECK(ring, "decode 应带环标志");
    mx_rotation_decode(0, NULL, NULL); /* 出参可为 NULL，不应崩溃 */
    CHECK(1, "decode 出参为 NULL 时不应崩溃");

    mx_uplink_t f;
    mx_uplink_t g;
    uint8_t buf[MX_UPLINK_FRAME_LEN];

    memset(&f, 0, sizeof(f));
    f.exists   = 1U;
    f.color    = MX_COLOR_GREEN;
    f.rotation = mx_rotation_encode(90, true);
    f.x = 320U;
    f.y = 240U;
    f.w = 64U;
    f.h = 32U;

    CHECK(mx_build_uplink(&f, buf) == MX_OK, "组帧失败");
    CHECK((buf[MX_OFF_ROTATION] & MX_ROTATION_RING_BIT) != 0U,
          "is_ring=1 时第 6 字节 bit7 必须为 1");
    CHECK((buf[MX_OFF_ROTATION] & MX_ROTATION_ANGLE_MASK) == 0x20U,
          "色环不能吃掉角度位");

    CHECK(mx_parse_uplink(buf, &g) == MX_OK, "解帧失败");
    CHECK(mx_uplink_is_ring(&g), "解析回来 is_ring 应为真");
    CHECK(mx_uplink_angle(&g) == 0x20U, "解析回来角度应为 32 步");
    CHECK(g.rotation == buf[MX_OFF_ROTATION], "rotation 原始字节应保留");

    /* 同一角度不带环：bit7 必须为 0，低 7 位不变 */
    f.rotation = mx_rotation_encode(90, false);
    CHECK(mx_build_uplink(&f, buf) == MX_OK, "组帧失败");
    CHECK((buf[MX_OFF_ROTATION] & MX_ROTATION_RING_BIT) == 0U,
          "is_ring=0 时 bit7 必须为 0");
    CHECK((buf[MX_OFF_ROTATION] & MX_ROTATION_ANGLE_MASK) == 0x20U,
          "非环帧角度应为 32 步");
}

static void test_checksum_reject(void)
{
    printf("用例5：校验和/帧头/帧尾错误必须被拒绝\n");

    uint8_t bad[MX_UPLINK_FRAME_LEN];
    mx_uplink_t g;

    /* 独立验证 11 字节全 0xFF 的溢出行为：11*255=2805=0x0AF5 -> 0xF5 */
    uint8_t all_ff[MX_UPLINK_SUM_LEN];
    memset(all_ff, 0xFF, sizeof(all_ff));
    CHECK(mx_uplink_checksum(all_ff) == 0xF5U,
          "溢出校验和 = %02X，期望 F5",
          (unsigned)mx_uplink_checksum(all_ff));

    uint8_t zeros[MX_UPLINK_SUM_LEN];
    memset(zeros, 0, sizeof(zeros));
    CHECK(mx_uplink_checksum(zeros) == 0x00U, "全 0 校验和应为 0");
    CHECK(mx_uplink_checksum(NULL) == 0x00U, "NULL 应安全返回 0");

    /* 校验和错 */
    memcpy(bad, DOC_FRAME, sizeof(bad));
    bad[MX_OFF_CHECKSUM] ^= 0xFFU;
    CHECK(mx_parse_uplink(bad, &g) == MX_ERR_CHECKSUM,
          "校验和错误应返回 MX_ERR_CHECKSUM");

    /* 载荷被改但校验和没跟着改 */
    memcpy(bad, DOC_FRAME, sizeof(bad));
    bad[MX_OFF_X] ^= 0x01U;
    CHECK(mx_parse_uplink(bad, &g) == MX_ERR_CHECKSUM,
          "载荷篡改应被校验和拦下");

    /* 帧头错 */
    memcpy(bad, DOC_FRAME, sizeof(bad));
    bad[2] = 0xBCU;
    CHECK(mx_parse_uplink(bad, &g) == MX_ERR_HEADER,
          "帧头错误应返回 MX_ERR_HEADER");

    /* 帧尾错 */
    memcpy(bad, DOC_FRAME, sizeof(bad));
    bad[MX_OFF_TAIL + 2U] = 0xEFU;
    CHECK(mx_parse_uplink(bad, &g) == MX_ERR_TAIL,
          "帧尾错误应返回 MX_ERR_TAIL");

    /* 空指针 */
    CHECK(mx_parse_uplink(NULL, &g) == MX_ERR_ARG, "NULL 帧应返回 MX_ERR_ARG");
}

static void test_stream_bytewise(void)
{
    printf("用例6：流式解析——逐字节喂入\n");

    mx_uplink_parser_t p;
    mx_uplink_t f;
    int frames = 0;

    mx_uplink_parser_init(&p);
    memset(&f, 0, sizeof(f));
    bytewise_feed(&p, DOC_FRAME, sizeof(DOC_FRAME), &frames, &f);

    CHECK(frames == 1, "逐字节喂入收到 %d 帧，期望 1", frames);
    CHECK(p.ok_frames == 1U, "ok_frames = %u", (unsigned)p.ok_frames);
    CHECK(p.bad_frames == 0U, "bad_frames = %u", (unsigned)p.bad_frames);
    CHECK(p.len == 0U, "解出帧后窗口应清空，len=%u", (unsigned)p.len);
    CHECK(f.x == 100U && f.y == 200U, "逐字节解析坐标错误");
    CHECK(f.color == MX_COLOR_GREEN, "逐字节解析颜色错误");

    /* 差最后 1 字节时不能提前出帧 */
    mx_uplink_parser_init(&p);
    frames = 0;
    bytewise_feed(&p, DOC_FRAME, sizeof(DOC_FRAME) - 1U, &frames, &f);
    CHECK(frames == 0, "不足 20 字节不该出帧，实际 %d", frames);
    CHECK(mx_uplink_parser_feed_byte(&p, DOC_FRAME[19], &f) == 1,
          "补上最后 1 字节应出帧");

    /* 参数保护 */
    CHECK(mx_uplink_parser_feed_byte(NULL, 0U, &f) == MX_ERR_ARG,
          "NULL 解析器应返回 MX_ERR_ARG");
    CHECK(mx_uplink_parser_feed(NULL, NULL, 0U, NULL, NULL) == MX_ERR_ARG,
          "NULL 解析器应返回 MX_ERR_ARG");
    mx_uplink_parser_init(NULL); /* 不应崩溃 */
    CHECK(1, "init(NULL) 不应崩溃");
}

static void test_stream_sticky(void)
{
    printf("用例7：流式解析——粘包 / 分批喂入 / 回调\n");

    uint8_t second[MX_UPLINK_FRAME_LEN];
    uint8_t both[2U * MX_UPLINK_FRAME_LEN];

    /* 第二帧：目标丢失、带色环、坐标取 uint16 边界值 */
    make_frame(second, 0U, MX_COLOR_RED, 0x80U, 511U, 256U, 640U, 480U);
    memcpy(both, DOC_FRAME, MX_UPLINK_FRAME_LEN);
    memcpy(&both[MX_UPLINK_FRAME_LEN], second, MX_UPLINK_FRAME_LEN);

    /* 一次喂两帧，用回调收 */
    mx_uplink_parser_t p;
    mx_uplink_parser_init(&p);
    collect_t c;
    memset(&c, 0, sizeof(c));
    int n = mx_uplink_parser_feed(&p, both, sizeof(both), collect_cb, &c);
    CHECK(n == 2, "粘包一次喂入收到 %d 帧，期望 2", n);
    CHECK(c.count == 2U, "回调调用 %u 次，期望 2", c.count);
    CHECK(c.last.exists == 0U, "最后一帧 exists = %u", (unsigned)c.last.exists);
    CHECK(c.last.x == 511U && c.last.y == 256U, "最后一帧坐标错误");
    CHECK(mx_uplink_is_ring(&c.last), "最后一帧应带色环标志");
    CHECK(p.ok_frames == 2U && p.bad_frames == 0U,
          "统计 ok=%u bad=%u", (unsigned)p.ok_frames, (unsigned)p.bad_frames);

    /* 同一份数据分两次喂（模拟两包 DMA） */
    mx_uplink_parser_init(&p);
    n = mx_uplink_parser_feed(&p, both, 10U, NULL, NULL);
    CHECK(n == 0, "不足一帧时帧数应为 0，实际 %d", n);
    n = mx_uplink_parser_feed(&p, &both[10], sizeof(both) - 10U, NULL, NULL);
    CHECK(n == 2, "分两次喂入共收到 %d 帧，期望 2", n);

    /* 逐字节同样能收到两帧 */
    mx_uplink_parser_init(&p);
    int frames = 0;
    mx_uplink_t last;
    memset(&last, 0, sizeof(last));
    bytewise_feed(&p, both, sizeof(both), &frames, &last);
    CHECK(frames == 2, "逐字节收到 %d 帧，期望 2", frames);
    CHECK(last.x == 511U && mx_uplink_is_ring(&last), "逐字节最后一帧错误");

    /* len=0、data=NULL 是合法调用 */
    mx_uplink_parser_init(&p);
    CHECK(mx_uplink_parser_feed(&p, NULL, 0U, NULL, NULL) == 0,
          "空数据应返回 0");
}

static void test_stream_noise(void)
{
    printf("用例8：流式解析——前面有噪声和假帧头\n");

    static const uint8_t noise[] = {
        0x00U, 0x13U, 0xAAU, 0xAAU, 0xBBU, 0x00U, 0x55U, 0xEEU
    };
    uint8_t stream[sizeof(noise) + MX_UPLINK_FRAME_LEN];
    memcpy(stream, noise, sizeof(noise));
    memcpy(&stream[sizeof(noise)], DOC_FRAME, MX_UPLINK_FRAME_LEN);

    mx_uplink_parser_t p;
    mx_uplink_parser_init(&p);
    int frames = 0;
    mx_uplink_t f;
    memset(&f, 0, sizeof(f));
    bytewise_feed(&p, stream, sizeof(stream), &frames, &f);

    CHECK(frames == 1, "噪声后收到 %d 帧，期望 1", frames);
    CHECK(f.x == 100U && f.y == 200U, "噪声后解析结果错误");
    CHECK(f.color == MX_COLOR_GREEN, "噪声后颜色错误");
    CHECK(p.bad_frames == 0U, "噪声不该被算成坏帧，bad=%u",
          (unsigned)p.bad_frames);

    /* 重叠帧头 AA AA BB + 真帧头 AA AA BB BB：必须逐字节滑到正确起点，
       不能因为前 3 个字节对上了就死等，也不能一次跳过 4 字节。 */
    uint8_t overlap[3U + MX_UPLINK_FRAME_LEN];
    overlap[0] = 0xAAU;
    overlap[1] = 0xAAU;
    overlap[2] = 0xBBU;
    memcpy(&overlap[3], DOC_FRAME, MX_UPLINK_FRAME_LEN);

    mx_uplink_parser_init(&p);
    frames = 0;
    memset(&f, 0, sizeof(f));
    bytewise_feed(&p, overlap, sizeof(overlap), &frames, &f);
    CHECK(frames == 1, "重叠帧头后收到 %d 帧，期望 1", frames);
    CHECK(f.x == 100U && f.rotation == 0x15U, "重叠帧头后解析结果错误");
}

static void test_stream_resync(void)
{
    printf("用例9：流式解析——坏帧之后重新同步\n");

    uint8_t bad[MX_UPLINK_FRAME_LEN];
    uint8_t stream[2U * MX_UPLINK_FRAME_LEN];

    /* 坏校验和帧 + 紧跟着的好帧 */
    memcpy(bad, DOC_FRAME, sizeof(bad));
    bad[MX_OFF_CHECKSUM] ^= 0xFFU;
    memcpy(stream, bad, MX_UPLINK_FRAME_LEN);
    memcpy(&stream[MX_UPLINK_FRAME_LEN], DOC_FRAME, MX_UPLINK_FRAME_LEN);

    mx_uplink_parser_t p;
    mx_uplink_parser_init(&p);
    int frames = 0;
    mx_uplink_t f;
    memset(&f, 0, sizeof(f));
    bytewise_feed(&p, stream, sizeof(stream), &frames, &f);

    CHECK(frames == 1, "坏帧后收到 %d 帧，期望 1", frames);
    CHECK(f.x == 100U && f.y == 200U, "重新同步后收到的应是好帧");
    CHECK(p.ok_frames == 1U, "ok_frames = %u", (unsigned)p.ok_frames);
    CHECK(p.bad_frames >= 1U, "应至少记录 1 个坏帧，bad=%u",
          (unsigned)p.bad_frames);

    /* 坏帧尾 + 好帧，同样要能爬回来 */
    memcpy(bad, DOC_FRAME, sizeof(bad));
    bad[MX_OFF_TAIL] = 0x00U;
    memcpy(stream, bad, MX_UPLINK_FRAME_LEN);
    memcpy(&stream[MX_UPLINK_FRAME_LEN], DOC_FRAME, MX_UPLINK_FRAME_LEN);

    mx_uplink_parser_init(&p);
    frames = 0;
    bytewise_feed(&p, stream, sizeof(stream), &frames, &f);
    CHECK(frames == 1, "坏帧尾后收到 %d 帧，期望 1", frames);
    CHECK(f.x == 100U, "坏帧尾后重新同步失败");
    CHECK(p.bad_frames >= 1U, "坏帧尾应计入 bad_frames");
}

static void test_downlink(void)
{
    printf("用例10：下行 4 字节指令组帧与解析\n");

    static const uint8_t cmds[] = {
        MX_CMD_RED, MX_CMD_GREEN, MX_CMD_BLUE, MX_CMD_PING
    };
    static const uint8_t expect[][MX_DOWNLINK_FRAME_LEN] = {
        { 0xAAU, 0xBBU, 0x01U, 0xFFU },
        { 0xAAU, 0xBBU, 0x02U, 0xFFU },
        { 0xAAU, 0xBBU, 0x03U, 0xFFU },
        { 0xAAU, 0xBBU, 0x7FU, 0xFFU }
    };

    uint8_t buf[MX_DOWNLINK_FRAME_LEN];
    for (unsigned i = 0U; i < sizeof(cmds); i++) {
        memset(buf, 0x5A, sizeof(buf));
        CHECK(mx_build_downlink(cmds[i], buf) == MX_OK,
              "组下行帧失败 cmd=%02X", (unsigned)cmds[i]);
        CHECK(memcmp(buf, expect[i], MX_DOWNLINK_FRAME_LEN) == 0,
              "cmd=%02X 组帧字节不符", (unsigned)cmds[i]);

        uint8_t cmd = 0U;
        CHECK(mx_parse_downlink(buf, &cmd) == MX_OK, "解析下行失败");
        CHECK(cmd == cmds[i], "解析 cmd=%02X，期望 %02X",
              (unsigned)cmd, (unsigned)cmds[i]);
    }

    /* 非法指令：一个字节都不该写 */
    static const uint8_t bad_cmds[] = { 0x00U, 0x04U, 0x05U, 0x10U, 0x80U, 0xFFU };
    for (unsigned i = 0U; i < sizeof(bad_cmds); i++) {
        memset(buf, 0x5A, sizeof(buf));
        CHECK(mx_build_downlink(bad_cmds[i], buf) == MX_ERR_CMD,
              "非法指令 %02X 应被拒绝", (unsigned)bad_cmds[i]);
        CHECK(buf[0] == 0x5AU, "非法指令不该写输出缓冲");
    }

    /* 帧头 / 帧尾 / 指令字错误 */
    uint8_t frame[MX_DOWNLINK_FRAME_LEN] = { 0xAAU, 0xBBU, 0x01U, 0xFFU };
    CHECK(mx_parse_downlink(frame, NULL) == MX_OK, "cmd 出参可为 NULL");

    frame[0] = 0xABU;
    CHECK(mx_parse_downlink(frame, NULL) == MX_ERR_HEADER, "下行帧头错误未被拒绝");

    frame[0] = 0xAAU;
    frame[3] = 0xFEU;
    CHECK(mx_parse_downlink(frame, NULL) == MX_ERR_TAIL, "下行帧尾错误未被拒绝");

    frame[3] = 0xFFU;
    frame[2] = 0x04U;
    CHECK(mx_parse_downlink(frame, NULL) == MX_ERR_CMD, "非法下行指令未被拒绝");

    CHECK(mx_parse_downlink(NULL, NULL) == MX_ERR_ARG, "NULL 下行帧应报参数错");
    CHECK(mx_build_downlink(MX_CMD_RED, NULL) == MX_ERR_ARG,
          "NULL 输出缓冲应报参数错");
}

static void test_constants(void)
{
    printf("用例11：常量与版本串\n");

    CHECK(MX_UPLINK_FRAME_LEN == 20U, "上行帧长 = %u", (unsigned)MX_UPLINK_FRAME_LEN);
    CHECK(MX_DOWNLINK_FRAME_LEN == 4U, "下行帧长 = %u",
          (unsigned)MX_DOWNLINK_FRAME_LEN);
    CHECK(MX_UPLINK_SUM_LEN == 11U, "校验长度 = %u", (unsigned)MX_UPLINK_SUM_LEN);
    CHECK(MX_UPLINK_SUM_LEN == MX_OFF_CHECKSUM - MX_OFF_EXISTS,
          "校验覆盖范围与偏移不自洽");
    CHECK(MX_OFF_X == 7U && MX_OFF_Y == 9U && MX_OFF_W == 11U &&
          MX_OFF_H == 13U && MX_OFF_CHECKSUM == 15U,
          "字段偏移与协议不符");
    CHECK(MX_REF_WIDTH == 640U && MX_REF_HEIGHT == 480U,
          "坐标参考系必须是 640x480");
    CHECK(mx_downlink_cmd_valid(MX_CMD_RED) &&
          mx_downlink_cmd_valid(MX_CMD_GREEN) &&
          mx_downlink_cmd_valid(MX_CMD_BLUE) &&
          mx_downlink_cmd_valid(MX_CMD_PING),
          "合法下行指令集合错误");
    CHECK(!mx_downlink_cmd_valid(0x00U) && !mx_downlink_cmd_valid(0x04U) &&
          !mx_downlink_cmd_valid(0x80U),
          "非法指令被当成了合法指令");
    CHECK(strstr(MX_PROTOCOL_VERSION, "v4") != NULL,
          "版本串里没有 v4：%s", MX_PROTOCOL_VERSION);
    CHECK(MX_PROTOCOL_VERSION_MAJOR == 4, "版本号不是 4");
}

int main(void)
{
    printf("=== mx_protocol v4 主机端测试 ===\n");
    printf("协议版本：%s\n\n", MX_PROTOCOL_VERSION);

    test_doc_golden();
    test_roundtrip();
    test_u16_le();
    test_rotation_ring();
    test_checksum_reject();
    test_stream_bytewise();
    test_stream_sticky();
    test_stream_noise();
    test_stream_resync();
    test_downlink();
    test_constants();

    printf("\n=== 结果：通过 %d 项，失败 %d 项 ===\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
