/**
 * @file host_test.c
 * @brief 在 PC 上验证 mx_protocol.c 的实现是否符合协议文档（不依赖 STM32 / HAL）
 *
 * 编译运行（Windows / MinGW）：
 *     gcc -std=c99 -Wall -Wextra -o host_test.exe host_test.c mx_protocol.c
 *     .\host_test.exe
 *
 * 测试用例里的字节序列直接取自《相机_主控串口通信协议_定稿.md》的示例。
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

/* 协议文档里的示例帧 */
static const uint8_t EX1[MX_UP_FRAME_LEN] = {
    0xAA, 0xAA, 0xBB, 0xBB, 0x2A, 0x01, 0x02, 0x00, 0x64,
    0x80, 0x3C, 0x28, 0x75, 0xFF, 0xFF, 0xEE, 0xEE
};
static const uint8_t EX2[MX_UP_FRAME_LEN] = {
    0xAA, 0xAA, 0xBB, 0xBB, 0x2B, 0x00, 0x02, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x2D, 0xFF, 0xFF, 0xEE, 0xEE
};

/* ---------------------------------------------------------------- 用例 */

static void test_doc_example_1(void)
{
    printf("用例1：文档示例一（绿色物块，偏左）整帧解析\n");
    mx_parser_t p;
    mx_parser_init(&p);
    mx_uplink_t f;
    memset(&f, 0, sizeof(f));
    bool got = false;
    for (unsigned i = 0; i < MX_UP_FRAME_LEN; i++) {
        if (mx_parser_feed(&p, EX1[i], &f)) {
            got = true;
        }
    }
    CHECK(got, "没有解析出帧");
    CHECK(f.seq == 0x2A, "seq=%u 期望 42", f.seq);
    CHECK(f.exists == 1, "exists=%u", f.exists);
    CHECK(f.color == MX_COLOR_GREEN, "color=%u", f.color);
    CHECK(f.state == 0, "state=%u", f.state);
    CHECK(f.x_n == 100, "x_n=%u", f.x_n);
    CHECK(f.y_n == 128, "y_n=%u", f.y_n);
    CHECK(f.w_n == 60, "w_n=%u", f.w_n);
    CHECK(f.h_n == 40, "h_n=%u", f.h_n);
    CHECK(p.ok_frames == 1, "ok_frames=%u", p.ok_frames);
    CHECK(p.bad_frames == 0, "bad_frames=%u", p.bad_frames);
    CHECK(mx_dx(&f, 128) == -28, "dx=%d 期望 -28", mx_dx(&f, 128));
    CHECK(mx_dy(&f, 128) == 0, "dy=%d 期望 0", mx_dy(&f, 128));
}

static void test_doc_example_2(void)
{
    printf("用例2：文档示例二（目标丢失，坐标必须全 0 而不是 128）\n");
    mx_parser_t p;
    mx_parser_init(&p);
    mx_uplink_t f;
    memset(&f, 0xEE, sizeof(f));
    bool got = false;
    for (unsigned i = 0; i < MX_UP_FRAME_LEN; i++) {
        if (mx_parser_feed(&p, EX2[i], &f)) {
            got = true;
        }
    }
    CHECK(got, "没有解析出帧");
    CHECK(f.seq == 0x2B, "seq=%u", f.seq);
    CHECK(f.exists == 0, "exists=%u", f.exists);
    CHECK(f.color == MX_COLOR_GREEN, "颜色字段应保留当前追踪色");
    CHECK(f.x_n == 0 && f.y_n == 0 && f.w_n == 0 && f.h_n == 0, "坐标字段必须为 0");
    CHECK(mx_dx(&f, 128) == -128, "dx=%d", mx_dx(&f, 128));
}

static void test_noise_prefix(void)
{
    printf("用例3：前面有随机噪声，仍能同步上帧头\n");
    mx_parser_t p;
    mx_parser_init(&p);
    mx_uplink_t f;
    bool got = false;
    const uint8_t noise[] = { 0x01, 0x02, 0xAA, 0xAA, 0x00, 0xBB, 0x7F };
    for (unsigned i = 0; i < sizeof(noise); i++) {
        (void)mx_parser_feed(&p, noise[i], &f);
    }
    for (unsigned i = 0; i < MX_UP_FRAME_LEN; i++) {
        if (mx_parser_feed(&p, EX1[i], &f)) {
            got = true;
        }
    }
    CHECK(got, "噪声后没有解析出帧");
    CHECK(f.seq == 0x2A, "seq=%u", f.seq);
    CHECK(p.bad_frames == 0, "噪声不该被计为坏帧，bad_frames=%u", p.bad_frames);
}

static void test_resync_after_bad_checksum(void)
{
    printf("用例4：坏帧（篡改校验和）被拒绝，且后续好帧仍能收到\n");
    mx_parser_t p;
    mx_parser_init(&p);
    uint8_t bad[MX_UP_FRAME_LEN];
    memcpy(bad, EX1, sizeof(bad));
    bad[MX_OFF_CHECKSUM] ^= 0xFFU;   /* 破坏校验和 */

    mx_uplink_t f;
    int good = 0;
    for (unsigned i = 0; i < MX_UP_FRAME_LEN; i++) {
        if (mx_parser_feed(&p, bad[i], &f)) {
            good++;
        }
    }
    CHECK(good == 0, "坏帧不该被接受");
    CHECK(p.bad_frames >= 1, "bad_frames=%u", p.bad_frames);

    /* 紧接着一个完好帧，必须能收到 */
    int ok = 0;
    for (unsigned i = 0; i < MX_UP_FRAME_LEN; i++) {
        if (mx_parser_feed(&p, EX2[i], &f)) {
            ok++;
        }
    }
    CHECK(ok == 1, "坏帧之后的好帧没收到（ok=%d）", ok);
    CHECK(f.seq == 0x2B, "seq=%u", f.seq);
}

static void test_overlapping_header(void)
{
    printf("用例5：帧头之间出现假帧头 AA AA BB，能正确恢复\n");
    mx_parser_t p;
    mx_parser_init(&p);
    const uint8_t stream[] = {
        0xAA, 0xAA, 0xBB, 0xAA, 0xAA, 0xBB, 0xBB, 0x2A, 0x01, 0x02, 0x00, 0x64,
        0x80, 0x3C, 0x28, 0x75, 0xFF, 0xFF, 0xEE, 0xEE
    };
    mx_uplink_t f;
    bool got = false;
    for (unsigned i = 0; i < sizeof(stream); i++) {
        if (mx_parser_feed(&p, stream[i], &f)) {
            got = true;
        }
    }
    CHECK(got, "假帧头之后没有恢复");
    CHECK(f.seq == 0x2A, "seq=%u", f.seq);
    CHECK(f.w_n == 60, "w_n=%u", f.w_n);
}

static void test_two_frames_in_a_row(void)
{
    printf("用例6：连续两帧粘在一起，必须都收到\n");
    mx_parser_t p;
    mx_parser_init(&p);
    mx_uplink_t f;
    int n = 0;
    uint8_t last_seq = 0;
    for (unsigned i = 0; i < MX_UP_FRAME_LEN; i++) {
        if (mx_parser_feed(&p, EX1[i], &f)) { n++; last_seq = f.seq; }
    }
    for (unsigned i = 0; i < MX_UP_FRAME_LEN; i++) {
        if (mx_parser_feed(&p, EX2[i], &f)) { n++; last_seq = f.seq; }
    }
    CHECK(n == 2, "收到 %d 帧，期望 2", n);
    CHECK(last_seq == 0x2B, "最后一帧 seq=%u", last_seq);
    CHECK(p.ok_frames == 2, "ok_frames=%u", p.ok_frames);
    CHECK(mx_seq_gap(0x2A, 0x2B) == 0, "相邻帧不该算丢帧");
    CHECK(mx_seq_gap(0x2A, 0x2E) == 3, "丢帧数算错");
}

static void test_downlink(void)
{
    printf("用例7：下行指令与回执的字节序列\n");
    uint8_t buf[MX_DOWN_FRAME_LEN];

    /* 设置追踪绿色 -> CC CC DD DD 01 02 03 FF FF EE EE */
    const uint8_t expect_set[] = {
        0xCC, 0xCC, 0xDD, 0xDD, 0x01, 0x02, 0x03, 0xFF, 0xFF, 0xEE, 0xEE
    };
    mx_build_downlink(MX_CMD_SET_COLOR, MX_COLOR_GREEN, buf);
    CHECK(memcmp(buf, expect_set, MX_DOWN_FRAME_LEN) == 0, "SET_COLOR 组帧不符");

    /* SET_COLOR 成功回执 -> CC CC DD DD 81 00 81 FF FF EE EE */
    const uint8_t expect_ack[] = {
        0xCC, 0xCC, 0xDD, 0xDD, 0x81, 0x00, 0x81, 0xFF, 0xFF, 0xEE, 0xEE
    };
    mx_build_ack(MX_CMD_SET_COLOR, MX_STATUS_OK, buf);
    CHECK(memcmp(buf, expect_ack, MX_DOWN_FRAME_LEN) == 0, "回执组帧不符");

    /* PING -> (0x7F + 0x00) & 0xFF = 0x7F */
    mx_build_downlink(MX_CMD_PING, 0x00, buf);
    CHECK(buf[4] == 0x7F && buf[5] == 0x00 && buf[6] == 0x7F, "PING 帧校验和不符");

    /* 解析回来 */
    uint8_t cmd = 0, arg = 0;
    mx_build_downlink(MX_CMD_SET_FPS, 30, buf);
    CHECK(mx_parse_downlink(buf, &cmd, &arg), "解析下行失败");
    CHECK(cmd == MX_CMD_SET_FPS && arg == 30, "解析结果 cmd=%u arg=%u", cmd, arg);

    /* 篡改校验和必须被拒绝 */
    buf[6] ^= 0xFFU;
    CHECK(!mx_parse_downlink(buf, &cmd, &arg), "坏校验和的下行帧不该被接受");

    /* 帧头不对必须被拒绝 */
    mx_build_downlink(MX_CMD_SAVE, 0, buf);
    buf[0] = 0xAA;
    CHECK(!mx_parse_downlink(buf, &cmd, &arg), "错误帧头不该被接受");
}

static void test_checksum_helper(void)
{
    printf("用例8：校验和助手函数\n");
    const uint8_t payload[8] = { 0x2A, 0x01, 0x02, 0x00, 0x64, 0x80, 0x3C, 0x28 };
    CHECK(mx_uplink_checksum(payload) == 0x75, "校验和=%02X 期望 75",
          mx_uplink_checksum(payload));

    /* 溢出后只取低 8 位：全部 0xFF，8 * 255 = 2040 = 0x7F8 -> 0xF8 */
    const uint8_t over[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    CHECK(mx_uplink_checksum(over) == 0xF8, "溢出校验和=%02X 期望 F8",
          mx_uplink_checksum(over));
}

int main(void)
{
    printf("=== mx_protocol 主机端测试 ===\n\n");
    test_doc_example_1();
    test_doc_example_2();
    test_noise_prefix();
    test_resync_after_bad_checksum();
    test_overlapping_header();
    test_two_frames_in_a_row();
    test_downlink();
    test_checksum_helper();

    printf("\n=== 结果：通过 %d 项，失败 %d 项 ===\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
