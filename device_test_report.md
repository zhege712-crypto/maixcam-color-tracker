# 真机验证报告：MaixCAM 上的相机端程序

日期：2026-10-16
设备：MaixCAM（LicheeRV Nano / SG2002），出厂 Buildroot 镜像
程序：`camera/maixcam_color_tracker.py`（当时协议 v2.0；**现行为 v4**）
方式：USB 虚拟网卡 + SSH（`root@10.171.126.1`）

> **历史说明（2026-10-30 补）**：本报告记录的是 **v2.0 / v3.0 时代**的实测数据。
> v4 已经改成：上行 **20 字节**、坐标 **uint16 小端像素（640×480）**、
> 第 6 字节 bit7 = 色环标志、校验和覆盖 **偏移 4~14 共 11 字节**、
> 下行 **4 字节**（1/2/3 选色 + 0x7F PING）、**没有 seq、没有回执、没有任务阶段**。
> 下文的 `seq`、`xn/yn/wn/hn`、17 字节帧、`SET_COLOR` / `SAVE` 等**只作历史参考**，
> 不能当作 v4 的验收依据。v4 的帧率实测是 **14~16 fps**。

---

## 1. 设备实况

| 项目 | 实测值 |
| --- | --- |
| 主机名 | `maixcam-ab7e` |
| 芯片 / 架构 | LicheeRV Nano / SG2002，**riscv64** |
| 内核 | Linux 5.10.4-tag，2024-03-26 构建 |
| 根文件系统 | **Buildroot**（不是 Debian，没装 pip 生态） |
| MaixPy | **4.8.2**，位于 `/usr/lib/python3.11/site-packages/maix/` |
| 摄像头 | **GCORE_GC4653**，4MP，30FPS，4-lane MIPI，传感器 ID `4653` |
| 内存 | 128MB 可见（芯片 256MB，一半被预留），空闲约 44MB |
| 存储 | `/` 28.8G，已用 5% |
| 串口设备 | `/dev/ttyS0` ~ `/dev/ttyS4` 全部存在 |
| 系统串口 | `/dev/serial0` → `ttyS0`（开机写 `serial ready`，**不要占用**） |
| USB 网卡 | `usb0`=10.171.127.1、`usb1`=10.171.126.1（NCM 与 RNDIS 各一，**任一可用**） |

设备自带的「找色块」应用配置 `/maixapp/apps/find_blobs/data/config.ini` 仍是全范围默认值
（`amin=-128, amax=127`），说明**此前从未在设备上标定过阈值**。

---

## 2. 测试矩阵（全部在真机执行）

| # | 测试 | 命令要点 | 结果 |
| --- | --- | --- | --- |
| 1 | 视觉链路 | `--duration 15 --no-display --no-uart` | ✅ 640×480 下 **~20fps**，无异常，优雅退出 |
| 2 | 几何自检 | `--duration 8 --no-display --no-uart --test-full` | ✅ `xn=128 yn=128 wn=255 hn=191`（见下） |
| 3 | 屏幕路径 | 停启动器后 `--duration 12 --no-uart` | ✅ **~29.5fps**，屏幕正常，优雅退出 |
| 4 | 串口路径 | `--duration 8 --no-display`（开 UART1） | ✅ 发送 **232 帧**，坏帧 0，无异常 |
| 5 | **装机 + 开机自启** | 打包成应用 + 写 `auto_start.txt` + 重启 | ✅ 重启后自动拉起，摄像头/屏幕/串口全开，~29.5fps |
| 6 | **电气链路（接 CH340）** | `A19`→CH340 `RXD`、`GND`→`GND`，PC 端抓包 | ✅ **20 秒 459 帧，零丢帧、零坏帧** |

每次退出后内核都打印 `maix multi-media driver released.` + `ISP Vipipe(0) Free`，
说明摄像头与显示驱动被正确释放，**没有残留占用**。

### 测试 2 的关键证据（几何换算，v3 归一化时代）

用全范围阈值匹配整幅画面，预期"铺满画面、中心在正中"：

> ⚠️ 下面是 v3 的 `xn/yn/wn/hn`（0~255）。v4 改成像素后，同样的整幅画面
> 应得到**中心 (320,240)、w=640、h=480**；这组 v4 期望值**待真机复测**。

```
[RUN] 9.7fps  seq=0 命中 xn=128 yn=128 wn=255 hn=191 像素=(0,0 640x480)
[RUN] 14.2fps seq=0 命中 xn=128 yn=128 wn=255 hn=191 像素=(0,0 640x480)
...
[INFO] 退出：识别 110 帧，发送 0 帧，坏帧 0
```

逐项对照协议定义：

| 字段 | 实测 | 理论值 | 结论 |
| --- | --- | --- | --- |
| `xn`（画面中心） | 128 | 320/640×255 = 127.5 → 128 | ✅ |
| `yn`（画面中心） | 128 | 240/480×255 = 127.5 → 128 | ✅ |
| `wn`（占满宽度） | 255 | 640/640×255 = 255 | ✅ |
| `hn`（占满高度） | **191** | 480/640×255 = 191.25 → 191 | ✅ **正好等于协议里推导的 4:3 上限** |

这条测试把"摄像头 → 找色块 → bbox → 归一化 → 协议字段"整条链路逐值钉死了，
不是"看起来能跑"，是数值对得上。

### 测试 4 的关键证据（串口）

```
[INFO] 等待主控命令……（无命令时按默认颜色持续上报）
[RUN] 25.4fps seq=26  未检测到目标
[RUN] 29.6fps seq=56  未检测到目标
...
[INFO] 退出：识别 233 帧，发送 232 帧，坏帧 0
```

说明：`pinmap` 把 `A19`/`A18` 复用为 `UART1_TX`/`UART1_RX` 成功，
`/dev/ttyS1` 打开正常，当时的 30fps 节流稳定，帧序号连续递增到 232（v4 已无 seq）。
**但电气链路尚未验证**——需要接上 STM32 或用 CH340 抓包确认。

### 测试 5 的关键证据（开机自启）

设备上的开机自启机制（从 `launcher_daemon`、`settings` 两个二进制里挖出来的）：

```
/maixapp/auto_start.txt        ← 内容 = 开机要启动的应用 id
get_curr_auto_start_app        ← 启动器里的读取函数
```

做法：把程序打包成应用放进 `/maixapp/apps/color_tracker/`（`app.yaml` + `main.py` + `app.png`），
再写入 `auto_start.txt`，重启。重启后实测：

```
$ ps -ef | grep color_tracker
  298 root     python3 /maixapp/apps/color_tracker/main.py auto_start

$ ls -l /proc/298/fd
  1  -> /maixapp/tmp/last_run.log      ← 启动器捕获的应用日志
  5  -> /dev/ttyS0
  7  -> /dev/ttyS1                     ← 我们的 UART1，确实打开了
  17 -> /dev/cvi-vo                    ← 屏幕
  15 -> /dev/cvi-vi                    ← 摄像头
```

应用自己打印的日志（`/maixapp/tmp/last_run.log`）：

```
[INFO] 追踪颜色：red  图像：640x480  发送帧率：30
[RUN] 29.5fps seq=26 未检测到目标
[RUN] 29.2fps seq=56 未检测到目标
...
[RUN] 29.7fps seq=40 未检测到目标
```

`seq` 多次回绕（255→0），证明帧在持续发送。

**两个重要结论：**

1. **由启动器启动应用时会正常让出屏幕**——`ps` 正常返回，`/proc` 没有阻塞。
   这和"启动器运行时用 SSH 直跑"会死锁形成鲜明对比（见下面第 2 条坑）。
2. 被启动器拉起时 stdout 不是终端，Python 默认块缓冲，**日志会卡在缓冲区里**。
   程序启动时执行 `sys.stdout.reconfigure(line_buffering=True)` 后日志才实时可见。

---

### 测试 6 的关键证据（电气链路，接 CH340）

接线（相机 → CH340）：`A19`(UART1_TX) → `RXD`，`GND` → `GND`。
PC 端没有 pyserial，所以用 `pc/serial_capture.ps1`（.NET 串口）抓原始字节，
再用 `pc/serial_monitor.py --file` 解码。

```
$ powershell -File pc\serial_capture.ps1 -Port COM4 -Seconds 20 -Out capture5.bin
captured 7804 bytes in 2259 reads
max gap between reads: 96 ms

$ python pc\serial_monitor.py --file capture5.bin
字节 7804   帧 459   约 22.9 fps   坏帧 0
*** 序号全程连续，零丢帧 ***
```

抽样帧并手工验算：

```
AA AA BB BB | F5 | 01 | 01 | 00 | DF | D7 | 3F | 39 | 25 | FF FF EE EE
帧头         245  有   红   0°   xn=223 yn=215 wn=63 hn=57 和   帧尾
校验和 F5+01+01+00+DF+D7+3F+39 = 805 = 0x325 → 低 8 位 0x25 ✓
```

**实测帧率约 23 fps**（不是设定的 30）：`find_blobs` 的处理耗时让主循环跑在 23fps 左右。
对协议完全够用（约 390 B/s，115200 绰绰有余）。

**期间排除了两个假警报**（以后会再遇到，记下来）：

1. **"丢了 46.57%"** —— 抓到 226 帧、序号跨度 423。实际是**设备中途重启**，
   序号复位造成跨度虚增。查 `uptime` 发现设备只开机 2 分钟，而没有坏帧、没有第二个实例。
2. **"跳了 16 帧"** —— 断口**总在第 4 帧之后**，且整包坏帧为 0。
   真因是**串口驱动缓冲区里残留着上一次打开会话的数据**，开端口时先读到了陈旧的几帧。
   修法：`Open()` 之后 `Start-Sleep -Milliseconds 200; $sp.DiscardInBuffer()`。
   修完再抓 20 秒，**序号全程连续**。

---

## 3. 踩到的坑（写下来省得再踩）

1. **设备上没有 `timeout` 命令**（BusyBox 未编入）。用 `timeout 20 python3 ...` 会得到
   `sh: timeout: not found`。所以限时改由程序自己实现（`--duration`），顺带还能走优雅退出。
2. **`display.Display()` 在启动器运行时会被阻塞**。启动器（`launcher_daemon` + `launcher`）
   占着屏幕与显示内存，第二个进程去开屏幕会卡死，并且会让 `/proc` 读取一起阻塞，
   导致 `ps`、`killall` 全部挂起，只能重启设备恢复。
   → **要开屏幕就必须先停启动器**（MaixVision 连接时也是这么做的，只是它自动完成）。
   纯 SSH 调试建议加 `--no-display`。
3. **不要用 `SIGKILL` 打断正在使用摄像头的程序**，容易把 ISP/VI 驱动留在半开状态。
   用 `--duration` 让它自己退。
4. **PID 是快照，不能隔几分钟拿来 `kill`**。PID 会被复用，我照着旧 `ps` 输出去
   `kill -9 1002 1003`，结果打断了当前会话。
5. **PowerShell 5.1 + ssh 会挂起**：`ssh` 等在 stdin 上不返回。加 `-n` 即可；
   另外 PS 5.1 会把双引号吃掉，远程命令尽量用单引号且不含双引号。
6. **设备控制台输出是 UTF-8**，本地 PS 5.1 按 GBK 读会显示成乱码，不是设备问题。

---

## 4. 复现步骤

```powershell
# 1. 上传
scp camera\maixcam_color_tracker.py root@10.171.126.1:/root/

# 2. 纯视觉调试（不碰屏幕，最安全）
ssh -n root@10.171.126.1 "python3 -u /root/maixcam_color_tracker.py --duration 15 --no-display --no-uart"

# 3. 几何自检：v3 期望 xn=128 yn=128 wn=255 hn=191；
#    v4 期望 中心(320,240)、w=640、h=480（待复测）
ssh -n root@10.171.126.1 "python3 -u /root/maixcam_color_tracker.py --duration 8 --no-display --no-uart --test-full"

# 4. 要开屏幕：先停启动器，再脱离会话运行
ssh -n root@10.171.126.1 "killall launcher_daemon launcher maixvision_server ; sleep 3 ; nohup python3 -u /root/maixcam_color_tracker.py --duration 12 --no-uart > /tmp/t.log 2>&1 < /dev/null &"

# 5. 恢复界面
ssh -n root@10.171.126.1 "sync ; sleep 1 ; /sbin/reboot -f"
```

重启后设备约 35 秒回到可用状态。

---

## 5. 还没验证的部分

| 项 | 说明 |
| --- | --- |
| **电气链路** | `A19`→STM32 `RX`、`A18`←STM32 `TX`、共地，需接上主控或用 CH340 抓包确认 |
| **颜色阈值** | 现用官方入门值，**必须对着真实物块重新标定**（用设备自带「找色块」应用点屏幕取 LAB） |
| **下行指令** | v4 下行只有 4 字节（`01/02/03` 选色、`7F` PING），**没有回执、SET_FPS、RUN、SAVE**；逻辑已由 `host_test.c` 196 项覆盖，但**没有真机双向联调过**（需要 PC 或 STM32 往 `ttyS1` 发帧） |
| **抓取标定** | `CENTER_X` / `CENTER_Y` / `GRAB_W_PX` 仍为空，需在实机上按**像素**标；这三个值在 **STM32 侧**使用，相机只上报 |
| **长时间稳定性** | 只跑了 8~15 秒的短测，未做长时间连续运行与温升观察 |

---

## 6. 结论

相机端程序在真机上**跑通了**：识别、坐标换算（v2/v3 是归一化，**v4 改成 640×480 像素**）、
协议组帧、串口发送、屏幕显示、优雅退出全部可用。当时的性能是
**~29.5fps @ 640×480**（v2/v3 的 30fps 节流上限内）；v4 加入环/块形状取样后
实测 **14~16 fps**（瓶颈是 `find_blobs`）。

**开机自启也已部署并验证**：设备重启后自动进入追踪程序，摄像头、屏幕、串口全部就绪，
日志实时可读。当前设备就处于这个状态（`/maixapp/auto_start.txt` = `color_tracker`）。

下一步应当是：**接一根线到 STM32（或先用 CH340 插到电脑），把双向链路和颜色阈值在现场定下来。**
