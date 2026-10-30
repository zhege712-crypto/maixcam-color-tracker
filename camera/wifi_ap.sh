#!/bin/sh
# ============================================================
# 把 MaixCAM 变成 WiFi 热点，手机直接连它（不需要路由器）
#
#   sh wifi_ap.sh start      开热点
#   sh wifi_ap.sh stop       关热点，恢复原来的 WiFi 客户端模式
#   sh wifi_ap.sh status     看当前状态
#
# 默认 SSID / 密码，可以用环境变量覆盖：
#   SSID=MaixCAM PASS=12345678 sh wifi_ap.sh start
#
# 手机连上之后打开： http://192.168.4.1:8000
#
# 实测踩到的坑（2026-10-30，aic8800 驱动）：
#   · `ip link set wlan0 down` 会让驱动卡在 D 状态，之后 `ps` 永久阻塞、
#     SSH 会在 banner exchange 阶段超时，只能断电重启。所以这里绝不 down 接口。
#   · `killall wpa_supplicant` 同样会让驱动进入坏状态，改用 `wpa_cli terminate`。
#   · 状态检查一律不用 `ps`，改看 `iw dev wlan0 info` 和 /proc/net/udp。
# ============================================================

IFACE=wlan0
SSID=${SSID:-MaixCAM}
PASS=${PASS:-maixcam888}
IPADDR=192.168.4.1
NETMASK=255.255.255.0
CONF=/tmp/hostapd_maixcam.conf
DNS_CONF=/tmp/dnsmasq_maixcam.conf

kill_all() {
    killall -q hostapd dnsmasq 2>/dev/null
    # 恢复客户端模式时要停掉 hostapd 之外的东西；开热点时也要先停 wpa_supplicant，
    # 否则它会一直抢 wlan0（表现为 hostapd 报 "interface busy"）。
    sleep 1
}

start_ap() {
    echo "[1/5] 停掉 WiFi 客户端（wpa_supplicant）…"
    wpa_cli -i "$IFACE" terminate 2>/dev/null
    kill_all
    # 注意：**不要** 用 `ip link set wlan0 down` —— aic8800 驱动会卡在 D 状态，
    # 之后连 `ps` 都读不动，只能重启整台设备。hostapd 自己会切 AP 模式。
    ip addr flush dev "$IFACE" 2>/dev/null

    echo "[2/5] 配置 wlan0 = $IPADDR/$NETMASK"
    ip addr add "$IPADDR/24" dev "$IFACE"
    ip link set "$IFACE" up
    sleep 1

    echo "[3/5] 写 hostapd 配置（SSID=$SSID）"
    cat > "$CONF" <<EOF
interface=$IFACE
driver=nl80211
ssid=$SSID
hw_mode=g
channel=6
ieee80211n=1
auth_algs=1
wpa=2
wpa_key_mgmt=WPA-PSK
wpa_passphrase=$PASS
rsn_pairwise=CCMP
EOF

    echo "[4/5] 启动 hostapd…"
    hostapd -B -P /tmp/hostapd_maixcam.pid "$CONF" 2>&1 | head -5
    sleep 2

    if ! iw dev "$IFACE" info 2>/dev/null | grep -q "type AP"; then
        echo "  !! hostapd 没起来，wlan0 仍是 $(iw dev "$IFACE" info 2>/dev/null | grep type)"
        echo "     常见原因：驱动不支持 AP，或 wpa_supplicant 没停干净"
        return 1
    fi

    echo "[5/5] 起 DHCP 服务（手机自动拿 IP）"
    cat > "$DNS_CONF" <<EOF
interface=$IFACE
bind-interfaces
dhcp-range=192.168.4.10,192.168.4.50,$NETMASK,12h
dhcp-option=3,$IPADDR
dhcp-option=6,$IPADDR
no-resolv
no-hosts
pid-file=/tmp/dnsmasq_maixcam.pid
EOF
    dnsmasq -C "$DNS_CONF" 2>&1 | head -3
    sleep 1

    echo
    echo "================ 热点已开 ================"
    echo "  手机连这个 WiFi : $SSID"
    echo "  密码            : $PASS"
    echo "  打开这个网址    : http://$IPADDR:8000"
    echo "=========================================="
}

stop_ap() {
    echo "关热点…"
    kill_all
    killall -q wpa_supplicant 2>/dev/null
    ip addr flush dev "$IFACE" 2>/dev/null
    # 同样不要 down/up，直接交给 S30wifi 重建
    # 重新拉起 WiFi 客户端（MaixCAM 自带的 wifi init 脚本）
    /etc/init.d/S30wifi restart >/dev/null 2>&1 || wpa_supplicant -B -i "$IFACE" -c /etc/wpa_supplicant.conf >/dev/null 2>&1
    echo "已恢复客户端模式"
}

status_ap() {
    echo "=== wlan0:"
    iw dev "$IFACE" info 2>/dev/null | grep -E "type|ssid|channel" | sed 's/^/  /'
    ip -4 addr show "$IFACE" 2>/dev/null | grep inet | sed 's/^/  /'
    echo "=== 服务:"
    grep -q ":0043" /proc/net/udp && echo "  DHCP 已监听 67" || echo "  DHCP 未启动"
    [ -f /tmp/hostapd_maixcam.pid ] && echo "  hostapd pid=$(cat /tmp/hostapd_maixcam.pid)"
    echo "=== 已连接设备:"
    iw dev "$IFACE" station dump 2>/dev/null | grep Station | sed 's/^/  /'
}

case "$1" in
    start|"") start_ap ;;
    stop)    stop_ap ;;
    status)  status_ap ;;
    *)       echo "用法: sh wifi_ap.sh [start|stop|status]" ;;
esac
