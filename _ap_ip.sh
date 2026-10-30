#!/bin/sh
# 给 AP 配 IP + 起 DHCP + 自测网页（一次性）
ip addr add 192.168.4.1/24 dev wlan0 2>&1
echo "--- wlan0 IP:"
ip -4 addr show wlan0 | grep inet

killall -q dnsmasq 2>/dev/null
dnsmasq --interface=wlan0 --bind-interfaces \
        --dhcp-range=192.168.4.10,192.168.4.50,255.255.255.0,12h \
        --no-resolv --no-hosts --pid-file=/tmp/dnsmasq.pid 2>&1 | head -3
sleep 1
echo "--- dnsmasq:"
ps | grep -c dnsmasq

echo "--- 页面自测:"
python3 - <<'PYEOF'
import urllib.request
for url in ("http://192.168.4.1:8000/api/latest", "http://127.0.0.1:8000/api/latest"):
    try:
        body = urllib.request.urlopen(url, timeout=5).read().decode()
        print("  OK  ", url, body[:110])
    except Exception as e:
        print("  失败 ", url, e)
PYEOF
