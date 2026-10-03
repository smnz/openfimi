#!/usr/bin/env bash
# Prepare a Raspberry Pi (Zero 2 W, 4 or 5; Raspberry Pi OS Bookworm) to stand
# in for the phone: USB device mode, openfimi installed, bridge as a service.
#
#   curl -fsSL https://raw.githubusercontent.com/smnz/openfimi/main/scripts/pi-zero-setup.sh | sudo bash
#
# Then reboot and plug the Pi's USB *data* port (Zero: the inner micro-USB
# marked "USB", not "PWR") into the remote controller.
set -euo pipefail

[ "$(id -u)" -eq 0 ] || { echo "run as root (sudo)"; exit 1; }

BOOT=/boot/firmware
[ -d "$BOOT" ] || BOOT=/boot
CONFIG="$BOOT/config.txt"
CMDLINE="$BOOT/cmdline.txt"

echo "== enabling USB device mode (dwc2) in $CONFIG"
if ! grep -q '^dtoverlay=dwc2' "$CONFIG"; then
    printf '\n[all]\ndtoverlay=dwc2,dr_mode=peripheral\n' >> "$CONFIG"
fi
if ! grep -q 'modules-load=dwc2' "$CMDLINE"; then
    sed -i 's/$/ modules-load=dwc2/' "$CMDLINE"
fi
grep -qx libcomposite /etc/modules || echo libcomposite >> /etc/modules

echo "== installing openfimi"
apt-get update -qq
apt-get install -y -qq python3-venv python3-pip
python3 -m venv /opt/openfimi
/opt/openfimi/bin/pip install -q --upgrade pip
/opt/openfimi/bin/pip install -q "${OPENFIMI_SPEC:-git+https://github.com/smnz/openfimi}"
ln -sf /opt/openfimi/bin/openfimi /usr/local/bin/openfimi

echo "== installing the bridge service (TCP port 10052)"
cat > /etc/systemd/system/openfimi-bridge.service <<'EOF'
[Unit]
Description=openfimi: relay the FIMI remote controller's USB link over TCP
After=network-online.target sys-kernel-config.mount
Wants=network-online.target

[Service]
ExecStart=/usr/local/bin/openfimi bridge -u usb --listen 0.0.0.0:10052
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable openfimi-bridge.service

echo
echo "Done. Reboot, connect the Pi's USB data port to the remote, then from"
echo "another machine:  openfimi monitor -u tcp://$(hostname).local"
