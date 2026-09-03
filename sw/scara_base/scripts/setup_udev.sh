#!/bin/bash
#
# Install udev rules for Raspberry Pi Pico (RP2040/RP2350)
# to allow picotool access without sudo.
#
set -e

echo "[*] Setting up udev rules for Raspberry Pi Pico devices..."

cat << 'EOF' | sudo tee /etc/udev/rules.d/99-pico.rules > /dev/null
# Raspberry Pi Pico (RP2040/RP2350) Bootloader and Serial
SUBSYSTEMS=="usb", ATTRS{idVendor}=="2e8a", MODE="0666", TAG+="uaccess"
KERNEL=="ttyACM*", ATTRS{idVendor}=="2e8a", MODE="0666", TAG+="uaccess"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "[+] udev rules installed to /etc/udev/rules.d/99-pico.rules"
echo "[+] You can now run picotool and flash without sudo."
