#!/bin/bash
#
# Helper script for building, flashing, and managing Raspberry Pi Pico W (BLE)
#
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${BASE_DIR}/build"
DEFAULT_UF2="${BUILD_DIR}/src/scara-base.uf2"

# Locate picotool binary
if command -v picotool >/dev/null 2>&1; then
    PICOTOOL="picotool"
elif [ -f "${BUILD_DIR}/_deps/picotool/picotool" ]; then
    PICOTOOL="${BUILD_DIR}/_deps/picotool/picotool"
elif [ -f "${BUILD_DIR}/_deps/picotool-build/picotool" ]; then
    PICOTOOL="${BUILD_DIR}/_deps/picotool-build/picotool"
else
    PICOTOOL=""
fi

# Detect serial port (e.g., /dev/ttyACM0)
detect_serial_port() {
    local port
    for p in /dev/ttyACM* /dev/ttyUSB*; do
        if [ -e "$p" ]; then
            echo "$p"
            return 0
        fi
    done
    return 1
}

# Detect RPI-RP2 mount point
detect_mount_point() {
    local mount_point
    mount_point=$(lsblk -o LABEL,MOUNTPOINT 2>/dev/null | awk '$1=="RPI-RP2"{print $2}')
    if [ -n "$mount_point" ] && [ -d "$mount_point" ]; then
        echo "$mount_point"
        return 0
    fi
    for path in "/media/${USER}/RPI-RP2" "/media/RPI-RP2" "/mnt/RPI-RP2" "/run/media/${USER}/RPI-RP2"; do
        if [ -d "$path" ]; then
            echo "$path"
            return 0
        fi
    done
    return 1
}

# Reboot into BOOTSEL mode via 1200 baud touch
reboot_bootsel() {
    echo "[*] Triggering reboot into BOOTSEL mode..."
    local port
    if port=$(detect_serial_port); then
        echo "[+] Found active serial port: $port"
        echo "[+] Sending 1200-baud reset trigger..."
        stty -F "$port" 1200 2>/dev/null || true
        sleep 2
        echo "[+] Reset signal sent."
        return 0
    elif [ -n "$PICOTOOL" ]; then
        "$PICOTOOL" reboot -f -u 2>/dev/null || true
        sleep 2
        return 0
    else
        echo "[-] No active serial port found. Please hold BOOTSEL button and reconnect USB cable."
        return 1
    fi
}

# Flash UF2 firmware
flash_firmware() {
    local uf2_file="${1:-$DEFAULT_UF2}"
    if [ ! -f "$uf2_file" ]; then
        echo "[-] Firmware file not found: $uf2_file"
        echo "[*] Building firmware first..."
        build_project
    fi

    echo "[*] Flashing: $uf2_file"
    
    # 1. Check if already mounted
    local mount_dir
    mount_dir=$(detect_mount_point || true)

    # 2. If not mounted, attempt reboot into BOOTSEL
    if [ -z "$mount_dir" ]; then
        reboot_bootsel || true
        # Wait up to 5 seconds for mount or picotool access
        for i in {1..10}; do
            mount_dir=$(detect_mount_point || true)
            if [ -n "$mount_dir" ]; then
                break
            fi
            sleep 0.5
        done
    fi

    # 3. Flash via mount point if available
    if [ -n "$mount_dir" ]; then
        echo "[+] Found RPI-RP2 mounted at: $mount_dir"
        cp "$uf2_file" "$mount_dir/"
        sync
        echo "[+] Firmware successfully copied! Device is restarting."
        return 0
    fi

    # 4. Fallback to picotool load if mounted folder not auto-mounted
    if [ -n "$PICOTOOL" ]; then
        echo "[*] Attempting flash via picotool..."
        if "$PICOTOOL" load -f -x "$uf2_file" 2>/dev/null; then
            echo "[+] Firmware flashed via picotool."
            return 0
        else
            echo "[-] picotool permission denied. Run ./scripts/setup_udev.sh or use sudo."
        fi
    fi

    echo "[-] Could not flash firmware. Please ensure Pico is connected in BOOTSEL mode."
    return 1
}

# Build firmware
build_project() {
    echo "[*] Building microhil_base_ble in ${BUILD_DIR}..."
    mkdir -p "${BUILD_DIR}"
    cd "${BUILD_DIR}"
    cmake ..
    make -j"$(nproc)"
    echo "[+] Build complete: ${DEFAULT_UF2}"
}

# Show device info
show_info() {
    echo "[*] USB Devices:"
    lsusb | grep -E "2e8a|Raspberry" || echo "No Raspberry Pi Pico USB device found."
    
    local port
    if port=$(detect_serial_port); then
        echo "[+] Active Serial Port: $port"
    fi

    local mount_dir
    if mount_dir=$(detect_mount_point); then
        echo "[+] BOOTSEL Mount Point: $mount_dir"
    fi

    if [ -n "$PICOTOOL" ]; then
        echo ""
        echo "[*] Picotool Device Info:"
        "$PICOTOOL" info 2>/dev/null || echo "(Run ./scripts/setup_udev.sh to allow picotool access without sudo)"
    fi
}

show_usage() {
    echo "Usage: $0 [command] [options]"
    echo ""
    echo "Commands:"
    echo "  flash [file.uf2]    Flash UF2 file to connected Pico (auto-triggers BOOTSEL if needed)"
    echo "  bootsel             Reboot connected Pico into BOOTSEL / Flashing mode"
    echo "  build               Build microhil_base_ble firmware"
    echo "  build-and-flash     Build and flash firmware in one step"
    echo "  info                Display connection and device info"
    echo "  help                Show this help message"
}

case "${1:-flash}" in
    flash)
        flash_firmware "$2"
        ;;
    bootsel|bootloader)
        reboot_bootsel
        ;;
    build)
        build_project
        ;;
    build-and-flash)
        build_project
        flash_firmware "$2"
        ;;
    info)
        show_info
        ;;
    help|-h|--help)
        show_usage
        ;;
    *)
        echo "[-] Unknown command: $1"
        show_usage
        exit 1
        ;;
esac
