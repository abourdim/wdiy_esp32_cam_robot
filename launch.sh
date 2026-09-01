#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# CamRobot workshop launcher -- check/install PlatformIO, then build, flash,
# and monitor any of the four ESP32-CAM sketches under firmware/.
# ---------------------------------------------------------------------------
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIRMWARE_ROOT="$SCRIPT_DIR/firmware"

# --- colors -----------------------------------------------------------------
if [ -t 1 ]; then
  C_RESET='\033[0m'; C_DIM='\033[2m'
  C_CYAN='\033[36m'; C_AMBER='\033[33m'; C_RED='\033[31m'; C_GREEN='\033[32m'; C_BOLD='\033[1m'
else
  C_RESET=''; C_DIM=''; C_CYAN=''; C_AMBER=''; C_RED=''; C_GREEN=''; C_BOLD=''
fi

ok()    { echo -e "${C_GREEN}[OK]${C_RESET} $1"; }
warn()  { echo -e "${C_AMBER}[!!]${C_RESET} $1"; }
err()   { echo -e "${C_RED}[XX]${C_RESET} $1"; }
info()  { echo -e "${C_CYAN}[--]${C_RESET} $1"; }

banner() {
  echo -e "${C_CYAN}${C_BOLD}"
  echo "  ┌──────────────────────────────────────────────┐"
  echo "  │   🤖  CAMROBOT WORKSHOP -- PLATFORMIO LAUNCH   │"
  echo "  └──────────────────────────────────────────────┘"
  echo -e "${C_RESET}"
}

pause() { read -rp "$(echo -e "${C_DIM}Press Enter to continue...${C_RESET}")" _; }

# --- app discovery ------------------------------------------------------------
# Any firmware/<name>/platformio.ini is a selectable app. Sorted so the
# numeric prefixes (1_blink, 2_breathing_light, ...) come out in order.
APP_NAMES=()
APP_DIRS=()

discover_apps() {
  APP_NAMES=(); APP_DIRS=()
  while IFS= read -r ini; do
    local dir; dir="$(dirname "$ini")"
    APP_DIRS+=("$dir")
    APP_NAMES+=("$(basename "$dir")")
  done < <(find "$FIRMWARE_ROOT" -mindepth 2 -maxdepth 2 -name platformio.ini | sort)
}

APP_DIR=""
APP_NAME=""

select_app() {
  discover_apps
  if [ ${#APP_NAMES[@]} -eq 0 ]; then
    err "No PlatformIO projects found under $FIRMWARE_ROOT"
    APP_DIR=""; APP_NAME=""
    return 1
  fi
  echo
  info "Available apps:"
  local i
  for i in "${!APP_NAMES[@]}"; do
    echo "  $((i+1))) ${APP_NAMES[$i]}"
  done
  echo
  read -rp "Choose an app [1-${#APP_NAMES[@]}]: " sel || { echo; info "Bye."; exit 0; }
  if [[ "$sel" =~ ^[0-9]+$ ]] && [ "$sel" -ge 1 ] && [ "$sel" -le ${#APP_NAMES[@]} ]; then
    APP_DIR="${APP_DIRS[$((sel-1))]}"
    APP_NAME="${APP_NAMES[$((sel-1))]}"
    ok "Selected: $APP_NAME"
  else
    warn "Unrecognized choice, keeping current selection ($APP_NAME)."
  fi
  pause
}

require_app() {
  if [ -z "$APP_DIR" ]; then
    warn "No app selected yet -- pick one first."
    select_app
  fi
  [ -n "$APP_DIR" ]
}

# --- pio discovery -----------------------------------------------------------
PIO_BIN=""

find_pio() {
  if command -v pio >/dev/null 2>&1; then
    PIO_BIN="$(command -v pio)"
    return 0
  fi
  if [ -x "$HOME/.platformio/penv/bin/pio" ]; then
    PIO_BIN="$HOME/.platformio/penv/bin/pio"
    return 0
  fi
  PIO_BIN=""
  return 1
}

pio_run() {
  if ! find_pio; then
    err "PlatformIO not found. Use option 2 (Install PlatformIO) first."
    return 1
  fi
  "$PIO_BIN" "$@"
}

# --- menu actions -------------------------------------------------------------

check_install() {
  echo
  info "Checking for prerequisites..."

  if command -v python3 >/dev/null 2>&1; then
    ok "python3 found: $(python3 --version 2>&1)"
  else
    err "python3 not found -- required to install/run PlatformIO."
  fi

  if command -v pip3 >/dev/null 2>&1; then
    ok "pip3 found"
  else
    warn "pip3 not found (only needed for the pip install method)."
  fi

  if find_pio; then
    ok "PlatformIO found at: $PIO_BIN"
    "$PIO_BIN" --version
  else
    warn "PlatformIO not found. Use option 2 to install it."
  fi

  echo
  discover_apps
  if [ ${#APP_NAMES[@]} -gt 0 ]; then
    ok "Found ${#APP_NAMES[@]} firmware project(s) under $FIRMWARE_ROOT:"
    local n; for n in "${APP_NAMES[@]}"; do echo "     - $n"; done
  else
    err "No PlatformIO projects found under $FIRMWARE_ROOT"
  fi
  pause
}

install_pio() {
  echo
  info "Installing PlatformIO Core..."
  echo "  1) pip install (fast, needs python3 + pip3)"
  echo "  2) official installer script (self-contained virtualenv)"
  echo "  b) back"
  read -rp "Choose an option: " choice
  case "$choice" in
    1)
      if ! command -v pip3 >/dev/null 2>&1; then
        err "pip3 not found. Install Python 3 + pip first, or use option 2."
      else
        pip3 install -U platformio && ok "Installed. You may need to restart your shell or add pip's bin dir to PATH."
      fi
      ;;
    2)
      if ! command -v python3 >/dev/null 2>&1; then
        err "python3 not found. Install Python 3 first."
      else
        curl -fsSL -o /tmp/get-platformio.py \
          https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py \
          && python3 /tmp/get-platformio.py \
          && ok "Installed to ~/.platformio. Add ~/.platformio/penv/bin to your PATH."
      fi
      ;;
    b|B) return ;;
    *) warn "Unrecognized option." ;;
  esac
  pause
}

build_firmware() {
  require_app || return
  echo
  info "Building $APP_NAME (pio run)..."
  pio_run run -d "$APP_DIR" && ok "Build succeeded." || err "Build failed -- see output above."
  pause
}

# 4_camrobot carries two environments the default build deliberately skips:
# esp32cam-ota (network push) and esp32cam-pir (motion sensor instead of PSRAM
# -- see firmware/4_camrobot/src/pir.h). default_envs in its platformio.ini
# keeps every other menu option pointed at the normal build; this is the one
# place you can reach the others without leaving the launcher.
build_alt_env() {
  require_app || return
  echo
  info "Extra environments defined in $APP_NAME:"
  local envs; envs="$(grep -oE '^\[env:[^]]+\]' "$APP_DIR/platformio.ini" | sed 's/^\[env:\(.*\)\]$/\1/')"
  if [ -z "$envs" ]; then
    warn "None found."
    pause; return
  fi
  local e; for e in $envs; do echo "     - $e"; done
  echo
  read -rp "Environment to build (blank to cancel): " chosen
  [ -z "$chosen" ] && return
  info "Building $APP_NAME [$chosen]..."
  pio_run run -e "$chosen" -d "$APP_DIR" && ok "Build succeeded." || err "Build failed -- see output above."
  pause
}

clean_firmware() {
  require_app || return
  echo
  info "Cleaning build artifacts for $APP_NAME..."
  pio_run run -t clean -d "$APP_DIR" && ok "Cleaned."
  pause
}

list_ports() {
  echo
  info "Detected serial devices:"
  pio_run device list
  pause
}

pick_port() {
  # Prints nothing on failure/no-selection; sets $SELECTED_PORT
  SELECTED_PORT=""
  echo
  info "Available serial ports:"
  pio_run device list
  echo
  read -rp "Enter the port to use (e.g. /dev/ttyUSB0 or COM5), or leave blank for auto-detect: " SELECTED_PORT
}

flash_firmware() {
  require_app || return
  echo
  # The car this launcher came from told you to hold the IO0/BOOT button. This
  # carrier does not have one: J3 is a bare 5V/RX/TX/GND header with no
  # auto-reset, and GPIO0 is routed to the module and nowhere else. So the
  # button press becomes a wire, and it has to come off again afterwards.
  warn "This board has NO auto-reset and NO boot button. To flash:"
  warn "  1) Jumper GPIO0 to GND on the ESP32-CAM module"
  warn "  2) Tap RESET (or power-cycle)"
  warn "  3) Start the upload"
  warn "  4) Remove the jumper and reset again once it finishes"
  warn "Do this once, then use OTA from then on (menu option 9)."
  if [ "$APP_NAME" = "3_motor" ]; then
    echo
    warn "3_motor runs its drive sequence immediately on boot/reset -- make sure"
    warn "the robot has room to move. It only ever drives FORWARD (no reverse on"
    warn "this chassis), so that means room in front of it."
  fi
  echo
  pick_port
  echo
  info "Flashing $APP_NAME..."
  if [ -n "$SELECTED_PORT" ]; then
    pio_run run -t upload --upload-port "$SELECTED_PORT" -d "$APP_DIR" && ok "Flash succeeded." || err "Flash failed -- see output above."
  else
    pio_run run -t upload -d "$APP_DIR" && ok "Flash succeeded." || err "Flash failed -- see output above."
  fi
  pause
}

# Worth its own menu entry here in a way it never was on the car: with no
# auto-reset and no boot button, a wired reflash is a two-handed job and this
# is the way you will actually update a robot after the first time.
ota_flash() {
  require_app || return
  if [ "$APP_NAME" != "4_camrobot" ]; then
    warn "OTA only applies to 4_camrobot -- the lesson sketches have no network."
    pause; return
  fi
  echo
  info "The robot must be powered and on WiFi -- either its own 'wdiy1' AP"
  info "(192.168.4.1) or a router it has joined."
  echo
  read -rp "Robot IP [192.168.4.1]: " ip
  ip="${ip:-192.168.4.1}"
  echo
  info "Pushing firmware to $ip over OTA..."
  pio_run run -e esp32cam-ota -t upload --upload-port "$ip" -d "$APP_DIR" \
    && ok "OTA succeeded -- the robot reboots itself." \
    || err "OTA failed. Check the IP, and that you are on the same network."
  pause
}

monitor_serial() {
  echo
  pick_port
  echo
  info "Opening serial monitor at 115200 baud. Press Ctrl+C to exit."
  if [ -n "$SELECTED_PORT" ]; then
    pio_run device monitor -b 115200 -p "$SELECTED_PORT"
  else
    pio_run device monitor -b 115200
  fi
}

build_flash_monitor() {
  require_app || return
  build_firmware
  flash_firmware
  read -rp "Open serial monitor now? [y/N] " yn
  case "$yn" in
    y|Y) monitor_serial ;;
    *) ;;
  esac
}

main_menu() {
  while true; do
    clear 2>/dev/null || true
    banner
    if [ -n "$APP_NAME" ]; then
      echo -e "  App: ${C_AMBER}$APP_NAME${C_RESET}  ($APP_DIR)"
    else
      echo -e "  App: ${C_RED}none selected${C_RESET}"
    fi
    if find_pio; then
      echo -e "  PlatformIO: ${C_GREEN}found${C_RESET} ($PIO_BIN)"
    else
      echo -e "  PlatformIO: ${C_RED}not found${C_RESET}"
    fi
    echo
    echo "  0) Select app"
    echo "  1) Check installation"
    echo "  2) Install PlatformIO"
    echo "  3) Build firmware"
    echo "  4) Flash firmware"
    echo "  5) Build + Flash + Monitor"
    echo "  6) Serial monitor"
    echo "  7) List serial ports"
    echo "  8) Clean build"
    echo "  9) Flash over WiFi (OTA)"
    echo "  e) Build a specific environment"
    echo "  q) Quit"
    echo
    read -rp "Choose an option: " opt || { echo; info "Bye."; exit 0; }
    case "$opt" in
      0) select_app ;;
      1) check_install ;;
      2) install_pio ;;
      3) build_firmware ;;
      4) flash_firmware ;;
      5) build_flash_monitor ;;
      6) monitor_serial ;;
      7) list_ports ;;
      8) clean_firmware ;;
      9) ota_flash ;;
      e|E) build_alt_env ;;
      q|Q) echo; info "Bye."; exit 0 ;;
      *) warn "Unrecognized option." ; sleep 1 ;;
    esac
  done
}

# Prompt for an app up front so options 3-9 have something to act on immediately.
select_app
main_menu
