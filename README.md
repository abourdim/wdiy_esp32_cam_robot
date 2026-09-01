# 🤖 CamRobot

**Repo:** [github.com/abourdim/wdiy_esp32_cam_robot](https://github.com/abourdim/wdiy_esp32_cam_robot)

```
git clone https://github.com/abourdim/wdiy_esp32_cam_robot.git
cd wdiy_esp32_cam_robot
```

Firmware and web control page for the **WDIY ESP32-CAM plugin robot** — an
AI-Thinker ESP32-CAM module plugged into the `esp32_cam_plugin_robot` carrier
board (KiCad project `01_kicad/4_esp32_cam_plugin_robot`, board ID_04,
16/10/2023).

It is a port of [abourdim/video-car](https://github.com/abourdim/video-car),
which does the same job for the keyestudio KS5017 Video Smart Car. The web app
— joystick, video stream, snapshot/recording, the vision overlays, OTA, the
WiFi setup portal, the optional face on the OLED — comes across whole. What
changed is everything that touches a pin, and one of those changes is large
enough that it is worth reading before you build anything:

> ### ⚠️ This robot has no reverse
>
> Each motor is switched by a single low-side MOSFET (Q1/Q2, 2N7002) with its
> gate on a GPIO. That is one degree of freedom per motor — how hard it is
> driven — and no H-bridge, so there is no way to run a wheel backwards.
>
> Everything downstream inherits it. **Forward** and **Stop** work as they
> always did. **Left** and **Right** are a forward pivot around the undriven
> wheel rather than a spin on the spot, so the robot swings through a much
> wider arc and needs the room in front of it. The **Back** button is gone
> from the D-pad, replaced by **Stop**. The joystick's reverse half is clamped
> to zero. **Follow Me** and **Colour chase** can no longer back away from a
> target that comes too close — they stop and wait for the gap to open.
>
> None of that is a bug to be fixed in software. It is one missing H-bridge.

## Quick start

**PlatformIO:**

```bash
./launch.sh
```

Walks you through checking/installing PlatformIO, then building, flashing,
monitoring, and pushing over OTA. Or go straight at it:

```bash
cd firmware/4_camrobot && pio run -t upload
```

**Arduino IDE** (known-good baseline): open `Codes/4_CamRobot/4_CamRobot.ino`,
select the AI-Thinker ESP32-CAM board, set `Tools > Partition Scheme` to
**Minimal SPIFFS (1.9MB APP with OTA)**, enable `Tools > PSRAM: Enabled`, and
install **Adafruit NeoPixel**, **Adafruit SSD1306** and **Adafruit GFX** from
`Tools > Manage Libraries`.

> **Flashing this board is a two-handed job.** J3 is a bare 4-pin FTDI header
> — 5V, RX, TX, GND — with no DTR/RTS auto-reset circuit, and GPIO0 is not
> broken out to a button either. To flash: jumper GPIO0 to GND on the module,
> tap reset, start the upload, then remove the jumper and reset again. Do this
> once, then use [OTA](#ota-firmware-updates) forever after.

Once flashed, connect to the `wdiy1` WiFi network (password `88888888`) and
open `http://192.168.4.1` in a browser.

> ### ⚠️ The Vision features need internet — the default AP mode has none
>
> Out of the box (`ap = 1`) the robot hosts its own isolated `wdiy1` network,
> which is not connected to anything. **Pose, Hand tracking, Facial
> expression, Plates, AI Vision, Follow Me, and the jsQR fallback all download
> their models from a CDN the first time you enable them**, so on the robot's
> own AP they fail to start. Driving, the video stream, snapshots and
> recording all work fine there — it is only the model-backed overlays that do
> not.
>
> To use them, set `ap = 0` near the top of `4_CamRobot.ino` along with your
> router's `ssid`/`password` and reflash, or press **WiFi setup** on the
> control page and pick a network from the captive portal. Your phone then has
> the robot and the internet at the same time.
>
> The exception is **QR / Barcode** on Chrome, Edge or Android Chrome, which
> uses the browser's built-in `BarcodeDetector` and needs no network at all.

## The board

Everything below is read off `esp32_cam_plugin_robot.kicad_sch` and
`.kicad_pcb`, not guessed from the module.

| GPIO | Goes to | Notes |
|------|---------|-------|
| **13** | J1 screw terminal, gate of Q1 | Motor **M1 = right wheel**. PWM, forward only. R1 10k pulldown, D1 1N4007 flyback. |
| **12** | J2 screw terminal, gate of Q2 | Motor **M2 = left wheel**. Same again. **Also MTDI**, the flash-voltage strapping pin — R2's pulldown is what makes this pin safe to use. |
| **4** | D3, TX1812DB RGB pixel | The "Lights" slider. Single pixel, DOUT unconnected. Also drives the module's onboard flash LED. |
| **16** | U2/J6 motion sensor, via Q3 | **Inverted** (motion reads LOW) and **no board pull-up** (needs `INPUT_PULLUP`). Also the PSRAM chip-select — see below. |
| **1 / 3** | J3 FTDI header | U0TXD / U0RXD. No auto-reset. |
| 14, 15, 2 | nothing | Routed from the module to named nets and no further. 14/15 are where an optional OLED goes. |
| 0 | nothing | Not broken out, so flash mode is a hand-placed jumper. |

Camera pins are the stock AI-Thinker map and are untouched.

**Power.** J7 is a USB-C receptacle wired for power only — CC1/CC2 are not
populated, so it takes 5V from an A-to-C cable or a dumb supply, not from a PD
charger expecting to negotiate. `+5V` runs the module, the pixel and the
sensor. `VCC` is the separate motor rail: J5 is the battery input, and J4 is a
2-pin header that bridges `VCC` to `+5V` if you want to run everything off one
supply.

**The microSD card is not usable.** GPIO 2, 4, 12, 13, 14 and 15 are the
module's SD-card pins, and this board spends 12, 13 and 4 on the motors and the
pixel. There is nothing to fix here; the slot simply has no pins left.

### The one choice you have to make: PSRAM or motion sensor

GPIO16 is the PSRAM chip-select on an AI-Thinker module, and the carrier routes
GPIO16 to the motion sensor. They cannot both have it.

| | Camera | Motion sensor |
|---|---|---|
| **Default build** (`env:esp32cam`) | PSRAM, two QVGA buffers | not compiled in; `/status` reports `"pir":-1` |
| **`env:esp32cam-pir`** | one DRAM buffer, QVGA is a hard ceiling | works; `/status` reports 0 or 1 |

Video is what this robot is for, so PSRAM wins the default. Both environments
build, so neither path can rot:

```bash
cd firmware/4_camrobot && pio run -e esp32cam-pir
```

Note that **deleting `-DBOARD_HAS_PSRAM` from `build_flags` is not enough** —
PlatformIO's `esp32cam` board definition defines it too, in its own
`extra_flags`, which `build_flags` cannot remove. It has to be undefined with
`-UBOARD_HAS_PSRAM`. `src/pir.h` has an `#error` that fires if you get this
half-right, because a build that quietly kept PSRAM while reading GPIO16 would
report "motion" every time the cache missed.

### OLED screen (optional, and rarely fitted)

An SSD1306 128x64 at 0x3C on **GPIO14 (SDA) / GPIO15 (SCL)** gets a boot
self-test, a status readout, and the same face the other WDIY robots wear.

**It is off by default and you almost certainly want it that way.** Neither pin
is broken out on the carrier, so a screen has to be soldered to the module's own
header — and with no screen fitted, those two pins are a bus with no devices and
no pull-ups on it at all. `oledInit()` runs before the camera and before WiFi, so
probing a floating bus there risks a robot that never reaches either, with no
serial attached to say why. Build with `-DUSE_OLED=1` only when a screen is
actually soldered on.

This is one place the port came out *better* than the robots it inherits from.
On the keyestudio car and the micro:bit rovers the screen shares the bus the
motors are driven over, which means a mutex, dropped frames, and a screen that
can take the motors down with it if it is unplugged. Here the motors are PWM
pins and the screen has the bus entirely to itself: no lock, no contention, and
a missing screen costs you nothing but the screen.

## Repository layout

```
firmware/                PlatformIO projects (one per lesson)
  1_blink/               blink the RGB pixel
  2_breathing_light/     fade it up and down
  3_motor/               drive the motors directly, no WiFi
  4_camrobot/            the real thing
    src/
      4_CamRobot.ino     setup/loop, camera init, WiFi, failsafe
      SetMotor.h         the two MOSFETs, and why there is no reverse
      lights.h           the RGB pixel that replaced the car's white LED
      pir.h              the motion sensor, and the GPIO16 argument
      oled.h             optional screen: self-test, status, face
      app_server.h       web server, handlers, and the control page
      wdiy_logo.h        boot splash bitmap
    platformio.ini       board, libraries, and the PSRAM/PIR choice
    git_rev.py           stamps the git revision into the binary
    ota_flash.sh         push firmware over WiFi
Codes/                   the same sources in Arduino IDE layout
launch.sh                Interactive PlatformIO menu -- build, flash, monitor, OTA
README.html              This file, formatted, in English / French / Arabic
ports.html               Step-by-step guide to the port 80 / port 81 servers
report.html              Port report -- what the hardware forced, what was verified
preview_full.png         The control page, as it actually renders
assets/                  Logo and the OLED wiring diagrams
```

`Codes/` is a mirror of `firmware/*/src/` in the folder layout the Arduino IDE
expects. Edit under `firmware/`, then copy across.

## Web API

Unchanged from VideoCar except where the chassis forced it. Port 80 serves the
control page and the endpoints; port 81 serves `/stream`.

| Endpoint | Notes |
|---|---|
| `GET /control?var=car&val=1..5` | 1 forward, **2 no-ops and stops**, 3 left, 4 right, 5 stop |
| `GET /joystick?x=-100..100&y=0..100` | **y is clamped at 0** — negative y means nothing here |
| `GET /control?var=flash&val=0..255` | the RGB pixel, white |
| `GET /control?var=speed` (also `trim`, `framesize`, `quality`, `vflip`, `hmirror`) | as before, all persisted to NVS |
| `GET /status` | adds `"pir"` (1/0/-1) and `"reverse":0` |
| `GET /capture`, `GET /stream` | unchanged |
| `POST /update` | raw `.bin` as the request body, OTA from the browser |
| `GET /wifi-setup` | reboots into the `CamRobot-Setup` captive portal |

`val=2` answering 200 rather than an error is deliberate: a browser holding a
cached copy of an older control page should get a robot that sits still, not
one logging failures ten times a second.

**The 500ms failsafe catches everyone out.** If no drive command arrives for
500ms the robot stops itself. Driving it from `curl` means sending repeatedly,
not once.

## OTA firmware updates

With no auto-reset and no GPIO0 button, wired reflashing is tedious enough that
OTA is the normal path after the first flash. Three ways in:

- **Browser** — the *Firmware* panel on the control page. Pick a
  `firmware.bin`, upload, and it reboots itself.
- **PlatformIO** — `pio run -e esp32cam-ota -t upload --upload-port <robot-ip>`
- **Script** — `./ota_flash.sh <robot-ip> [path/to/firmware.bin]`

All three work whether the robot is on its own AP (192.168.4.1) or joined to a
router. The partition scheme is `min_spiffs.csv` — two ~1.9MB app slots — and
the firmware is currently ~1.2MB, so there is room, but not unlimited room.

## Troubleshooting

**The wheels do nothing.** Check `VCC` actually has a battery on it. `+5V`
(USB-C) runs the ESP32 but the motor rail is separate, so a robot on USB alone
boots, streams video, and cannot move. J4 bridges the two if that is what you
want.

**It turns the wrong way.** M1 (J1) is the right wheel and M2 (J2) is the left.
Nothing on the silkscreen says so. Swap the plugs rather than editing the
firmware — the mixing convention runs through `app_server.h` in several places.

**It will not go straight.** Two bare MOSFETs driving two unmatched motors off
one rail is about as open-loop as a robot gets. That is what the **Trim**
slider is for, and it is saved to NVS.

**It does not boot, or boots into flash mode.** GPIO12 is MTDI and must be low
at reset. R2 handles that — but anything else attached to J2 that pulls the
gate up will stop the module booting.

**Camera init failed.** The firmware retries at 10MHz XCLK automatically and
then reports the error. If the pixel is doing a fast red triple-flash, that is
this. Check the ribbon is fully seated (contacts toward the PCB) and that the
5V supply is a real one, not just an FTDI adapter's regulator.

**Colours on the pixel are wrong.** Change `NEO_GRB` to `NEO_RGB` in
`src/lights.h`. Nothing else cares.

**Motion is always reported.** You built with PSRAM and `USE_PIR` both on. The
`#error` in `pir.h` exists to stop that; if you reached it another way, see the
PSRAM section above.

**`/status` says `"gitrev":"nogit"`.** You built from the Arduino IDE, which
has no pre-build hook. Expected, and honest.

## Documentation

- **[report.html](report.html)** — the port report: every place the hardware
  forced a change, the corrections made along the way, and exactly what was
  and was not verified.
- **[ports.html](ports.html)** — a beginner's walkthrough of both HTTP servers,
  from joining the WiFi to driving from a shell, including the 500ms failsafe
  that catches everyone out.
- **[README.html](README.html)** — this file, formatted, with the pin map and
  wiring diagrams, in English, French and Arabic.

## Credits

Ported from [abourdim/video-car](https://github.com/abourdim/video-car), which
builds on keyestudio's stock KS5017 sketch. The OLED face descends from
`dfrobot-rover`'s MakeCode original by way of
`esp32c3_super_mini_robot-bit-rxy`. Carrier board designed in KiCad as
WORKSHOP-DIY ID_04.
