# XIAO BLE Microphone

Real-time audio processing firmware for the **Seeed XIAO BLE Sense** (nRF52840),
built with [nRF Connect SDK v3.4.0](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/3.4.0/nrf/index.html).

Each feature is an independent Kconfig module that can be included or excluded
from the build without touching the source code.

---

## Hardware

| Component | Required |
|-----------|----------|
| [Seeed XIAO BLE Sense](https://wiki.seeedstudio.com/XIAO_BLE/) (nRF52840 + PDM mic) | yes |
| [Seeed Round Display for XIAO](https://wiki.seeedstudio.com/get_start_round_display/) (240x240 touch display) | for `APP_DISPLAY_UI` |

---

## Features

| Feature | Kconfig symbol | Enabled by default | Description |
|---------|---------------|--------------------|-------------|
| Pitch detection | `APP_PITCH` | yes | Real-time monophonic pitch detector (McLeod Pitch Method). Logs note name + frequency to the shell. |
| Round display UI | `APP_DISPLAY_UI` | yes | LVGL UI on the round display: button to start/stop pitch detection, live note label. |
| WAV recording | `APP_REC` | no | Records PDM audio to the on-board QSPI flash. The flash is accessible as a USB mass-storage drive when recording is enabled. |
| Keyword spotting | `APP_KWS` | no | Continuous keyword detection using the [Edge Impulse SDK](https://docs.edgeimpulse.com/docs/run-inference/cpp-library/deploy-your-model-as-a-zephyr-application). |

---

## Prerequisites

- nRF Connect SDK v3.4.0 with a west workspace initialised (`west init` + `west update`)
- NCS toolchain installed and on `PATH`

All build commands are run from the **workspace root** (the directory containing
this repository, `nrf/`, `zephyr/`, etc.).

---

## Build

### Default -- pitch detection + round display

```bash
west build -b xiao_ble/nrf52840/sense \
  --shield seeed_xiao_round_display \
  app
```

`--shield seeed_xiao_round_display` is required whenever `CONFIG_APP_DISPLAY_UI=y`
(the default). To build without the display, remove `CONFIG_APP_DISPLAY_UI=y` and
the display-related lines from `app/prj.conf` and omit `--shield`.

### Add WAV recording

```bash
west build -b xiao_ble/nrf52840/sense \
  --shield seeed_xiao_round_display \
  app \
  -- -DEXTRA_CONF_FILE="rec.conf"
```

When `APP_REC` is enabled the device enumerates as both a CDC-ACM serial port
and a USB mass-storage drive (the 2 MB QSPI flash).

### Add keyword spotting

```bash
west build -b xiao_ble/nrf52840/sense \
  --shield seeed_xiao_round_display \
  app \
  -- -DEXTRA_CONF_FILE="kws.conf"
```

### All features

```bash
west build -b xiao_ble/nrf52840/sense \
  --shield seeed_xiao_round_display \
  app \
  -- -DEXTRA_CONF_FILE="rec.conf;kws.conf"
```

### Force clean rebuild

Add `-p always` after `west build` to wipe the build directory before configuring.

### Flash

```bash
west flash -r uf2
```

---

## Configuration fragments

Two Kconfig fragments live in `app/` and are passed via `-DEXTRA_CONF_FILE`:

| Fragment | Enables |
|----------|---------|
| `app/rec.conf` | `APP_REC`, USB MSC class, flash partition map, FAT filesystem |
| `app/kws.conf` | `APP_KWS`, Edge Impulse SDK, C++ support |

The base `app/prj.conf` always enables pitch detection (`APP_PITCH`), the round
display UI (`APP_DISPLAY_UI`), the PDM microphone subsystem, and the USB
CDC-ACM shell.

---

## Shell commands

Connect to the USB CDC-ACM port (any baud rate) to access the Zephyr shell.

### Pitch detection

```
pitch start    -- start real-time pitch detection
pitch stop     -- stop detection
pitch status   -- show current state and parameters
```

Detected notes are printed as `<freq> Hz  <note><octave>  (<cents> ct)`,
for example `440.0 Hz  A4` or `261.6 Hz  C4  (+3 ct)`.

### WAV recording (`APP_REC`)

```
record start [max_seconds]  -- start recording to recNNNN.wav on the QSPI flash
record stop                 -- finalise the WAV header and close the file
record status               -- show captured / written byte counts
```

Files are written to the FAT filesystem on the QSPI flash and are accessible
via the USB mass-storage drive after stopping the recording.

### Keyword spotting (`APP_KWS`)

```
kws start   -- start continuous keyword detection
kws stop    -- stop detection
kws status  -- show current state and label list
```

---

## Round display UI

Touch the **Start** button on the display to begin pitch detection. The button
label changes to **Stop** and the detected musical note (e.g. **A4**, **C#3**)
appears below. Touch **Stop** to end detection; the note label hides.

Pitch detection started or stopped from the shell is reflected on the display
automatically.

---

## Project structure

```
app/
  src/
    main.c            USB device init (CDC-ACM always; MSC added by APP_REC)
    pitch.c / pitch.h Pitch detector (McLeod Pitch Method)
    rec.c   / rec.h   WAV recorder
    kws.cpp / kws.h   Keyword spotter (Edge Impulse SDK)
    mic.c   / mic.h   Shared PDM microphone layer
    display_ui.c / .h Round display LVGL UI (button + note label)
  boards/
    xiao_ble_nrf52840_sense.overlay  QSPI flash + PDM devicetree overlay
  model/              Edge Impulse compiled model (used by APP_KWS)
  prj.conf            Base configuration (pitch + display on)
  rec.conf            Fragment: WAV recording
  kws.conf            Fragment: keyword spotting
```
