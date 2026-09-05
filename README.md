# Big Beautiful Box (BBB) — Flow Meter Monitoring & Pump Control

A Raspberry Pi-based spray trailer monitoring system for agricultural helicopter operations.

## ⚠️ Version Compatibility

**This software is designed for BBB HAT v1.1**

A new HAT version (v1.2) is in development with updated GPIO mappings and features.
Check for v1.2 branch or updated documentation before use with newer hardware.

---

## Features

- **Real-time flow monitoring** via IO-Link (Picomag flow meter)
- **Auto-shutoff** with flow-rate-compensated coast prediction
- **BLE integration** with RotorSync iOS app for remote monitoring and control
- **Sensor monitoring** — battery (BMS) and tank levels (Mopeka)
- **BatchMix support** — receive mix formulas from iPad app
- **Serial control** via Switch Box (Pico-based remote)
- **7" screen dashboard** with fullscreen Tkinter GUI

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Spray Trailer                            │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────┐  │
│  │  Flow Meter  │───▶│   IOL HAT    │───▶│   Raspberry Pi   │  │
│  │  (Picomag)   │    │  (IO-Link)   │SPI │    Dashboard     │  │
│  └──────────────┘    └──────────────┘    └────────┬─────────┘  │
│                                                    │            │
│  ┌──────────────┐              ┌──────────────────┼──────────┐ │
│  │  Switch Box  │─────UART────▶│    Serial        │   BLE    │ │
│  │   (Pico)     │    RJ45      │   Listener       │  Server  │ │
│  └──────────────┘              └──────────────────┼──────────┘ │
│                                                    │            │
│  ┌──────────────┐              ┌──────────────────▼──────────┐ │
│  │ Thumbs Up    │─────GPIO────▶│      Pump Relay (K2)        │ │
│  │   Button     │              └─────────────────────────────┘ │
│  └──────────────┘                                              │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    │ Bluetooth LE
                                    ▼
                         ┌──────────────────┐
                         │   RotorSync App  │
                         │  (iPhone/iPad)   │
                         └──────────────────┘
```

## Hardware

### Main Components

| Component | Description |
|-----------|-------------|
| Raspberry Pi 5 | Main controller (Ubuntu) |
| IOL HAT | IO-Link master for flow meter (SPI) |
| Picomag Flow Meter | IO-Link industrial flow sensor |
| HDMI Display | 1920x1080 @ 30 Hz field dashboard |
| Switch Box | Pico-based pilot remote control |
| BLE Adapters | CSR (GATT server) + Realtek (sensors) |

### GPIO Pin Assignments

| GPIO | Function |
|------|----------|
| GPIO 14/15 | UART RX/TX (Serial to Switch Box) |
| GPIO 22 | Thumbs Up button input |
| GPIO 27 | Pump stop relay |
| GPIO 7-11, 24-25 | SPI + interrupts (IOL HAT) |

### Switch Box Controls

The Pico-based Switch Box connects via RJ45 and sends serial commands:

| Input | Command | Function |
|-------|---------|----------|
| Rotary encoder | `+1` / `-1` | Adjust target gallons |
| Encoder + modifier | `+10` / `-10` | Coarse adjustment |
| Pump Stop button | `PS` | Emergency pump stop |
| Override button | `OV` | Toggle auto-alert |
| Reset button | `RST` / `RESET` | Reset flow totalizer |
| Fill/Mix toggle | `FILL` / `MIX` | Switch modes |
| Thumbs Up button | `TU` | Pilot acknowledgment |

## BLE Integration (RotorSync App)

The system runs a BLE GATT server using [Bumble](https://github.com/google/bumble) that exposes:

### Service UUID
`12345678-1234-5678-1234-56789abcdef0`

### Characteristics

| UUID Suffix | Name | Type | Description |
|-------------|------|------|-------------|
| `def1` | BMS | READ | Battery status `{"voltage": x, "soc": y}` |
| `def2` | Mopeka1 | READ | Tank 1 level `{"level_mm": x, "quality": y}` |
| `def3` | Mopeka2 | READ | Tank 2 level `{"level_mm": x, "quality": y}` |
| `def4` | Pump | WRITE | Send `PS` to stop pump |
| `def5` | Gallons | WRITE | Send `+1`, `-1`, `+10`, `-10` |
| `def6` | Requested | READ | Target gallons |
| `def7` | Actual | READ | Current gallons dispensed |
| `def8` | History | READ | Last 5 fill records |
| `def9` | BatchMix | WRITE | JSON batch mix data from iPad |

### BatchMix Format

Liquid product amounts are sent in ounces only. Each liquid product entry should
include `amount_oz`; do not send jug count or jug-size fields for liquid
products. Dry products use `amount_lb`. Product rate fields are optional, but
when sent they must include both `rate_per_acre` and `rate_unit`. Liquid rates
can use `oz/ac`, `pt/ac`, `qt/ac`, or `gal/ac`; dry rates use `lb/ac`.

Product row colors are sent in the parallel `field_colors` list, using either
solid `#RRGGBB` strings or two-color striped `#RRGGBB/#RRGGBB` strings. The
first color entry is shown as the upper-left Mix badge color. If no color is
present, the badge shows `NO COLOR MIX` with white/gray striping.

When this BatchMix screen is active, knob adjustments change `water_needed`.
The box scales `total_acres`, product amounts, and `total_liquid` from that
gallon change. If a product includes rate fields, its amount is recalculated
from `rate_per_acre * total_acres` for dry products or
`rate_per_acre * liquid-ounce-multiplier * total_acres` for liquid products;
otherwise the older proportional amount scaling is used. `gallons_per_acre`
remains unchanged, and `water_needed` remains the pump target.

```json
{
  "product_count": 2,
  "products": [
    {
      "name": "Miravis Ace",
      "amount_oz": 265,
      "rate_per_acre": 26.5,
      "rate_unit": "oz/ac"
    },
    {
      "name": "AMS",
      "amount_lb": 20,
      "rate_per_acre": 2,
      "rate_unit": "lb/ac"
    }
  ],
  "field_colors": [
    {"color": "#00FF00"},
    {"color": "#FF0000/#0000FF"}
  ],
  "water_needed": 36.0,
  "total_acres": 19.3,
  "gallons_per_acre": 2.0,
  "total_liquid": 38.7,
  "timestamp": "2026-05-28T21:08:00Z"
}
```

For large payloads, BatchMix supports chunked writes:
```
CHUNK:1/3:{"product_count":2,"products":[...
CHUNK:2/3:...],"water_needed":45.5,...
CHUNK:3/3:...}
```
### Bluetooth Hardware

For the Dongle that connects to the ipad StaerTech AV53C1-USB-Bluetooth is used.

### Sensor Monitoring

The BLE server also reads nearby sensors via a second Bluetooth adapter:
- **JBD BMS** (A5:C2:37:31:77:C0) — Battery voltage and state of charge
- **Mopeka Pro** sensors — Ultrasonic tank level monitors

### BLE Architecture Notes

- `rotorsync_bumble.py` owns both Bluetooth adapters through Bumble HCI socket transports.
- The GATT/iPad adapter is selected by `GATT_ADAPTER_MAC`.
- The sensor adapter is selected by `SENSOR_ADAPTER_MAC`.
- BlueZ is intentionally stopped for the Rotorsync runtime path. The sensor side no longer uses `bleak`.
- Adapter MAC lookup is only used at startup. After startup, the GATT watchdog tracks the adapter by its resolved sysfs USB device path so it does not touch the live controller.

## Software

### Services

| Service | Description |
|---------|-------------|
| `iol_dashboard.service` | Main dashboard GUI |
| `rotorsync.service` | BLE GATT server |
| `rotorsync_watchdog.service` | BLE server monitor |

### Local GUI Simulator

For GUI work away from the trailer/Pi hardware, run:

```bash
./run_tk_sim_mac.sh
```

Simulator mode opens the real native Tkinter dashboard plus a control panel for
flow rate, switch-box commands, IO-Link disconnects, tank levels, and BMS state.
The simulator uses the same `dashboard.py` canvas code as the field box with a
virtual 1920x1080 field coordinate system, scaled down only to fit the Mac
screen. This keeps GUI sizing and spacing tied to the native Tk renderer instead
of the browser mock.

Simulator mode redirects `/home/pi` dashboard state and logs into `.sim-data/`
so local GUI testing does not touch production Pi files. Production startup is
unchanged; the simulator is enabled only by `BBB_SIM_MODE=1`.

The browser workbench is secondary and should not be used as the visual
authority for field GUI sizing:

```bash
python3 web-sim/server.py
```

Then open `http://127.0.0.1:8765/`. The browser workbench mirrors the main
dashboard layout and provides the same basic flow, switch-box, and sensor
controls for rough iteration only. Verify final GUI work in the native Tk
simulator.

### Configuration

Edit `config.py` to adjust:
- GPIO pin assignments
- Serial port settings
- Flow curve calibration
- Display refresh rate
- Log file paths

### Flow Shutoff Curve

The system predicts coast distance with the piecewise flow curve in `config.py`.
The factory curve remains in the repo and is the fallback on every boot.

Auto-shutoff timing is handled by a dedicated flow-control thread, enabled by
`FLOW_CONTROL_THREAD_ENABLED` in `config.py`. That thread owns normal IO-Link
process-data reads, updates the latest flow/totalizer state, and fires the pump
stop relay when the flow curve threshold is reached. The Tk dashboard reads the
cached state for display so GUI rendering jitter does not move the shutoff
decision point. `FLOW_CONTROL_INTERVAL` controls the safety loop period; the
default is 50 ms.

Confirmed Auto fills can also learn a conservative field correction. After the
last three thumbs-up-confirmed Auto fills, the box saves a pending proposal to
`/home/pi/flow_curve_proposal.json` and keeps the samples in
`/home/pi/flow_curve_samples.json`. The learned curve shifts the factory
intercepts by a clamped offset; it does not rewrite `config.py` and does not
become active automatically.

Use `ACCEPT CURVE` in the on-screen system menu to manually activate a pending
proposal. Use `FACTORY CURVE` to archive learned files and immediately return to
the factory curve.

## Installation

### Deployment Assumptions

These steps assume the target Pi starts from a normal Ubuntu Desktop image with the default desktop/login setup.

- User account is `pi`
- GDM is the active display manager
- Network access is available for the initial clone
- BBB hardware is connected after install as normal

The installer then layers the BBB-specific configuration on top of that default Ubuntu desktop install:

- tracked HDMI and boot settings from `deploy/boot-firmware-bbb.conf`
- tracked GDM autologin/X11 settings from `deploy/gdm3-custom.conf`
- vendored `iol-hat` source from this repo
- dashboard and Rotorsync systemd services

### Install Steps

```bash
git clone https://github.com/RotorSync/Big-Beautiful-Box.git
cd Big-Beautiful-Box
chmod +x install.sh
./install.sh
```

Run the installer as user `pi`, then reboot when it finishes.

The install script configures:
- Python dependencies
- HDMI display settings
- UART on GPIO 14/15
- Auto-login and screen timeout
- Systemd services

### Update Branch

Production devices use the on-screen updater to pull from `origin/master`.
Development can continue on `main`, but anything intended for field updates must also be pushed to `master`.

### Maintenance Update Bundle

RotorSync admin maintenance updates expect a verified BBB tar bundle. Build one
from committed runtime files with:

```bash
python3 scripts/build_update_bundle.py
```

The helper writes `dist/Big-Beautiful-Box-<VERSION>-<commit>.tar.gz`, prints the
SHA-256 and size used by the admin relay, and refuses uncommitted changes in the
included runtime paths unless `--allow-dirty` is supplied for a test bundle.
The bundle intentionally contains only the bounded runtime set that the Pi-side
maintenance updater can apply: `dashboard.py`, `rotorsync_bumble.py`,
`rotorsync_watchdog.py`, `start_iol_dashboard.sh`, `VERSION`, `config.py`,
`install.sh`, `src`, and `deploy`.

## Operational Workflow

### Fill Cycle
1. Pilot sets target gallons via Switch Box encoder
2. Ground crew monitors dashboard during fill
3. System auto-stops pump when target reached (with coast compensation)
4. Display turns green when actual is within ±2 gallons
5. Ground crew presses Thumbs Up → pilot sees confirmation
6. Fill history logged to `fill_history.log`

### BatchMix (via RotorSync App)
1. iPad sends mix formula via BLE (BatchMix characteristic)
2. Dashboard displays product list overlay
3. Water target automatically set from formula
4. Ground crew follows mix sequence

## BLE Stability Investigation (March 21, 2026)

### Symptom

The external StarTech GATT dongle would repeatedly fail with:

- `Bluetooth: hciN: command tx timeout`
- `Bluetooth: hciN: Resetting usb device`
- Bumble `BrokenPipeError: [Errno 32] Broken pipe`

When this happened, the same physical adapter kept re-enumerating as new HCI indices (`hci0`, `hci2`, `hci10`, etc.), and `rotorsync.service` would restart repeatedly.

### What Was Ruled Out

- Not a dashboard bug.
- Not a Mopeka sensor issue.
- Not just a stale USB state; reboot did not fix it.
- Not only BlueZ; removing BlueZ helped but did not fully stop the resets.
- Not just Realtek firmware loading; a firmware swap changed the loaded firmware version but did not stop the resets.
- Not just basic Bumble advertising; a minimal Bumble advertiser on the StarTech dongle was stable.

### Root Cause

There were two separate userspace conflicts:

1. BlueZ and Bumble were both touching the same GATT adapter.
2. After BlueZ was removed from the runtime path, the Rotorsync GATT watchdog was still polling the Bumble-owned adapter every 5 seconds with `hciconfig`.

That watchdog polling was enough to reproduce the failure. A minimal Bumble GATT service stayed stable until the same `hciconfig -a` / `hciconfig hciN` polling loop was added. As soon as that loop ran, the HCI socket broke and the controller reset.

### Final Fix

The final stable design is:

- Sensor scanning moved off BlueZ/`bleak` and onto Bumble on the dedicated sensor adapter.
- `rotorsync.service` no longer declares `Wants=bluetooth.target`.
- Rotorsync no longer starts `bluetooth.service`.
- The runtime watchdog no longer polls the live GATT adapter with `hciconfig`.
- Startup still resolves the GATT adapter by MAC.
- After startup, the watchdog tracks the same physical USB interface by its sysfs device path and only exits if that path disappears or rebinds to a different `hciN`.

### Verification

After the watchdog change:

- `rotorsync.service` remained active.
- No new watchdog events were added.
- No new Bumble `BrokenPipeError` entries appeared.
- A 15 minute soak test completed with no new kernel resets:

```text
command tx timeout: 57 -> 57
Resetting usb device: 58 -> 58
```

That was the first stable run after the GATT watchdog stopped touching the Bumble-owned adapter.

### Design Rule Going Forward

Once Bumble has opened the GATT adapter:

- do not poll that adapter with `hciconfig`
- do not have BlueZ manage that same controller
- do not mix `bleak`/BlueZ calls against the Bumble-owned adapter

If adapter presence must be checked at runtime, use sysfs path tracking instead of controller management commands.


## Known Issues

### Flow Meter Disconnect Recovery ([#2](https://github.com/RotorSync/Big-Beautiful-Box/issues/2)) — REVISED 2026-09-05

**Update (V2.50, 2026-09-05).** The 100 ms watchdog below turned out to be the dominant *cause*
of field disconnects once (a) the master's stdout was routed through `src/log_filter.py`
(2026-04-13), (b) the flow-control thread started polling at 50 Hz (2026-05-28), and (c) meter
faults began pulsing the pump-stop relay (2026-05-28/29). The DEBUG master build printed ~350
lines/s into that pipe with `fflush` per line under one stdout lock; any stall of the log
consumer (SD card, logrotate, CPU) blocked the IO-Link cycle thread, the 100 ms watchdog fired,
and a healthy link was torn down (`Cycle timeout in AW_REPLY - recovering` → COMLOST → dashboard
`IOL DISCONNECT` → pump-stop pulse + 10 s hold). Reproduced on demand on trailersync-sn018 by
freezing the log reader for 4 s; a real event (one late meter reply → retry → watchdog) was
captured the same day. Changes:

- `start_iol_dashboard.sh` now syncs the vendored `iol-hat` source into `~/iol-hat`, builds the
  **release** binary (no `PINEDEBUG` per-cycle logging), prefers it, and **supervises** the
  master (relaunch on exit). The in-app updater had never rebuilt the master before this.
- Watchdog reworked (`iolink_dl.c`): 1000 ms (≈80 cycles) instead of 100 ms, and a stale-expiry
  guard so a timer that raced a re-arm cannot tear down a link that just answered. Missing or
  late device replies are still caught by the MAX14819 itself (`DelayErr`/`RxErr` → retry →
  COMLOST) within one cycle; the watchdog only backstops a total interrupt loss.
- `iolhat.py` `verbose = False` (was 4 print lines per read at 50 Hz).
- Master TCP server: 2 s receive timeout on accepted sockets (a hung client can no longer wedge
  every later dashboard request); an SDCI port that is not RUNNING answers process-data reads
  with zeros instead of issuing an SMI job per request (removes the 50 Hz `DEV_NOT_IN_OPERATE`
  log flood and the 10-slot API job pool assert/abort risk); STATUS reports `pdInValid=0`
  whenever the port is not RUNNING; status thread divide-by-zero fixed.
- `osal.c`: `os_timer_start` never armed periods ≥ 1 s (`tv_nsec` overflow → `EINVAL`); fixed.
- Recovery ladder, found necessary by stall-fuzzing (SIGSTOP of the master for 0.02–10 s, 150+
  stops): (1) DL OPERATE watchdog 1 s → COMLOST → WURQ; (2) app-level port watchdog: no
  progress for 5 s → forced DL COMLOST (unwinds an SM stuck in `ReadComParameter`), then
  MAX14819 channel reset + re-establish; (3) after 4 failed attempts the master exits and the
  start script's supervisor relaunches it. Under 3 min of continuous 0.3–4 s stalls the link
  always came back within ~10 s of the stalls ending.

Still open: the meter's L+ is hardwired to the shared 24 V rail on the trailers, so
`iol_power_cycle()` cannot actually power-cycle the meter (it only flips the MAX14819 L+ switch).


The Picomag flow meter would silently lose its IO-Link connection. The dashboard detected this via stale data (identical raw bytes for 5+ seconds) and triggered power-cycles, but the IOL master daemon could not recover without a full service restart.

**Root cause:** Three bugs in the i-link DL (Data Link) layer in `iol-hat/src-master-application/ilink/iolink_dl.c`:

1. **No watchdog timer in steady-state OPERATE.** The `TInitcyc` software timer is one-shot and only fires once when entering OPERATE. The `timer_tcyc` timer infrastructure existed in skeleton form but was never wired up (never started, no event handler in `dl_main`, `timer_tcyc_elapsed` never set). Once the initial cycle begins, timing is entirely hardware-driven with no software fallback. If the MAX14819 stops generating `RXRDY` interrupts for any reason, the DL thread blocks forever in `os_event_wait()`.

2. **`timer_elapsed` not handled in `AW_REPLY_16`.** Even if a timer did fire, the `AW_REPLY_16` state handler had no check for `timer_elapsed` — it fell through to "unknown event triggered" which did nothing useful.

3. **`get_data` failure caused silent hang.** When `iolink_pl_get_data()` returned `false` (e.g., due to a FIFO level mismatch between `TxRxDataA` and `RxFIFOLvl` register reads), the DL main loop skipped calling `iolink_dl_message_h_sm()` entirely. No next TX was sent, so no response would ever come, causing a permanent hang.

**Fixes applied** (all in `iol-hat/src-master-application/ilink/iolink_dl.c`):

- **100ms watchdog timer** started after every TX message (in `get_od14`, the `AW_REPLY_16` success path, and the retry path). If no `RXRDY` arrives within 100ms, the timer fires and triggers COMLOST recovery, which re-establishes the connection automatically.
- **`timer_elapsed` handler in `AW_REPLY_16`** — when the watchdog fires, the state machine now detects it and calls `iolink_dl_mh_handle_com_lost()` to recover.
- **`get_data` failure signals `rxerror`** — when `get_data` returns false on an `RXRDY` event, the DL main loop now sets `rxerror=true` and calls `iolink_dl_message_h_sm()` so the state machine can handle it (retry or COMLOST) instead of silently hanging.
- **Retries before COMLOST** — transient RX timeouts and errors in `AW_REPLY_16` are retried up to 3 times with `PL_Resend()` before triggering a full COMLOST recovery, reducing unnecessary reconnection cycles.

Also fixed: **Port 2 cross-channel interference** — Port 2 (unused) was set to IOL mode (`-m1 0`) which caused cross-channel interference with Port 1. Changed to OFF (`-m1 3`) in `start_iol_dashboard.sh`.

## Related Repositories

- [Switch-Box-For-BBB](https://github.com/austins05/Switch-Box-For-BBB) — Pico switch box firmware
- [iol-hat](https://github.com/Pinetek-Networks/iol-hat) — IOL HAT library

## License

Proprietary — Headings Helicopters / Rotorsync

## Author

Developed by Rotorsync, 2025-2026
