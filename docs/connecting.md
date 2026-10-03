# Connecting openfimi to the aircraft

The FIMI app talks to the remote controller (RC) over USB, and the RC relays
everything (commands, telemetry and live video) over its long-range radio. The
catch is the USB roles. The RC is the USB **host** and the phone is a USB
**device**, switched into *Android Open Accessory* (AOA) mode. Ordinary PCs
and laptops are USB hosts only, so plugging the RC into one does nothing:
nothing enumerates and no logs appear.

openfimi solves this with interchangeable transports. Each one carries the same
byte stream, and everything above it (the `Link`, `Drone`, CLI and video) works
the same over any of them.

| Option | Range | Extra hardware | Status |
|---|---|---|---|
| **Pi Zero 2 W as a USB gadget** (`openfimi bridge`) | full RC range | Pi Zero 2 W + cable (~US$15) | implemented; needs field test |
| **Android phone bridge app** | full RC range | none (your existing phone) | planned |
| Other Linux board with USB device mode (Pi 4/5, many SBCs) | full RC range | the board | same code as the Pi Zero |
| Direct Wi-Fi to the aircraft (`-u udp`) | **tens of metres** | Wi-Fi adapter | bench/dev only |
| `openfimi sim` simulator | n/a | none | implemented |

## 1. Raspberry Pi Zero 2 W gadget (recommended headless setup)

```
 laptop / PC / AI box  ── Wi-Fi/LAN (TCP 10052) ──  Pi Zero 2 W  ── USB (AOA) ──  RC  ~~radio~~  aircraft
 openfimi (Drone API)                              openfimi bridge
```

The Zero has two micro-USB ports. The one marked **PWR** is power only. The
one marked **USB** is the OTG data port, so it goes to the RC. Power the Pi
from its PWR port, either from a power bank or from the RC's own charging
output if it has one.

1. Flash Raspberry Pi OS Lite (Bookworm) with Wi-Fi and SSH configured.
2. On the Pi: `sudo bash scripts/pi-zero-setup.sh` (or pipe it from GitHub).
   This enables `dtoverlay=dwc2`, installs openfimi in `/opt/openfimi`, and
   installs `openfimi-bridge.service`.
3. Reboot. Connect the Pi's **USB** port to the RC's phone port (an OTG
   adapter or a micro-B to USB-C cable, as the RC requires), and switch the RC
   into its connected/command mode.
4. Check: `journalctl -u openfimi-bridge -f` should show the RC configuring
   the gadget, plus any AOA identity strings it sends.
5. From any machine on the network: `openfimi monitor -u tcp://<pi>.local`.

`openfimi doctor` on the Pi reports whether a USB device controller is present.

You can also run openfimi directly on the Pi (`-u usb`, as root) without the
bridge, for example for an on-board autonomous controller.

### Pi 4 / Pi 5

These work the same way through their USB-C port, but that port also powers
the board. Feed power through the GPIO 5 V pins, or use a USB-C power/data
splitter, so that the RC only has to provide data.

## 2. Android phone bridge (lowest friction, planned)

Every FIMI owner already has a phone that works with this RC. A small
open-source app would open the RC accessory just as the FIMI app does and relay
the byte stream over TCP on the local network (phone hotspot or shared Wi-Fi).
The computer then uses `-u tcp://<phone-ip>`. The phone becomes a dumb modem
while openfimi does the work. The wire format is the same as the Pi bridge
(raw bytes both ways), so nothing changes on the Python side.

## 3. Direct Wi-Fi (bench only)

The aircraft runs an access point (`X8Min_...`, aircraft at `192.168.40.210`)
that carries the same multiplex over UDP ports 10051/9397. It needs no USB
trickery, but **its range is very short**, so use it only for bench work. Join
the AP, then use `openfimi monitor -u udp`.

## 4. Simulator

`openfimi sim` serves a crude simulated aircraft on `127.0.0.1:10052`. Use it
to develop controllers, AI loops and UIs without hardware:

```
openfimi sim &
openfimi monitor -u tcp://127.0.0.1:10052
```

## Things that do not work

* **A PC USB port in device mode.** x86 xHCI controllers are host-only. The
  xHCI Debug Capability (DbC) can present a device with a configurable VID/PID,
  but only over a USB 3 SuperSpeed link, and it cannot answer the AOA control
  requests. It was tried against the RC and nothing enumerated.
* **A plain USB A-to-A or A-to-C cable to a PC.** Both ends are hosts.
