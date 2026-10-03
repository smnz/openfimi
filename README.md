# openfimi

Control a **FIMI X8 Mini** drone from Linux, through its own remote controller.

openfimi replaces the phone in the FIMI system. A Linux machine speaks the
RC's USB protocol and gets the RC's full radio range for:

* telemetry: position, height, attitude, battery, GPS, home point and mission state
* flight actions: take-off, land, return home and point-to-point flight
* waypoint missions: upload, read back, start and stop, with POIs and per-point actions
* direct control: realtime gimbal tilt and virtual sticks (with a dead-man timeout)
* camera: photo, video recording and settings
* live FPV video: H.265/H.264 access units, ready for ffmpeg, OpenCV or your model

It is a pure-Python library with no runtime dependencies, plus a CLI and a
simulator. The aim is to make the aircraft scriptable, including by AI agents.

> **Status: alpha.** The protocol was reverse-engineered from the FIMI X8M
> Android app. The framing, CRCs and command encoders are checked byte-for-byte
> against the app's own code. **No part has yet been tested against a real
> aircraft.** Expect rough edges and read [Safety](#safety).

## How it connects

The RC is a USB **host** that expects the phone to be a USB **device** (an
Android Open Accessory), so a normal PC USB port cannot talk to it. Pick a
transport; everything above it works the same way:

| Transport | URL | Range | Needs |
|---|---|---|---|
| Pi Zero 2 W as USB gadget + `openfimi bridge` | `tcp://pi.local` | full RC range | a ~US$15 Pi |
| USB gadget on this machine (Pi, SBC with OTG) | `usb` | full RC range | root, device-mode USB |
| Direct Wi-Fi to the aircraft | `udp` | **short, bench only** | Wi-Fi adapter |
| Simulator | `tcp://127.0.0.1:10052` | — | nothing |

An Android bridge app (use the phone you already have as a dumb USB-to-network
modem) is planned. See [docs/connecting.md](docs/connecting.md).

## Install

```
pip install git+https://github.com/smnz/openfimi
```

On a Raspberry Pi wired to the RC, `scripts/pi-zero-setup.sh` sets up USB
device mode and the bridge service.

## Try it without hardware

```
openfimi sim &                                   # simulated aircraft on :10052
export OPENFIMI_URL=tcp://127.0.0.1:10052
openfimi monitor                                 # live telemetry as JSON lines
openfimi send takeoff --yes
openfimi mission upload examples/route.json
openfimi mission start --yes
openfimi send gimbal -- -45
```

## Python

```python
from openfimi import Drone, Mission, Waypoint, PointAction, transport

with Drone(transport.from_url("tcp://pi.local")) as d:
    d.wait_for_telemetry()
    print(d.state.summary())            # lat, lon, height, battery, sats, ...

    d.takeoff()
    d.wait_until(lambda s: s.sport.height_m > 1.0, timeout=15)

    d.gimbal_pitch(-90)                 # straight down
    d.upload_mission(Mission([
        Waypoint(-43.5319, 172.6362, 30, PointAction.hover_then_photo()),
        Waypoint(-43.5319, 172.6372, 30, PointAction.photo()),
    ], speed_ms=5))
    d.start_mission()
```

Video, for example into OpenCV through PyAV:

```python
import av
codec = av.CodecContext.create("hevc", "r")
for pkt in d.video_packets():
    for frame in codec.decode(av.Packet(pkt.data)):
        img = frame.to_ndarray(format="bgr24")   # hand to your model
```

Virtual sticks for closed-loop control, with inputs from -1 to 1. If you stop
updating them for 0.6 s they re-centre:

```python
with d.sticks as s:
    s.set(pitch=0.3)          # gentle forward
    ...
```

Lower layers are public too: `Link` (sequence numbers, ACKs, retransmit),
`framing`, `commands` (exact payload builders), `telemetry` decoders, and
`commands.raw()` for experimenting with unwrapped opcodes.

## Recording and offline decoding

```
openfimi monitor --record flight.ofcap          # record everything both ways
openfimi decode flight.ofcap --video fpv.h265   # list frames, extract video
```

Captures from real hardware are the fastest way to settle the open protocol
questions listed in [docs/PROTOCOL.md](docs/PROTOCOL.md), so please share them.

## Safety

* **There is no emergency motor stop.** The FIMI app has no kill-motor command
  either. The only remedies are the RC's own sticks and switches, plus land
  and return home. Keep the RC in hand with a pilot ready to take over.
* Virtual-stick directions and scales are unconfirmed. Test every axis on the
  ground with the **propellers removed**.
* Per-waypoint gimbal pitch is transmitted, but the stock firmware ignores it
  on routes. Use `gimbal_pitch()` while hovering instead.
* Fly within your local regulations and keep line of sight. You are responsible
  for what your code makes the aircraft do.

## Legal

This is an independent interoperability project. It is not affiliated with or
endorsed by FIMI or Xiaomi, and it contains no FIMI code: the protocol is
described from observation and reimplemented from scratch. "FIMI" is a
trademark of its owner.

MIT licensed.
