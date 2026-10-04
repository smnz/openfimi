# openfimi

Control a **FIMI X8 Mini** drone from a computer, through its own remote controller.

openfimi replaces the FIMI phone app. Your computer talks to the RC, and the
RC relays everything over its long-range radio:

* telemetry: position, height, attitude, speeds, battery, GPS, home point,
  route progress, the RC's own sticks and buttons
* flight actions: take-off, land, return home and fly-to-point
* waypoint routes: upload, verify by read-back, fly, with POIs and photo or
  video actions; load routes straight from the FIMI app's database
* **per-waypoint gimbal pitch**, which the aircraft ignores on routes, applied
  from the ground station during the flight
* hands-off launch: wait for GPS, a home point and the drone set down level
  and still, then take off and fly
* realtime gimbal tilt, camera control, and virtual sticks with a dead-man timeout
* live FPV video: H.265 720p access units for ffmpeg, OpenCV or your model

It's a pure-Python library with no runtime dependencies, plus a CLI, a
simulator and an Android bridge app. The aim is to make the aircraft
scriptable, including by AI agents.

> **Status: alpha, flight-tested.** The protocol was reverse-engineered from
> the FIMI X8M Android app and verified on an X8 Mini V3 with an RCX6E remote
> (October 2026). Working in real flight: telemetry, take-off, landing,
> fly-to-point, return home, 7-waypoint routes with photo actions, the gimbal
> follower, waiting for GPS and settling before launch, and FPV video.
> **Virtual sticks do not work through the remote** (flight-tested: the aircraft ignores them there; keyboard flying uses fly-to moves via `openfimi.manual`). Read [Safety](#safety).

## How it connects

The RC is a USB **host** that expects the phone to be a USB **device** (an
Android Open Accessory), so a normal PC USB port can't talk to it. Something
has to sit in the middle. Every option ends at the same TCP byte stream, so the
library works the same way over all of them:

| Option | URL | Range | Needs | Status |
|---|---|---|---|---|
| **Android bridge app** on your phone | `tcp://<phone-ip>` | full RC range | an Android phone | flight-tested |
| Pi Zero 2 W as USB gadget + `openfimi bridge` | `tcp://pi.local` | full RC range | a ~US$15 Pi | built, not yet tested |
| USB gadget on this machine (Pi, SBC with OTG) | `usb` | full RC range | root, device-mode USB | built, not yet tested |
| Direct Wi-Fi to the aircraft | `udp` | **short, bench only** | Wi-Fi adapter | built |
| Simulator | `tcp://127.0.0.1:10052` | n/a | nothing | built |

See [docs/connecting.md](docs/connecting.md).

## Install

```
pip install git+https://github.com/smnz/openfimi
```

The bridge app is in [`android/`](android): `./gradlew assembleDebug`, install
the APK, plug the phone into the RC and choose *openfimi bridge* when Android
asks. The FIMI app can't use the RC at the same time.

## Fly a route

```
openfimi mission fly -u tcp://<phone-ip> --fimi-db fimi.db --route "My route"
```

This runs the pre-flight checks, takes off, uploads the route in the air,
reads it back and compares it, starts it, drives the per-waypoint gimbal and
follows it to landing. A route's finish action decides whether it returns
home. Routes can also come from JSON (`examples/route.json`).

Hands-off launch: start it, then switch the drone on and carry it to the launch spot:

```
openfimi mission fly ... --wait-gps --min-sats 16 -y
```

It waits for satellites, a home point, the drone's own take-off clearance
and for the drone to sit **level and still for 5 s**. Being carried keeps it
waiting. **`-y` skips the confirmation, so it takes off as soon as all of that
holds.**

### Per-waypoint gimbal

Each waypoint has a pitch (−90 = straight down, 0 = level, +10 up) and a mode
(the FIMI database's otherwise unused `GIMBAL_MODE` column):

* **0 none** (default): the gimbal is left alone.
* **1 before arrival**: in position `--lead` seconds (15) before reaching the
  waypoint, or on leaving the previous one if the leg is shorter. Use this for photos.
* **2 on arrival**: moves when the waypoint is reached. Use it for video. A photo
  action at that waypoint is always taken *before* the move (measured: about 1.1 s).

The ground station applies these during the flight, so the RC must stay in
range for the whole route.

## Try it without hardware

```
openfimi sim &                                   # simulated aircraft on :10052
export OPENFIMI_URL=tcp://127.0.0.1:10052
openfimi monitor                                 # live telemetry as JSON lines
openfimi mission fly examples/route.json --pitch -60 -y
openfimi send gimbal -- -45
```

The simulator uses the telemetry codes the real aircraft sends (flight phases,
route progress, the 65535 start-of-route placeholder, action timing).

## Python

```python
from openfimi import Drone, transport
from openfimi.mission import mission_from_fimi_db

with Drone(transport.from_url("tcp://192.168.1.50")) as d:
    d.wait_for_telemetry()
    print(d.state.summary())             # lat, lon, height, battery, sats, faults, ...

    route = mission_from_fimi_db("fimi.db", "My route")
    d.fly_route(route, wait_ready=600, min_satellites=12, on_event=print)
```

Lower-level pieces:

```python
d.takeoff(); d.land(); d.return_home()
d.fly_to(lat, lon, alt_m=30, speed_ms=5)   # sets the target (3/52), then goes (3/48)
d.gimbal_pitch(-90)                        # reached within ~0.1 deg in under 0.6 s
d.take_photo()
print(d.preflight())                       # [] when ready, else reasons
```

Video, for example into OpenCV through PyAV:

```python
import av
codec = av.CodecContext.create("hevc", "r")
for pkt in d.video_packets():
    for frame in codec.decode(av.Packet(pkt.data)):
        img = frame.to_ndarray(format="bgr24")   # hand to your model
```

Keyboard-style flying through the remote, as fly-to moves (the aircraft ignores
virtual sticks over the remote's link; they only work over direct Wi-Fi). Inputs
are −1 to 1, and releasing or 0.6 s without updates hovers. There's no yaw:

```python
from openfimi.manual import ManualFlight
with ManualFlight(d) as m:
    m.set(pitch=0.5)          # forward, in the heading at start
```

Lower layers are public too: `Link` (sequence numbers, ACKs, retransmit),
`framing`, `commands` (exact payload builders), `telemetry` decoders, and
`commands.raw()` for experimenting with unwrapped opcodes.

## Things worth knowing

* **Speeds:** a waypoint's speed is flown on the leg *arriving at* that
  waypoint. Acceleration is gentle (about 0.5 m/s²), so legs under about 45 m
  never get much above 4.5 m/s. The FIMI app ignores per-waypoint speeds; openfimi uses them.
* **Heat:** the drone is cooled by flying. Left powered on the ground it
  overheats, then refuses take-off (code 236, "sensor temperature too high").
  Launch promptly; `preflight()` reports it.
* **Photos and POIs:** with a plain photo action, the photo at a waypoint
  faces that waypoint's POI.
* Full protocol notes and every flight-test finding: [docs/PROTOCOL.md](docs/PROTOCOL.md).

## Recording and offline decoding

```
openfimi monitor --record flight.ofcap          # record everything both ways
openfimi decode flight.ofcap --video fpv.h265   # list frames, extract video
openfimi video -u tcp://<phone-ip>              # watch live (sends nothing)
```

## Safety

* **There is no emergency motor stop.** The FIMI app has none either. The
  only remedies are the RC's own sticks and switches, plus land and return
  home. Keep the RC in hand with a pilot ready to take over.
* Hands-off launch (`--wait-gps -y`) takes off by itself. Keep clear of the
  drone once it's set down.
* Keyboard flying (`ManualFlight`) is not yet flight-tested. Its moves are
  autopilot legs that start and stop gently; keep the remote in hand.
* Fly within your local regulations and keep line of sight. You are
  responsible for what your code makes the aircraft do.

## Legal

This is an independent interoperability project. It is not affiliated with or
endorsed by FIMI or Xiaomi, and it contains no FIMI code: the protocol is
described from observation and reimplemented from scratch. "FIMI" is a
trademark of its owner.

MIT licensed.
