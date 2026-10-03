# FIMI X8 Mini protocol: overview

This is the protocol the FIMI X8M Android app (`com.fimi.app.x8m`
V1.1.43.20703) uses to talk to the remote controller, and through it to the
aircraft. It was reconstructed for interoperability from a decompile of that
app. No FIMI code is included here; the `file:line` citations in the chapters
refer to the obfuscated decompile, so others can check them.

This page is the overview and the cross-cutting rules. Byte-level detail is
in four chapters:

| Chapter | Covers |
|---|---|
| [01 transport & framing](protocol/01_transport_framing.md) | AOA USB transport, both frame wrappers, CRCs, addressing, sequence numbers, RX parser, ACK matching |
| [02 commands](protocol/02_commands.md) | every command encoder: target module, opcode and payload layout |
| [03 telemetry](protocol/03_telemetry.md) | receive-side decoders; flight-critical messages byte-mapped |
| [04 video](protocol/04_video.md) | FPV video: multiplex, RTP depacketizing, codecs, start command |

Confidence tags used in the chapters: **CONFIRMED** (read from the encode or
decode code), **LIKELY**, **UNKNOWN**. Open items are listed rather than
guessed. The [reference implementation](../src/openfimi) follows these
documents. Its CRCs, waypoint frame and activation block are tested against
output from the app's own Java code.

## 1. Topology and the USB roles

```
  host (openfimi)              RC                      aircraft
  ┌──────────────┐  USB AOA  ┌─────────┐   radio    ┌─────────────────┐
  │ Drone / Link │◀────────▶│ FmLink  │◀──────────▶│ FC, gimbal,     │
  │ framing      │           │ bridge  │ long range │ camera, CV, ... │
  └──────────────┘           └─────────┘            └─────────────────┘
```

The phone connects to the RC as an **Android Open Accessory**. In AOA the
accessory (the RC) is the USB **host** and the phone is the USB **device**. A
replacement must therefore be a USB device: a board with a device controller
(Pi Zero 2 W, Pi 4/5, most SBCs) running a gadget. Desktop xHCI ports cannot
do this. See [connecting.md](connecting.md) for the options.

The accessory identity the app matches (`res/xml/accessory_filter.xml`) is
manufacturer `Beijing FIMI Technology Limited`, models `RCX6B`, `RCX6E` and
`RCX6F`. After opening the accessory the app writes one `0x00` byte, then
reads in 16 KiB chunks.

The same multiplex also runs over Wi-Fi/UDP straight to the aircraft
(`192.168.40.210`, ports 10051 and 9397; each socket binds the same local
port it sends to). That link is **short range** and only suits bench work.
Control and video share one multiplex, so the USB path carries both.

## 2. Cross-cutting rules

* **Little-endian everywhere** except inside the RTP video header. Coordinates
  are IEEE-754 f64, **longitude first, then latitude**, in WGS-84. (The app
  converts to GCJ-02 only when it is using the AMap/China map provider.)
* **Two nested wrappers, no byte stuffing.** Both resynchronise by scanning for
  their start byte and validating length plus checksum.
  * Outer (`UsbLinkPacket`): `0xAE | ver(4b)+len(12b) | TYPE | sum8 | body`,
    where `len` includes the 5-byte header. TYPE 0 = commands/telemetry,
    **2 = live video (RTP)**, 6/7 = firmware/media, 11 = 4G video. The receiver
    also accepts an 8-byte version-2 header with a 16-bit length.
  * Inner (`FmLink4`), with a 16-byte header:
    `0xFE | u16(ver=4 in bits 0-4, total len in bits 6-14) | flags | src | dst |
    0 | 0 | u16 seq | u16 crcHeader | u32 crcFrame | payload`.
* **CRCs** (see chapter 01 §5):
  * `crcHeader` = **CRC-16/MCRF4XX** (reflected 0x1021, init 0xFFFF) over header
    bytes 0..9. The app computes it but does not check it on receive.
  * `crcFrame` = CRC-32/MPEG-2 (poly 0x04C11DB7, init 0xFFFFFFFF, no
    reflection, no xorout) over the payload, with **each aligned 4-byte word
    fed most-significant byte first** and the tail zero-padded to 4 bytes.
* **flags** byte: 1 for normal acknowledged commands. It is 0 for the
  streaming, no-ACK commands: time sync, virtual sticks, GPS inject. Despite
  the field's name in the app ("encrypt"), nothing is encrypted.
* **Module ids** (header src/dst): FC=2, CAMERA=3, **GCS=7 (us)**, GIMBAL=8,
  CV=10, RC=13, REPEATER_VEHICLE=14, BATTERY=15, REPEATER_RC=16, NFZ=17,
  ESC=18. We send `src=7`, except virtual sticks, which use `src=RC(13)`. The
  app drops frames whose `dst != 7`.
* **Payload header**: `[0]=group (cmdset) [1]=msg_id [2..3]=version nibble +
  12-bit report`, followed by the body from offset 4. Many short commands
  are only `{group, msg_id}`.
* **Sequence numbers**: one wrapping counter, 0..32765. Replies echo it.
* **ACKs** match on **(group, msg_id, seq)**, not on module. On a reply the
  12-bit **report field is the result code** (0 = success). Retransmit after
  500 ms, up to 5 times.
* **No transport heartbeat.** The aircraft pushes FC telemetry continuously
  (group 12), and that push is the liveness signal.

## 3. Commands (MVP)

Payloads, exact bytes and the full catalogue are in chapter 02.

| Action | Target, group/msg | Notes |
|---|---|---|
| Take off / cancel | FC 3/16, 3/19 | `{3,16,0,0}` |
| Land / cancel | FC 3/21, 3/24 | also used for low-battery landing |
| Return home / cancel | FC 3/26, 3/29 | 2-byte payloads |
| Upload waypoint | FC 3/36 | 58 bytes, see §6 |
| Upload point action | FC 3/37 | 56 bytes |
| Start / exit mission | FC 3/32, 3/35 | no proven pause/resume (3/33, 3/34 are unlabelled candidates) |
| Read waypoint / action | FC 3/38, 3/39 | read back from the aircraft |
| Point-to-point fly | FC 3/48 then 3/52 | lon, lat f64; alt dm i16; speed dm/s u8 |
| Flight mode | FC 4/3 | sport (7,1), smooth (8,1), normal (8,0) |
| Limits | FC 4/5 | index 3 speed, 5 height, 7 distance; f32 |
| RTH altitude | FC 4/8 | f32 |
| **Gimbal pitch (realtime)** | GIMBAL 9/6 | 17 bytes; pitch i16 deg×100 at 13, rate i16 at 7 |
| Virtual sticks | FC 11/2, src RC, no ACK | roll, pitch, throttle, yaw; 0..1024, centre 512; 5 Hz |
| Photo / record | CAMERA 2/4, 2/2, 2/3 | |
| FPV stream config | CAMERA 2/114 | `(1, 1280, 720)`, sent on connect |
| Camera clock | CAMERA 2/135 | sent on connect |
| Activation | FC 1/23 | one-time product activation, not part of connecting |

**There is no emergency motor stop** in the app: no kill-motor or lock command
exists. Only the cancels above and the RC's own controls are available.

## 4. Telemetry (the continuous push)

| Message | Key (src, group, msg) | Content |
|---|---|---|
| `FcHeart` | FC 12/1 | flight phase at body offset 6: {0,1,5} on ground, {2,3,4} flying |
| `FcSportState` | FC 12/2 | lon, lat f64; height f32 m; ground and vertical speed i16 (scale unknown); roll, pitch, yaw i16 /10 deg; home distance f32 m |
| `FcSignalState` | FC 12/3 | GPS satellites; RC signal 0..100 % |
| `FcErrCode` | FC 12/4 | four u32 fault bitmasks (bits unmapped) |
| `FcBattery` | FC 12/5 | cells (raw/100 + 2.0 V), mAh, temperature /10 °C, percent |
| `HomeInfo` | FC 12/6 | home lon, lat f64; height f32 |
| `NavigationState` | FC 3/1 | task mode, nav state, autopilot status, waypoint index |
| `GimbalState` | GIMBAL 9/1 | roll, pitch, yaw i16 (scale unknown) |
| `CameraState` | CAMERA 2/21 | mode, record time, SD space |

## 5. Video

Outer TYPE 2 records are RTP packets: a 12-byte big-endian header, a 2-byte
sub-header (`0x7C` + flags: 0x80 start of access unit, 0x40 end, 0x08/0x09
AI overlay records), then Annex-B NAL data. Codec (H.265 by default, or H.264)
is detected in-band and parameter sets are in-band. Default size is 1280×720,
about 30 fps. Chapter 04 has the exact reassembly rules, which
`openfimi.video.Depacketizer` ports one-to-one.

## 6. Missions

Upload every point with FC 3/36, then every action with FC 3/37, waiting for
each ACK (the app does this; it also silently ignores a refused packet). Then
send FC 3/32 to fly. There is no route header: each point carries its index
and the point count.

58-byte waypoint payload:

| Offset | Field | Encoding |
|---|---|---|
| 4, 5 | index, count | u8 |
| 8 | longitude | f64 |
| 16 | latitude | f64 |
| 24 | altitude | i16 **decimetres** |
| 26 | yaw | i16, whole degrees × 100 |
| 28 | gimbal pitch | i16 degrees × 100 (−9000 down … +1000 up) |
| 30 | speed | u8 **decimetres/s** |
| 34 | flags | `autoRecord<<4 \| coordinatedTurnOff<<2 \| poiEnable \| 0x02` |
| 35 | heading mode \| rotation<<4 | heading 0 Free, 1 Waypoint, 2 Route; rotation 0 min-angle, 1 CW, 2 CCW |
| 36 | 1 if heading mode == 1 | |
| 38 | finish action | 0 hover, 4 return home (others unnamed) |
| 39 | RC-lost action | 0 exit, 1 continue |
| 40, 48 | POI longitude, latitude | f64 |
| 56 | POI altitude | i16 dm |

Point action: index and count at 4 and 5; actions at 8 and 9
(`NONE, HOVER, PHOTO, VIDEO, SLOW_VIDEO, PANORAMA`); parameters at 24, 25
and 26. The app's presets: hover 10 s `(HOVER, -, 10, 1, 0)`, record 10 s
`(VIDEO, -, 10, 1, 0)`, photo `(PHOTO, -, 1, 1, 0)`, burst of 3
`(PHOTO, -, 1, 3, 0)`, hover 5 s then photo `(HOVER, PHOTO, 5, 1, 1)`.

Flight-tested behaviour of the stock firmware with routes uploaded by the app:

* The per-waypoint gimbal pitch is transmitted but **ignored**. To tilt
  programmatically, send GIMBAL 9/6 yourself, for example during a hover.
* A POI steers **yaw only**, and is honoured in heading mode Free. Its bearing
  is computed from the aircraft's live position, so use POIs hundreds of
  metres away.
* **The photo taken at waypoint *k* faces the POI stored on waypoint *k+1*.**
  Plan POIs one waypoint late, and end a route with a photo-less tail point.
* The app sends one route-wide speed, heading, finish action and RC-lost action
  to every point. Per-point values beyond that are unverified.

## 7. Connecting, step by step

1. Become the AOA device (gadget). Answer the AOA handshake if the RC sends it
   (GET_PROTOCOL 51 → 2, SEND_STRING 52, START 53 → re-enumerate as
   `18d1:2d00`). Open the bulk endpoints and write one `0x00` byte.
2. Wait for FC group-12 telemetry.
3. Optionally do what the app does on connect: set the camera clock (2/135)
   and configure FPV (2/114).
4. Send commands and match ACKs; retransmit 500 ms × 5.

## 8. Open questions (settle them with captures)

* The purpose of the initial `0x00` byte, and whether the RC needs the AOA
  handshake or accepts a device that is already `18d1:2d00`.
* Scales: SportState ground and vertical speed, gimbal angles, battery current
  and time remaining.
* `flightPhase` codes 3 and 4; `FcErrCode` bits; `NavigationState` values;
  result codes beyond 0.
* Virtual-stick axis signs, and whether the FC needs a mode switch to accept
  them.
* Whether video flows without the 2/114 command, the frame rate, and the split
  between the two UDP ports.

`openfimi monitor --record` and `openfimi decode` exist to make those captures
easy to take and share.
