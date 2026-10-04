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
| Point-to-point fly | FC 3/52 (target) then 3/48 (go) | lon, lat f64; alt dm i16; speed dm/s u8. 3/48 alone → code 30 |
| Flight mode | FC 4/3 | sport (7,1), smooth (8,1), normal (8,0) |
| Limits | FC 4/5 | index 3 speed, 5 height, 7 distance; f32 |
| RTH altitude | FC 4/8 | f32 |
| **Gimbal pitch (realtime)** | GIMBAL 9/6 | 17 bytes; pitch i16 deg×100 at 13, rate i16 at 7 |
| Virtual sticks | FC 11/2, src RC, no ACK | same frame the RC streams (see §4a); app sends at 5 Hz |
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

## 4a. The RC's own frames (verified on hardware)

Captured from an RCX6E (firmware `V020SP11B160602R105`) through the Android
bridge with the aircraft off. Both CRCs verify on every frame.

| Message | Key | Rate | Content |
|---|---|---|---|
| Sticks | RC 11/2 | ~8.3 Hz | six i16 channels then a u16 key word |
| Heartbeat | RC 11/1 | 1 Hz | u16 ≈ battery centivolts (397→393), u8 ≈ battery % (98→96), 5 more bytes |
| State | RC 11/4 | 1 Hz | state, error (both 0) |
| Relay | REPEATER_RC 14/4 | ~1.2 Hz | 4 zero bytes with no aircraft |

Stick channels: 0..1023, 512 centred. In mode 2: ch1 roll (right stick
horizontal, left = 0), **ch2 pitch (right stick vertical, forward/up = 0)**,
**ch3 throttle (left stick vertical, up = 0)**, ch4 yaw (left stick
horizontal, left = 0), ch5 unused (512), ch6 gimbal wheel. In the key word,
idle `…3C`: bit 1 set while return-home is held, bits 2 (video) and 3 (photo)
clear while pressed. The upper ten bits vary continuously and are not
understood. The app's virtual-stick frame reuses this layout with key word
`0x1F1E` (the app's default `rockerKeyMessage`), **which has the RTH bit
set**, so openfimi sends the RC's idle value `0x003C` instead.

The app relays the RC stick frame to the FC (with dst rewritten to FC) when
it is using the 4G link. That explains why its virtual sticks use src = RC.

## 4b. Aircraft on the bench (verified on hardware, indoors, no GPS)

| Message | Rate | Observed |
|---|---|---|
| FC 12/1 heartbeat | 5 Hz | phase 1 on the ground; take-off caps 0 without GPS; byte 2-3 = seconds since power-on |
| FC 12/2 sport state | 5 Hz | lat/lon 0 with no fix; roll/pitch/yaw ×0.1° confirmed plausible (yaw −28.0°) |
| FC 12/3 signal | 1 Hz | 0 satellites; the three accuracy bytes read 250 with no fix; RC signal 99 % |
| FC 12/4 error codes | 1 Hz | `00000000 01000000 00200600 00000000` with no GPS |
| FC 12/5 battery | 0.5 Hz | 2 cells at 4.36 V, 2250/2258 mAh, 25.2 °C, 100 %; current field −584 (likely mA) |
| FC 12/6 home | 0.5 Hz | zeros, accuracy byte 250 (no fix) |
| GIMBAL 9/1 | 5 Hz | **angles in 0.01°** (yaw −28.98° against the FC's −28.0°) |
| CAMERA 2/21 | 2 Hz | camera state |
| CAMERA 2/135 | 0.5 Hz | the camera asking for the clock; the app answers with its own 2/135 |
| REPEATER_VEHICLE 14/4, 14/42; NFZ 17/3 | 1-2 Hz | not yet decoded |

**Gimbal 9/6 verified**: −45° reached −44.88°, −90° reached −89.88° and 0°
reached −0.11°, each in under 0.6 s at rate 20000. At rate 1000 the gimbal
creeps and stops after about a second, which is how the app's hold-to-move
works (it resends every 0.3 s). The camera ACKs 2/114 (FPV config) by echoing
its arguments.

**Video verified**: TYPE-2 records arrive at about 50/s **without** sending
2/114. Reassembled, they give clean HEVC Main 1280×720 at about 21 fps, with
VPS/SPS/PPS/SEI and an IDR roughly every 1.2 s, at about **0.4 Mbit/s**.

## 4c. First flight (auto take-off and auto land, 2026-10-04)

* **Take-off FC 3/16** answered with code 0 even though `takeOffCap` and
  `autoTakeOffCap` both read 0, so those flags do not gate take-off. The
  aircraft climbed to about 2.5 m in about 7 s and hovered.
* **Land FC 3/21** answered with code 0. Descent at about 0.3 m/s; on the
  ground 12 s later.
* `flightPhase`: **1 on the ground, 2 taking off, 3 flying/hovering, 4 landing.**
* `NavigationState.taskMode` 4 during take-off and 5 during landing;
  `apStatus` 32 while landing.
* SportState offset 22 (the app's "downVelocity") is **cm/s, positive up**:
  +81 climbing at about 0.7 m/s, −27…−34 descending at about 0.3 m/s. Ground
  speed (offset 20) read single digits while hovering, probably cm/s.
* The barometric height briefly read −0.5 m just before touchdown.

## 4d. Second flight: fly-to and return home (2026-10-04)

* **Fly-to ordering matters.** Sending 3/48 first was refused with **code 30**.
  3/52 was then accepted (code 0), but the aircraft only stores the target and
  kept hovering. The app sends **3/52 then 3/48**, which is what openfimi now does.
* **Return home (3/26)** from 0.5 m away, at 2.7 m: accepted. The aircraft
  climbed to about 6 m (not the configured 46 m), steadied, descended at about
  0.3 m/s and landed on the home point. During RTH: `taskMode` 3, `apStatus`
  2, and `flightPhase` stayed 3 until the final 1 m, then 4.
* `get_rth_altitude` (4/9) reply body: f32 = 46.0 m.
* `NavigationState.taskMode` is not a per-command mode: it read 4 throughout
  take-off **and** the following hover, 5 for land and 3 for RTH.

## 4e. Third flight: fly-to 30 m and return home (2026-10-04)

* **Fly-to (3/52 then 3/48) worked.** The target was 25 m ahead at 30 m, 5 m/s.
  Motion began about 5 s after 3/48, climbing and travelling together (peak
  3.3 m/s up and 2.8 m/s horizontal), and stopped at 29.9 m, 25.2 m from home,
  22 s after the command. `taskMode` 2 during fly-to.
* **Return home from 25 m out at 30 m:** turned toward home, **climbed to
  the configured RTH altitude (46 m)**, flew home at up to 3.6 m/s, rotated
  back to the take-off heading, then descended at 2.0 → 1.5 → 1.0 → 0.5 → 0.3
  m/s, landing 1.0 m from the take-off point. `taskMode` 3 throughout.
* **Ground speed (SportState offset 20) is cm/s**: raw 284 matched a 2.9 m/s
  change in home distance.
* Battery: 86 % → 79 % for a 2-minute flight.

## 4f. Take-off refusal and fault alarms

* `FcHeart` bytes 11 and 12 (the app's `takeOffCap` / `autoTakeOffCap`) are
  **refusal codes, 0 = take-off allowed**. After three flights in the sun
  both read **236**, and take-off was refused with **code 236**.
* At the same time FC error word A gained **bit 17**. The app's
  `assets/Alarms.json` maps FCS-A bit 17 (on the ground) to message 181:
  *"Sensor temperature too high, please power off and cool down."* The
  battery read 46 °C.
* The app's alarm system: `Alarms.json` lists entries `{GroupID, OffsetBit or
  Value, Severity, Text, IsInFlight}`, and `assets/<lang>.txt` maps the Text
  numbers to messages. Groups: FCS-A/B/C/D = the four `FcErrCode` words; MTC/ATC
  = the two heartbeat refusal codes; also RCS, NFZS, CVS and P2P. openfimi does
  not redistribute FIMI's tables; they can be read from your own copy of the APK.

## 4g. Full route with actions (7-point perimeter route, 2026-10-04)

A 7-point route at 60 m and 5.6 m/s, with a single photo plus a POI on
waypoints 2 and 4, loaded from the FIMI app database. Sent in the air: 14
frames (all ACKed), read back identical, then 3/32.

* **NavigationState during a route:** `taskMode` **1**, and `wpNUM` counts
  waypoints *reached* (0 while climbing to waypoint 0, 1 on reaching it, …,
  7 after the last). The aircraft climbs to the first waypoint's altitude
  while flying toward it.
* **End of route with finish action 4:** `taskMode` 3 with `apStatus` **5**
  (a commanded RTH shows `apStatus` 2), `wpNUM` briefly 65535 then counting RTH
  stages 0 → 3. It flew home at the route altitude (60 m, above the 46 m RTH
  altitude), descended at about 1.7 m/s, and landed 0.5 m from the take-off point.
* **Photo action (PHOTO, 1, 1):** fired on arrival. `CameraState` went 4 → 5 →
  0 (shooting, saving, idle) and the camera mode went from 32 (video) to 16
  (photo) when the route started.
* **POI and yaw (Free heading):** the aircraft yawed to waypoint *k*'s POI as
  soon as it left waypoint *k−1*, and the photo at *k* was taken at that yaw.
  Earlier flights with hover-then-photo actions found photo *k* facing POI
  *k+1*. The likely explanation: during a hover the aircraft has already
  turned to the next leg's POI. Confirm with the photos.
* Totals: 257 s, battery 100 → 85 %, battery temperature 31 → 39 °C.

## 4h. Route start and photo timing (2026-10-04, 20 Hz logging)

* As a route starts, `NavigationState.wpNUM` reads **65535** for a moment
  before 0. Treat it as "no count yet".
* At a photo waypoint the camera fired **about 0.7 s before** `wpNUM` ticked
  (wp2: camera state 4 at 151.4 s, tick at 152.2 s; wp4: 215.7 s and 216.4 s).
  Anything triggered by the tick is too late for that waypoint's photo.

## 4i. Per-waypoint speed and gimbal follower (2026-10-04)

Route at 50 m with waypoint speeds 2.8 / 8.3 / 8.3 / 11 / 8.3 / 11 / 2.8 m/s
(the route-level SPEED was 11 m/s and is never sent).

* **A waypoint's speed byte sets the speed of the leg arriving *at* that
  waypoint.** wp3 → wp4 (180 m) cruised at 8.16 m/s, wp4's 8.3 rather than
  wp3's 11; wp5 → wp6 (93 m) cruised at 2.81 m/s, wp6's 2.8 rather than wp5's 11.
  The FIMI app writes the route speed into every waypoint, which is why only
  the route speed seems to matter there.
* Acceleration and deceleration are gentle (about 0.5 m/s²): legs of 31–43 m
  peaked at 3.6–4.6 m/s whatever their set speed, and the 180 m leg spent
  about 16 s at each end changing speed.
* **Gimbal follower, on arrival (wp2, photo):** photo at 176.4 s, wpNUM tick at
  177.5 s, command at 177.6 s, gimbal at −89° by 178.1 s. The photo is taken
  at the previous pitch, as predicted.
* **Gimbal follower, before arrival (wp4, photo):** command at 210.4 s (a
  straight-line ETA of 15 s at 8.3 m/s), −45.8° by 210.7 s, photo at 235.5 s:
  25 s of real lead, because of the slow deceleration.

## 4j. Result codes, start-up and power (observed)

Result codes in the reply's 12-bit report field:

| Code | Seen on | Meaning (from context) |
|---|---|---|
| 0 | everything accepted | OK |
| 22 | land (3/21) on the ground | not flying |
| 30 | fly-to go (3/48) without a target | no target set |
| 236 | take-off (3/16) | take-off blocked: equals the heartbeat refusal code (here: sensor overheat) |

**Take-off refusal codes after power-on** (heartbeat bytes 11/12), about 10 s
from power-on to cleared: 241 → 216 → 214 → 246 → 214 → **0**, while
satellites rose from 0 to 30+ and the home point was recorded. 240 is "IMU
check in progress" in the app's alarm table; the others are unnamed there.

**Power:** the RC's phone port supplies power (the phone reported charging),
so a gadget board on that port can run off the RC.

**Heat:** the aircraft is cooled by airflow in flight. Powered on the ground
between short flights for about an hour, it reached 47 °C battery, set the
sensor-overheat fault and refused take-off until powered off and cooled.

## 4k. Virtual sticks are ignored over the RC link; no power-off command

* **The aircraft ignores virtual-stick frames (11/2, src RC → FC) sent through
  the RC.** Flight test 2026-10-04: 318 frames with deflections on four axes
  (up to full throttle, roll and pitch at ±40 %), held up to 2.7 s, while
  hovering at 2.7 m. Height stayed within 0.1 m, yaw at exactly −81.0°,
  ground speed ≤ 0.09 m/s, tilt ≤ 3°: identical to the quiet periods. The
  physical sticks were centred throughout. This matches the app, which streams
  virtual sticks only when connected over Wi-Fi directly to the aircraft
  (`n6.a.f24477a` is set only by the "connect type Wi-Fi" event), i.e. with no
  RC in the loop. Over the RC, stick input comes from the RC's own radio.
  openfimi's `ManualFlight` (`openfimi.manual`) flies keyboard moves over the
  RC link as fly-to targets instead.
* **No power-off command.** Nothing in the app powers the aircraft (or
  motors) off remotely. The alarm table has "the drone temperature is too
  high; it will shut down soon", so the aircraft can shut itself down, but no
  command for it is known. The FC firmware is encrypted, so an undocumented
  opcode cannot be ruled out.

## 4l. Manual flight through the RC: one-point routes (2026-10-04)

Three scripted flights at 15 m (`openfimi.manual.ManualFlight`):

* **Fly-to cannot be re-targeted mid-move**: 3/48 while a fly-to flies → code
  21. A route upload (3/36) during a fly-to, or for about 1.6 s after it
  arrives, → code 41; exiting it (3/51) first clears both.
* **A route can be restarted mid-flight**: stop (3/35), point (3/36), action
  (3/37), start (3/32) was accepted every time.
* **Turning:** a one-point route **at the current position** with heading Free
  and a POI 500 m along the wanted heading turns the nose in place: +42° for
  a +42° request and −45° for −46°, drifting ≤ 0.3 m.
* **Moving:** with the POI kept on the desired heading, travel goes straight
  along it and the nose doesn't drift (10.2 m forward with 0.1 m sideways). A
  waypoint at the same lat/lon at another altitude climbs or descends straight up or down.
* **Response:** each route starts after about 1 s and accelerates at about
  0.5 m/s². A 1.5 s press moved ±0.3 m, a 3 s press about 1–2 m, and a 6 s
  full-input press 10.2 m (peaking at 2.8 m/s toward a 5 m/s target). Stopping
  the route (3/35) stops a climb or descent in progress.

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
* Scales: battery current (likely mA) and time remaining.
* `flightPhase` codes 0 and 5; `FcErrCode` bits; the remaining
  `NavigationState` values; result codes beyond 0.
* Whether the FC obeys virtual-stick frames while the physical RC is also
  sending sticks over the radio, and whether it needs a mode switch first.
* The split between the two UDP ports on the Wi-Fi link.

`openfimi monitor --record` and `openfimi decode` exist to make those captures
easy to take and share.
