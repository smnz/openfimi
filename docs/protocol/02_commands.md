# openfimi — Command (app → drone) Catalog

FIMI X8-series control protocol, reconstructed from the decompiled Android app.
Scope of this document: the **command encoders** (app builds a packet and sends it to
the drone). Telemetry/parsing (drone → app) and the raw link/transport framing are
covered in the sibling documents; only the framing fields that affect how a command is
addressed are repeated here.

All file:line citations are into the decompiled tree at
a jadx decompile of `com.fimi.app.x8m` V1.1.43.20703 (not included in this repo) (obfuscated package/class names; the original class name is in
each file's `/* compiled from: … */` comment, noted below).

Primary sources:
- `j8/c.java`  = **FcCollection**      (flight controller; also carries the mission/AiLine
                                        upload and the realtime gimbal-pitch command)
- `j8/i.java`  = **X8GimbalCollection** (gimbal module)
- `j8/a.java`  = **CameraCollection**
- `j8/j.java`  = **X8MediaCollection**
- `j8/l.java`  = **X8VisionCollection** (CV / tracking)
- `j8/f.java`  = **RcCollection**, `j8/g.java` = **RcRelayCollection**,
  `j8/d.java` = **FcRelayCollection**, `j8/b.java` = **DynamicNFZCollection**,
  `j8/e.java` = **FwUpdateCollection**
- `j8/h.java`  = **X8BaseCmd** (module enum), `j8/k.java` = **X8SendCmd** (packet assembler)
- Header/payload: `a7/a.java` **Header4**, `a7/b.java` **LinkPacket4**, `a7/c.java` **LinkPayLoad4**
- Dispatch (encoder → public API): `u8/b.java` **IFcAction** impl `z8/c.java` **FcPresenter**;
  `u8/c.java` impl `z8/b.java` **FcCtrlPresenter**. Managers (public singletons):
  `q8/e.java` **FcManager** (`q8.e.a()`), `q8/d.java` **FcCtrlManager** (`q8.d.u()`),
  `q8/k.java` **X8GimbalManager**, `q8/b.java` **CameraManager**.

---

## 1. How a command is built

Every encoder method does the same three steps (example, `j8/c.java:118` `D()` = stop):

```java
k u10 = u((byte) h.a.MODULE_FC.ordinal());   // allocate frame, set target module
u10.E(new byte[]{4, 23, 0, 0});              // write the payload (cmdset, cmdid, …)
u10.D();                                     // finalize (checksums, optional transform)
```

### 1.1 Module addressing (frame header)

`u(target)` / `v(target,…)` / `w(target,…)` (`j8/c.java:31-70`) fill the 16-byte
`Header4` (`a7/a.java:85` `f()`), of which the command-relevant fields are:

| Header byte | Field (Header4 setter)            | Value for app→drone commands |
|-------------|-----------------------------------|------------------------------|
| 0           | magic `f780q`                     | `0xFE` (constant)            |
| 1–2         | version(5 bits)+totalLen(9 bits)  | version `4`, len = payload+16 |
| 3           | flags `r()&3 \| m()&0x1C \| j()&0xE0` | see **ack flag** below   |
| 4           | **source module** `q()`           | `MODULE_GCS` = **7** (the app) — except virtual-stick, which uses `MODULE_RC`=13 |
| 5           | **target module** `i()`           | the `h.a` ordinal passed to `u()` |
| 6,7         | `n()`,`o()`                       | 0                            |
| 8–9         | **sequence** `p()` (LE16)         | per-instance auto-increment `h.f21229a` (`j8/h.java:41`) |
| 10–11       | header CRC (LE16)                 | `n7/g.java` CRC-16 over header[0..9], init `0xFFFF` |
| 12–15       | payload CRC (LE32)                | `b7/a.java` CRC-32/MPEG-2 (poly `0x04C11DB7`, init `0xFFFFFFFF`, no reflect/xor-out) over the payload |

**Ack flag (header byte 3):** `u(...)` sets `r((byte)1)` → byte3 bit0 = 1 = *ack-required*.
`v(...)`/`w(...)` set `r((byte)0)` → *no-ack / streaming* (used by GPS inject, RC virtual
stick, RTC, home-point). (`j8/c.java:31,44,58`.)

### 1.2 Module enum `h.a` (`j8/h.java:15-39`) — ordinals used as the target byte

```
0 MODULE_IDLE   1 MODULE_UAV    2 MODULE_FC     3 MODULE_CAMERA  4 MODULE_OPTFLOW
5 MODULE_OBSAVOID 6 MODULE_HTTP 7 MODULE_GCS    8 MODULE_GIMBAL  9 MODULE_BLACKBOX
10 MODULE_CV   11 MODULE_SV_DWN 12 MODULE_SV_FW 13 MODULE_RC      14 MODULE_REPEATER_VEHICLE
15 MODULE_BATTERY 16 MODULE_REPEATER_RC 17 MODULE_NFZ 18 MODULE_ESC 19 MODULE_SERVO
20 MODULE_Default0X14 21 MODULE_Default0X15 22 MODULE_ULTRASONIC
```

### 1.3 Payload layout (the `bArr` passed to `E()`)

The byte array handed to `E()` **is** the on-wire payload (`a7/c.java:95` just appends it;
`j8/k.java:38` additionally records `bArr[0]`,`bArr[1]` as the (cmdset,cmdid) to match the
reply). Universal layout:

```
offset 0 : cmdset   (command group / sub-protocol selector)
offset 1 : cmdid    (sub-command)
offset 2 : 0        (reserved — on replies this carries a result/ack code)
offset 3 : 0        (reserved)
offset 4+: arguments
```

Short "action" payloads omit bytes 2–3 entirely and are only `{cmdset, cmdid}` (2 bytes).
The reply dispatchers switch on **(cmdset, cmdid)** (`z8/c.java:151`, `z8/b.java:170`), so
that pair is the real command identity. Throughout this doc "opcode = `cmdset/cmdid`".

> ⚠ **cmdset is NOT the module.** e.g. FC commands (target module `MODULE_FC`=2) use cmdset
> 1/3/4/8/13/21 depending on the command group. Always set BOTH the target module (header)
> and the cmdset/cmdid (payload).

Opcode bytes decompiled as netty `BinaryMemcacheOpcodes.*` / memcache magic constants are
just the decompiler matching an integer to a same-valued named constant
(`io/netty/.../BinaryMemcacheOpcodes.java`). Decode table used below:

```
GET 0  SET 1  ADD 2  REPLACE 3  DELETE 4  INCREMENT 5  DECREMENT 6  QUIT 7  FLUSH 8
GETQ 9  NOOP 10  VERSION 11(0x0B)  GETK 12(0x0C)  GETKQ 13(0x0D)  APPEND 14(0x0E)
PREPEND 15(0x0F)  STAT 16(0x10)  SETQ 17(0x11)  ADDQ 18(0x12)  REPLACEQ 19(0x13)
DELETEQ 20(0x14)  INCREMENTQ 21(0x15)  DECREMENTQ 22(0x16)  QUITQ 23(0x17)  FLUSHQ 24(0x18)
APPENDQ 25(0x19)  PREPENDQ 26(0x1A)  TOUCH 28(0x1C)  GAT 29(0x1D)  GATQ 30(0x1E)
SASL_LIST_MECHS 32(0x20)  SASL_AUTH 33(0x21)  SASL_STEP 34(0x22)  GATK 35(0x23)  GATKQ 36(0x24)
REQUEST_MAGIC_BYTE 0x80(-128)  RESPONSE_MAGIC_BYTE 0x81(-127)
```
Other constants: `HttpConstants.COMMA`=44(0x2C) `COLON`=58(0x3A) `SEMICOLON`=59(0x3B);
`okio.Utf8.REPLACEMENT_BYTE`=63(0x3F); `Http2CodecUtil.MAX_UNSIGNED_BYTE`=255.

### 1.4 Number encodings (all little-endian)

- `b7.b.c(double)` (`b7/b.java:32`) → 8-byte **f64 little-endian** (IEEE-754 bits, LSB first).
- `n7.f.g(float)` (`n7/f.java:68`) → 4-byte **f32 little-endian** (it builds big-endian then
  byte-swaps the whole word).
- i16 / i32 are written LSB-first by explicit shifts (`(byte)v, (byte)(v>>8), …`).
- `n7.f.n(long)` (`n7/f.java:166`) → 8-byte LE; `b7.b.e(int)` / `n7.f.k(int)` → 4-byte LE.

### 1.5 Transport note (deferred to the frame doc)

`j8/k.java:27` `D()` builds `header(16)+payload`, then if the active link is USB
(`k8.c.b().f()/.k()`) wraps it in a 5-byte `UsbLinkPacket` (`c7/e.java`). Over WiFi/FmLink4
the 16-byte-header frame is sent as-is. This outer wrapping is **framing**, not part of the
command payload — see the frame/transport document.

---

## 2. MVP commands (full detail)

Unless stated, target module = `MODULE_FC` (2), source = `MODULE_GCS` (7), ack-required.
Byte arrays below are the exact `E(...)` payloads.

> Naming note: human names below come from the flight-state enum
> (`a5/k1.java:25-78`, class *X8mAiFlyTaskManager*) and the readable UI/manager callers.
> Where a 2-byte `{3,cmdid}` trigger's exact action could not be pinned to a UI string with
> certainty it is marked ⚠; the opcode bytes themselves are certain.

Flight-state enum (telemetry `flightMode`, for reference) — `a5/k1.java`:
`2 AUTO_TAKEOFF · 3 AUTO_LANDING · 4 POINT2POINT · 5 SURROUNDPOINT(orbit) · 6 LINE(waypoint) ·
7 AUTO_RETURN(RTH) · 8 FAILSAFE_RETURN · 9 LOWPOWER_LANDING · 11 FOLLOW · 12 HEADINGLOCK ·
13 FIXEDWING · 14 SCREW(spiral) · 15 TRIPOD · 16 AERIALPHOTOGRAPH · 20 CRUISE_CONTROL`.

### 2.1 Flight actions — VERIFIED

Target `MODULE_FC`, source GCS, ack-required. UI driver: **LeftFragment** (`d6/z0.java`,
slide-to-confirm `M0`) → **LeftModel** (`k6/b0.java`) → `FcManager q8.e.a()` → presenter
`z8/c.java`. Battery-failsafe auto-land/return also issued by
`X8BatteryReturnLandingView` (`com/fimi/app/x8s/widget/…`).

| Action | Encoder (z8.c→j8.c) | Opcode | Payload bytes | UI evidence |
|---|---|---|---|---|
| **Auto take-off** | `g2`→`a1((byte)16)` | `3/16` (STAT) | `{3,16,0,0}` | `d6/z0.java:325` slide mode 0 → `k6.b0.D()` (`:72`); dialog `x8m_main_takeoff_dialog_title`; state AUTO_TAKEOFF=2 |
| **Cancel take-off** | `s1`→`a1((byte)19)` | `3/19` (REPLACEQ) | `{3,19,0,0}` | cancel dialog `d6/z0.java:608`→`k6.b0.E()`, `AUTO_TAKEOFF_RUNNING` branch `k6/b0.java:83` |
| **Auto land** | `N`→`N((byte)21)` | `3/21` (INCREMENTQ) | `{3,21,0,0}` | slide mode 1 → `k6.b0.z()` (`:154`); dialog `x8m_main_landing_dialog_title`; also `X8BatteryReturnLandingView.k()` |
| **Cancel / abort landing** | `m2`→`N((byte)24)` | `3/24` (FLUSHQ) | `{3,24,0,0}` | cancel dialog→`k6.b0.E()`, `AUTO_LANDING_RUNNING`/`LOWPOWER_LANDING_RUNNING` branches `k6/b0.java:92` |
| **Return-to-home (RTH)** | `f`→`h0(26)` | `3/26` | `{3,26}` (2 B) | slide mode 2 → `k6.b0.m()` (`:129`); dialog `x8m_main_course_reversal_dialog_title` ("course reversal" = FIMI's RTH); also `X8BatteryReturnLandingView.l()` |
| **Cancel RTH** | `H`→`h0(29)` | `3/29` | `{3,29}` (2 B) | cancel dialog→`k6.b0.E()`, `AUTO_RETURN_RUNNING`/`FAILSAFE_RETURN_RUNNING` branches `k6/b0.java:101` |

> ⚠ **No emergency-stop / kill-motor / lock-motor command exists in this app.** An
> exhaustive search (急停/紧急/停桨/锁桨/emergency/killMotor/stopMotor across the whole tree)
> found only drone-reported failsafe *status* enums (`i8/l.java` `VCM_EMERGENCY_RTH`,
> `VCM_EMERGENCY_LANDING`), never an app-issued command. There is also no distinct "forced
> landing" opcode — low-battery/forced landing reuses the normal land command (`3/21`); only
> the aborts above exist.

### 2.1b Flight settings — VERIFIED

UI driver: **SettingFcFragment** (`h6/p4.java`) → `k6/m1.java` → `FcCtrlManager q8.d.u()`
→ presenter `z8/b.java`. Target FC, cmdset 4.

| Setting | Encoder (z8.b→j8.c) | Opcode | Payload bytes | Notes / evidence |
|---|---|---|---|---|
| **Flight mode** | `C2`→`v0(i,i)` | `4/3` | `{4,3,0,0,i,i}` | **sport**=`(7,1)` → `{4,3,0,0,7,1}`; **smooth/cinematic**=`(8,1)`; **normal**=`(8,0)` (`h6/p4.java:313-320`; labels `x8m_fly_mode_sport_heigh`/`_smooth_low`/`_ordinary`). Standalone sport toggle also via `r2`→`T0(i)` = `{4,3,0,0,7,i}` |
| **Beginner / novice** | `t`→`M0((byte)v)` | `4/1` | `{4,1,0,0,v}` | **v=0 novice ON**, **v=2 novice OFF** (`k6/m1.java:477`, log "新手模式"); read-back via `q8.d.x`→`F()` |
| **Max horizontal speed** | `A0((byte)3,f)` | `4/5` idx 3 | `{4,5,0,0,3, f32LE}` | `q8.d.c0` (`k6/m1.java:495`, log "飞行速度限制") |
| **Max height** | `A0((byte)5,f)` | `4/5` idx 5 | `{4,5,0,0,5, f32LE}` | `q8.d.Z` (`k6/m1.java:450`, "高度限制"; novice cap 120 m, hard cap 500 m) |
| **Max distance** | `A0((byte)7,f)` | `4/5` idx 7 | `{4,5,0,0,7, f32LE}` | `q8.d.Y` (`k6/m1.java:441`, "飞行距离限制") |
| **RTH altitude** | `O0(f)` | `4/8` | `{4,8,0,0, f32LE}` | `q8.d.k0` (`k6/m1.java:486`, "返航高度限制") |
| **RC-signal-lost action** | `m`→`I0((byte)b)` | `4/12` | `{4,12,0,0,b}` | hover/lead/back (`k6/m1.java:468`, strings `x8_setting_fc_loastaction_*`) |
| **Accurate-landing on/off** | `k1`→`T(1)` / `l0`→`T(0)` | `4/51` / `4/52` | `{4,51,0,0}` / `{4,52,0,0}` | ON=51, OFF=52 (`k6/m1.java:414`, `x8s21_fc_item_accurate_landing_title`) |

Float params (`A0`, `O0`) are f32 little-endian; the value unit (m, m/s) follows the slider
model — reconcile with route-DB / telemetry docs.

### 2.1c Calibration — VERIFIED

| Calibration | Chain | Opcode | Payload bytes | Args / evidence |
|---|---|---|---|---|
| **Compass / magnetometer** | `q8.d.O`→`z8.b.O`→`q0(i,i,i)` | `13/5` | `{13,5,0,0, type, step, mode}` | `type`=CALI_MAG(1); `step`=START(1)/NEXT_STEP(3)/ALL_DONE(5); `mode`=MANUAL(2). `h6/k.java` CompassCalibrateFragment; enums `i8/e.java`,`i8/a.java`,`i8/b.java` |
| **IMU / accelerometer** | `q8.d.j`→`z8.b.X1`→`o(i,i)` | `13/6` | `{13,6,0,0, type, step}` (no-ack) | `type`=IMUM(0); `step`=CALI_ACC_SIX_POINT(2)/CALI_IMU_ORTH(3). `h6/r.java` DroneCalibrateFragment |
| **Gimbal** | `q8.d.a`→`z8.b.P0`→`j8.c.a()` | `9/45` (GIMBAL module) | `{9,45,0,0}` | `h6/r1.java` GimbalCalibrationFragment, strings `x8_cloud_gimbal_*` |

(ack variants of the same opcodes also exist: compass `m0`=`13/5`, IMU `r`=`13/6`.)

### 2.2 Mission / AiLine (waypoint) commands

The waypoint route is uploaded point-by-point, then started. All are target `MODULE_FC`,
cmdset `3`.

#### 2.2.1 Waypoint upload — one point — `e0(s8.b)` — opcode `3/36` (0x24)
`j8/c.java:662`. `s8.b` = **CmdAiLinePoints** (`s8/b.java`). **58-byte** payload:

| Off | W | Type/enc | Source (`s8.b`) | Meaning / units |
|----:|--:|----------|-----------------|-----------------|
| 0   | 1 | u8       | —               | cmdset = 3 |
| 1   | 1 | u8       | —               | cmdid = 36 (0x24) |
| 2   | 1 | u8       | —               | 0 (reserved) |
| 3   | 1 | u8       | —               | 0 |
| 4   | 1 | u8       | `r()` nPos      | waypoint index (position in route) |
| 5   | 1 | u8       | `g()` count     | total waypoint count in route |
| 6   | 2 | —        | —               | 0,0 |
| 8   | 8 | f64 LE   | `l()` longitude | **longitude**, decimal degrees (f64) |
| 16  | 8 | f64 LE   | `j()` latitude  | **latitude**, decimal degrees (f64) |
| 24  | 2 | i16 LE   | `a()` altitude  | **altitude** (unit per route DB — see route-encoding doc; likely 0.1 m) |
| 26  | 2 | i16 LE   | `c()`·100       | **yaw/heading**, centidegrees (deg × 100) |
| 28  | 2 | i16 LE   | `i()` gimbalPitch | **gimbal pitch** (⚠ unit: deg or centideg — see §4) |
| 30  | 1 | u8       | `q()` speed     | **speed** (unit per route DB — see route doc) |
| 31  | 3 | —        | —               | 0,0,0 |
| 34  | 1 | u8 flags | `(d()<<4)\|(f()<<2)\|o()\|0x02` | bit layout: `d()`=autoRecord «4, `f()`=coordinatedTurnOff «2, `o()`=POI-enable «0, +const `0x02` |
| 35  | 1 | u8       | `n()\|(p()<<4)`   | low nibble = `n()` orientation, high nibble = `p()` rotation direction |
| 36  | 1 | u8       | `n()==1?1:0`    | 1 iff orientation==1 |
| 37  | 1 | u8       | —               | 0 |
| 38  | 1 | u8       | `e()` compeletEvent | **mission-finish action** (end-of-route behaviour) |
| 39  | 1 | u8       | `h()` disconnectEvent | **RC-lost action** |
| 40  | 8 | f64 LE   | `m()` longitudePIO | **POI longitude**, decimal degrees |
| 48  | 8 | f64 LE   | `k()` latitudePIO  | **POI latitude**, decimal degrees |
| 56  | 2 | i16 LE   | `b()` altitudePIO  | **POI altitude** (same unit as alt) |

`s8.b` field↔accessor map (from `s8/b.java` toString + getters):
`q()`=speed, `c()`=angle(float, yaw), `n()`=orientation, `p()`=rotation, `j()`=latitude,
`l()`=longitude, `a()`=altitude, `g()`=count, `r()`=nPos, `k()`=POI-lat, `m()`=POI-lon,
`b()`=POI-alt, `h()`=disconnectEvent(RC-lost), `e()`=compeletEvent(finish),
`d()`=autoRecord, `f()`=coordinatedTurnOff, `o()`=pioEnable(POI on), `i()`=gimbalPitch.

> There is **no separate mission-header frame**: the per-point index(`off 4`)+count(`off 5`)
> carry the structure. The app-side route container is *X8AilinePrameter* (`h5/e.java`:
> speed, orientation, disconnectActioin, endAction, autoRecorde) — a UI model that is copied
> into each `s8.b` point; it is **not** itself serialized to the drone.

#### 2.2.2 Waypoint point-action — `f0(s8.c)` — opcode `3/37` — `j8/c.java:728`
56-byte payload. `s8.c` = **CmdAiLinePointsAction** (`s8/c.java`), action enum
`a{NULL,HOVER,PHOTO,VIDEO,SLOW_VIDEO,PANORAMA}`:
```
off0 =3  off1 =37  off2,3 =0
off4 = (byte)cVar.f28662b     off5 = (byte)cVar.f28661a
off8 = (byte)cVar.f28663c     off9 = (byte)cVar.f28664d
off24= (byte)cVar.f28665e     off25= (byte)cVar.f28666f   off26=(byte)cVar.f28667g
(remaining bytes 0)
```
(Field semantics of `s8.c` are only positional in the decompiled source — `f28661a..g` —
no names recovered; treat as action-slot parameters. ⚠)

#### 2.2.3 Waypoint-route lifecycle — VERIFIED
Driver: **AiLineCmdDataManager** (`a5/d0.java`) uploads every point then every point-action,
then executes; **X8mAiflyExcuteController** (`z4/h0.java`) issues the in-flight exit.

| Action | Encoder (z8.c→j8.c) | Opcode | Payload | Evidence |
|---|---|---|---|---|
| Upload waypoint #i | `L0`→`e0(s8.b)` | `3/36` | 58 B (§2.2.1) | `a5/d0.java:234` (loops all points) |
| Upload point-action #i | `D0`→`f0(s8.c)` | `3/37` | 56 B (§2.2.2) | `a5/d0.java:219` |
| **Execute / start route** | `E1`→`b0()` | `3/32` (0x20) | `{3,32}` | `a5/d0.java:304`, log "startExcute" `:158` |
| **Stop / exit running route** | `u2`→`c0()` | `3/35` (GATK) | `{3,35}` | `z4/h0.java:384` (LINRE_RUNNING); `x8_ai_fly_route_exit` |
| **Get-points: read waypoint #i** (reply gives count, `r8.j0`) | `j2`→`k(i)` | `3/38` | `{3,38,0,0,i8}` | `a5/d0.java:357,367` (reads pt 0 → count, then loops) |
| Get-points: read POI/ext #i (`r8.k0`) | `b`→`l(i)` | `3/39` | `{3,39,0,0,i8}` | `a5/d0.java:378` |

> ⚠ **There is no dedicated waypoint pause/resume opcode.** The AI-fly status enum
> (`i5/e.java`) has only `LINRE_RUNNING / LINRE_FINISH / LINRE_FINISH_CONTINUE`
> (auto-continue after an RC reconnect). The only in-flight route control is **exit = `3/35`**.
> The two unlabeled buttons in the run overlay (`z4/h0.java:682,689`) map to `d0()`=`3/33`
> and `g0()`=`3/34` and are the *only* pause/resume candidates, but no string/label confirms
> it (layout XML not in the decompiled tree). Do not assume pause/resume = 3/33/3/34.

#### 2.2.4 Other AI-fly modes — VERIFIED (same FC/cmdset-3 chain)
Start/query come from per-mode controllers in `a5/**` / `e6/**`; exits from `z4/h0.java`.

| Mode / action | Encoder | Opcode | Payload (if args) | Evidence |
|---|---|---|---|---|
| **Tap-to-fly GO** (Point2Point, state 4) | `G0`→`X(d,d,i,i)` | `3/52` (0x34) | `{3,52,0,0, c0 f64@4, c1 f64@12, alt×10 i16@20, 0,0, speed×10 i8@24}` | `a5/g.java:45` (AiD2Point), log "指点飞行" |
| Tap-fly arm / confirm | `x0`→`Y(48)` | `3/48` | `{3,48}` | `a5/g.java:35` |
| Tap-fly query point (`r8.l0`) | `m0`→`Y(53)` | `3/53` | `{3,53}` | `a5/g.java:54` |
| Tap-fly exit | `I2`→`Y(51)` | `3/51` | `{3,51}` | `z4/h0.java:402`, `x8_ai_fly_p2p_exite` |
| **Orbit / Spiral SET & GO** | `Q`→`k0(68,…)` | `3/68` (0x44) | `{3,68,0,0, ctr(lat,lon f64), radius×10 i16, aircraft(lat,lon f64), height×10 i16, mode i8}` **mode 1=Orbit, 2=Spiral** | `a5/d1.java:99` (Surround, mode 1); `a5/s0.java:117` (Screw, mode 2) |
| Set orbit speed/direction | `D`→`l0(70,i)` | `3/70` | `{3,70,0,0, i16LE,0,0}` | `a5/d1.java:127`, `a5/s0.java:193` |
| Orbit rotation-dir flag | `V1`→`j0(72,i)` | `3/72` | `{3,72,0,0,i8,0,0,0}` | `a5/d1.java:118` |
| Query orbit/spiral center (`r8.m0`) | `J`→`m()` | `3/69` | `{3,69}` | `a5/d1.java:152` |
| Query orbit/spiral speed (`r8.i`) | `k`→`n()` | `3/71` | `{3,71}` | `a5/d1.java:161` |
| Orbit exit | `c`→`i0(67)` | `3/67` | `{3,67}` | `z4/h0.java:429`, `x8_ai_fly_surround_eixte` |
| Spiral set params (`r8.h`: pitch×10, angle, toggle, mode=3) | `S1`→`Q0(h)` | `3/103` | `{3,103,0,0, i16LE, b, b, b}` | `a5/s0.java:186` |
| Spiral query params | `g0`→`I()` | `3/104` | `{3,104,0,0}` | `a5/s0.java:221` |
| Spiral set-circle confirm | `f0`→`R0()` | `3/99` | `{3,99,0,0}` | `a5/s0.java:126` |
| Spiral exit | `H1`→`P0()` | `3/102` | `{3,102,0,0}` | `z4/h0.java:420`, `x8_ai_fly_screw_exte` |
| **Follow start (lock target)** | `Y`→`U(80)` | `3/80` | `{3,80}` | `a5/p.java:59` (AiFollow) |
| Follow sub-mode (normal/parallel/lock) | `X`→`W(i)` | `3/85` | `{3,85,0,0,i8}` | `a5/p.java:68` |
| Follow speed | `R1`→`Z(i)` | `3/88` | `{3,88,0,0, i16LE}` | `a5/p.java:77` |
| Follow VCM-mode | `l2`→`Z0(i)` | `3/107` | `{3,107,0,0,i8}` (no-ack) | `a5/p.java:47` |
| Query follow status (`r8.i0`) | `e`→`i()` | `3/86` | `{3,86,0,0}` | `a5/p.java:226` |
| Query follow target-box (`r8.f`) | `X0`→`j()` | `3/89` | `{3,89,0,0}` | `a5/p.java:235` |
| Follow exit | `V`→`a0(97)` then `N1`→`U(83)` | `3/97` then `3/83` | `{3,97}`,`{3,83}` | `z4/h0.java:347,353` |

Cross-mode toggles (opcodes certain, labels ⚠): `a0(96)`=`3/96` / `a0(97)`=`3/97` — paired
ON/OFF bound to the per-mode action button (not-sel→96, sel→97), likely **AI auto-record /
run toggle** (`a5/d1.java:76,238`, `a5/s0.java:103,315`, `a5/g.java:103`, `a5/d0.java:500`);
`a0(98)`=`3/98` — sent on a map gesture, likely **clear/cancel AI target** (`a5/d1.java:136`,
`a5/p.java:86`).

**Unused / legacy (no UI caller found):** `E0()`=`3/90` (`s(float,lat,lon,int,float)`,
`:155`); `O()`=`3/<msgId>` OneKeyVideo (`z8.c.q2`, *OneKeyVideoParam*: cmd,cmdParam,distance,
speed,direction,msgId; `:310`). Present in encoder+presenter but no caller — treat as legacy.

### 2.3 Realtime gimbal pitch — `m(int pitch, int rate)` — opcode `9/6` — VERIFIED

> **CORRECTION to the task brief.** The realtime gimbal pitch command is **NOT** `d1`
> (cmd21/0x81). It is `j8/i.java` **X8GimbalCollection.m(pitch, rate)**, target
> **MODULE_GIMBAL** (8), opcode `9/6`. `d1`/cmd21/0x81 is the uBlox **AGPS upload** handshake
> (see §2.3a). Verified: `d6/g0.java` **GimbalRuler** calls `q8.k.f().n(pitch, rate)`
> (`:153` hold rate=20000, `:169` tap rate=1000, log "调整云台"=adjust gimbal) →
> `z8/j.java:21` `K`→`j8.i.m(i10,i11)` (`j8/i.java:119`).

17-byte payload:
```
off0  = 9     # cmdset (MODULE_GIMBAL)
off1  = 6     # cmdid
off2,3= 0,0
off4  = 10    # constant (0x0A — axis/sub-selector)
off5,6= 0,0
off7,8   = rate   (i16 LE)   # slew rate: 1000 (tap/step) or 20000 (hold-to-limit)
off9..12 = 0
off13,14 = pitch  (i16 LE)   # target pitch angle
off15,16 = 0,0
```
Public API `q8.k.f().n(int pitch, int rate, c7.c)` (`q8/k.java:72`). ⚠ pitch units
inferred **centidegrees** (ruler spans roughly −90°…+10°); confirm against telemetry.

### 2.3a uBlox AGPS upload — `d1`/`c1` (cmdset 21, MODULE_FC)
`z8.c.e0(blobLen, packetCount)` → `d1(len,(short)count)` opcode **`21/0x81`** begins the
AGPS data upload; the 128-byte data chunks follow via `z8.c.x2(i,data)` → `c1` opcode
**`21/0x82`**. Only caller: **UBloxGpsManager** (`l5/f.java:331` begin, `:314` chunks;
downloads from `agps.u-blox.com:46434`). `d1` payload (12 B): `{21,0x81,0,0, len i32LE@4,
count i16LE@8, 0,0}`; `c1` payload: `{21,0x82,0,0, seq i16LE, dataLen i16LE, data}`.
Other cmdset-21 FC commands: `z()` `21/0x85` (`:1024`); `R()` `21/0x80` (RTC, `n7.l` date,
`:356`); `y(int)` `1/21` `{1,21,i8}` (`:1010`).

### 2.4 Gimbal — module commands (`j8/i.java` X8GimbalCollection) — VERIFIED
Target **MODULE_GIMBAL** (8), source GCS, cmdset = 9. Manager = **X8GimbalManager**
(`q8.k.f()`) → presenter `z8/j.java` (X8GimbalPresenter) → `j8/i.java`.

| Human name | Mgr `q8.k` | Encoder | Opcode | Payload | Evidence |
|---|---|---|---|---|---|
| **Realtime pitch (tilt)** | `n(pitch,rate)` | `m(i,i)` | `9/6` | see §2.3 | `d6/g0.java` GimbalRuler |
| Get pitch-speed setting | `g()` | `g()` | `9/41` | `{9,41,0,0}` | `h6/e5.java:69` SettingGimbal |
| Set pitch-speed | `o(v)` | `n(i)` | `9/40` | `{9,40,0,0,i8}` | `h6/e5.java:188` |
| Restore/reset gimbal params | `h()` | `h()` | `9/47` | `{9,47}` | `h6/e5.java:184`, `x8_gimbal_setting_gimbal_reset_params` |
| Get gimbal gain % | `a()` | `a()` | `9/31` | `{9,31}` | `h6/k1.java:158` |
| Set gimbal gain % (1..200) | `j(v)` | `i(i)` | `9/30` (GATQ) | `{9,30,0,0,i8}` | `h6/k1.java:189` |
| Get P/R/Y fine-tune offsets (0.1°) | `c()` | `d()` | `9/29` (GAT) | `{9,29}` | `h6/k2.java:150` GimbalXYZAdjust |
| Live/cancel/apply/save P/R/Y fine-tune (0.1°, ±10°) | `l(sel,r,p,y)` | `k(i,f,f,f)` | `9/28` (TOUCH) | `{9,28,0,0, sel, r×10,p×10,y×10 (u8)}` ; sel 0=begin,1=cancel,2=apply,3=save | `h6/k2.java:82,111,387` |
| Get gimbal sensor diag (poll 500ms) | `d()` | `f()` | `9/96` | `{9,96,0,0}` | `h6/s1.java:168` |
| **Start gimbal calibration** (module path) | `e()` | `e()` | `9/51` | `{9,51,0,0}` (+60s) | `h6/r.java:546` |
| Gimbal cal state machine (idle/abort/restart) | `m(cmd)` | `l(i)` | `9/50` | `{9,50,0,0,i8}` (+5s) | `h6/r.java:889,950` |
| Factory 3-axis adjust: set P(2)/R(4)/Y(8)/Save(1) | `k(sel,f)` | `j(i,f)` | `9/105` | `{9,105,0,0,i8, f32LE}` | `f5/s.java:158` |
| Factory 3-axis adjust: read offsets | `b()` | `c()` | `9/106` | `{9,106}` | `f5/s.java:138` |

**Gimbal calibration via the FC path** (`q8.d.u()` → `z8.b` → `j8.c`): start/stop =
`r0(i)` `9/44` (COMMA) `{9,44,0,0,i8}` (0=start/1=abort, `h6/r1.java:451,567`
GimbalCalibrationFragment); status poll = `a()` `9/45` `{9,45,0,0}` (`h6/r1.java:148`).

> ⚠ **No gimbal recenter/reset-to-forward opcode and no follow/FPV/lock *attitude-mode*
> opcode exist in cmdset 9.** (`9/44`,`9/45` are calibration, not recenter — correcting the
> task brief.) The "Follow mode" in the UI is AI subject-tracking (a different subsystem,
> §2.2.4), not a gimbal attitude mode. The closest to "reset" is restore-params `9/47`.

### 2.5 Camera (`j8/a.java` CameraCollection)
Target **MODULE_CAMERA** (3), source GCS, cmdset = **2**. (`e()` helper `j8/a.java:34`.)

| Encoder | Opcode `2/cmdid` | Payload | File:line |
|---|---|---|---|
| `y()` | `2/2` | `{2,2}` | `j8/a.java:264` |
| `z()` | `2/3` | `{2,3}` | `:271` |
| `B()` | `2/4` | `{2,4}` | `:64` |
| `A()` | `2/5` | `{2,5}` | `:57` |
| `d()` | `2/9` | `{2,9}` | `:128` |
| `h(byte b)` | `2/b` | `{2,b,0,0}` | `:150` |
| `p(byte key,byte idx)` | `2/key` | `{2,key,0,0,idx}` (generic param set) | `:199` |
| `q(byte,int,int)` | `2/71` | `{2,71,0,0,b,i8,i8}` | `:207` |
| `r(byte)` | `2/105` | `{2,105,0,0,b}` | `:214` |
| `n(byte)` | `2/107` | `{2,107,0,0,b}` | `:185` |
| `u(int)` | `2/109` | `{2,109,0,0,i8}` | `:236` |
| `x(int)` | `2/0x84` | `{2,-124,0,0,i8}` | `:257` |
| `w(int)` | `2/0xE1` | `{2,-31,0,0,i8}` | `:250` |
| `t(int)` | `2/0xE0` | `{2,-32,0,0, i32 LE}` | `:229` |
| `c(b,b,s,s,i,i)` | `2/0xE2` | `{2,-30,0,0,b,b, s16LE, s16LE, i32LE, i32LE}` | `:121` |
| `k()` | `2/0xE3` | `{2,-29,0,0}` | `:164` |
| `o(bool)` | `2/58` (COLON) | `{2,58,0,0, 0/1}` | `:192` |
| `C(bool)` | `2/56` | `{2,56,0,0, 0/1}` | `:71` |
| `D(b,b,b)` | `2/112` | `{2,112,0,0,b,b,b}` | `:80` |
| `s(i,i,i)` | `2/114` | `{2,114,0,0, 3×i32LE}` | `:221` |
| `v(b,b)` | `2/6` | `{2,6,0,0,b,-1,b}` | `:243` |
| `g()` | `2/108` | `{2,108,0,0}` | `:143` |
| `i()` | `2/72` | `{2,72,0,0}` | `:157` |
| `l()` | `2/106` | `{2,106,0,0}` | `:171` |
| `m()` | `2/49` | `{2,49,0,0}` | `:178` |
| `f()` | `2/64` | `{2,64,0,0}` | `:135` |
| `b(byte[16])` | `2/111` | `{2,111,0,0, 16B}` | `:104` |
| `a()` | `2/0x87` | `{2,-121,0,0, sec,min,hr,day,mon,yrLE, tzOffset i32LE}` (set camera clock) | `:87` |

Value tables in `j8/a.java`: EV steps `f21209e` ("-3.0"…"+3.0"), EV(video) `f21210f`,
ISO `f21211g` {Auto,100,200,400,800,1600,3200}. Take-photo / start-record / stop-record /
photo-vs-video mode map to the small cmdids (2,3,4,5,9) — verified names in **§2.5a**.

### 2.5a Verified camera names
Manager = **CameraManager** (`q8.b.k()`) → `z8/a.java` → `j8/a.java`. Most settings use the
generic setter `q8.b.o(key,index)` → `j8.a.p(key,index)` → opcode `2/key` (log
`setCameraKeyParam`). Capture is dispatched by camera-status in **MainRightFragment**
(`d6/e3.java`).

| Human name | Mgr `q8.b` | Encoder | Opcode | Evidence |
|---|---|---|---|---|
| **Take photo** | `B()`→ | `B()` | `2/4` | `d6/e3.java:1372`, toast `x8_camera_take_success` |
| **Start record** | | `y()` | `2/2` | `d6/e3.java:1332`, log "点击了拍摄" |
| **Stop record** | `A()` | `z()` | `2/3` | `d6/e3.java:287` |
| Abort photo (in progress) | `z()` | `A()` | `2/5` | `d6/e3.java:324` |
| EV / exposure comp | `o(25,i)` | `p` | `2/25` | EV table `j8/a.java` `f21210f`; `d6/b4.java:148` |
| **ISO** | `o(29,i)` | `p` | `2/29` | `R.array.x8_iso_options`; `d6/b4.java:152` |
| **Shutter speed** | `o(27,i)` | `p` | `2/27` | `d6/b4.java:156` |
| White balance | `o(65,i)` | `p` | `2/65` | `h6/l7.java:65` SettingShoot |
| Colour/tone style | `o(67,i)` | `p` | `2/67` | `h6/l7.java:68` |
| Photo size | `o(85,i)` | `p` | `2/85` | `h6/l7.java:72` |
| Metering (avg/center/spot) | `p(71,…)` | `q`/`p` | `2/71` | `h6/l7.java:93`, `x8_meter_*` |
| Video codec | `o(75,i)` | `p` | `2/75` | `h6/l7.java:95` |
| Video quality | `o(52,i)` | `p` | `2/52` | `h6/l7.java:655` |
| Photo format (JPEG / +RAW) | `o(87,i)` | `p` | `2/87` | `h6/l7.java:673` |
| **Video resolution** / vertical | `o(23,i)` | `p` | `2/23` | `h6/p7.java:110` |
| Zoom/focus mode | `x(0/1)` | `x` | `2/132` (0x84) | `h6/l7.java:308` |
| HDR on/off | `n(bool)` | `n` | `2/58` | `h6/l7.java:313` |
| Format SD card | `d()` | `d` | `2/9` | `h6/l7.java:679` |
| Reset camera params | `l()` | `m` | `2/49` | `h6/l7.java:110` |
| **Capture/record mode (photo↔video)** | `o(101,i)` | `p` | `2/101` | `d6/s3.java:131`, log "setRecordMode" |
| Mode sub-cmd | `o(103,i)` | `p` | `2/103` | `d6/s3.java:122` |
| Panorama mode | `o(91,i)` | `p` | `2/91` | `d6/s3.java:99` |
| Mode first-step | `u(i)` | `u` | `2/109` | `d6/s3.java:94` |
| **Manual focus** | `m(i)` | `r` | `2/107` | `IndicatorVerticalSeekBar.java:112`, log "相焦设置成功" |
| Set date/time+TZ (on connect) | `a()` | `a` | `2/135` (0x87) | `m8/a.java:263` |
| Camera auth/license handshake | `b(byte[16])` | `b` | `2/111` | `k6/h.java:142`, log "授权成功" |
| Auto/manual exposure toggle ⚠ | `o(99,…)` | `p` | `2/99` | `d6/e3.java:1040` (key-99 label unconfirmed) |
| Enable/disable camera session ⚠ | `C(bool)` | `C` | `2/56` | `X8mMainActivity.java:779` (inferred) |

(`o(77,…)`=`2/77` exists in `k6/n0.java:68` with no caller — unused.)

### 2.6 Device activation handshake — `S0(int state, byte[14] challenge)` — opcode `1/23`
`j8/c.java:378`. Target MODULE_FC, cmdset `1`, cmdid `23` (0x17, QUITQ). Crypto
(`n7/a.java` AES, `n7/a.java:19` = `AES/ECB/PKCS5Padding`, encrypt mode):

1. `state` (1..4) selects a status string: `1=UNACTIVATED 2=ACTIVATED 3=LOCKED 4=DISPOSABLE`
   (else `""`). (`j8/c.java:380-396`.)
2. plaintext = that ASCII string copied into a **16-byte** zero-padded buffer.
3. key = **16 bytes** = `{0x91, 0x87, challenge[0..13]}` — `0x91,0x87` then the 14-byte
   challenge supplied by the drone's activation-status telemetry. (`:400-403`.)
4. `cipher = AES-ECB(plaintext16, key16)` → PKCS5 makes 32 bytes; **only the first 16-byte
   block is used** (`System.arraycopy(a10, 0, bArr4, 4, 16)`, `:411`). Effectively
   AES-ECB(single block, no padding).
5. payload (20 B): `{1, 23, 0, 0, cipher[0..15]}`. (`:406-412`.)

Public API `z8.c.i0(int,byte[],c7.c)` (`z8/c.java:264`).

---

## 3. Full command enumeration

### 3.1 FcCollection `j8/c.java` — complete
Target `MODULE_FC` (2) unless the **Tgt** column says otherwise. "cmd" = `cmdset/cmdid`.
"ack" column: `Y`=u()/ack-required, `N`=v()/w() no-ack. Presenter path abbreviations:
`C`=FcCtrlManager(`q8.d.u()`)→z8.b, `M`=FcManager(`q8.e.a()`)→z8.c.

| Enc | Tgt | cmd | Payload | ack | via | Line |
|---|---|---|---|---|---|---|
| `A(i)` | FC | 12/7 | `{12,7,0,0,i8}` | Y | C.t | 72 |
| `A0(b,f)` | FC | 4/5 | `{4,5,0,0,b, f32LE}` | Y | C.Y/Z/c0 (idx 7/5/3) | 79 |
| `B()` | FC | 4/19 | `{4,19,0,0}` | Y | C.o | 90 |
| `B0(i,i)` | FC | 1/12 | `{1,12,0,0,i8,i8}` | Y | C.a0 | 97 |
| `C()` | FC | 4/13 | `{4,13,0,0}` | Y | C.b2 | 104 |
| `C0(i,i)` | FC | 4/25 | `{4,25,0,0,3,i8,i8,0,0}` | Y | C.b0 | 111 |
| `D()` | FC | 4/23 | `{4,23,0,0}` | Y | C.w | 118 |
| `D0(GpsInfoCmd)` | FC | 8/5 | `{8,5,0,0, lon f64@4, lat f64@12, alt f32@20, hAcc@24, vAcc@25, speed f32@26, bearing i16@30}` | N | M.C | 125 |
| `E()` | **NFZ** | 17/2 | `{17,2,0,0}` | Y | M.e1 | 148 |
| `E0(f,d,d,i,f)` | FC | 3/90 | see §2.2.4 | Y | M.s | 155 |
| `F()` | FC | 4/2 | `{4,2,0,0}` | Y | C.x | 175 |
| `F0(i)` | FC | 4/18 | `{4,18,0,0, i16LE}` | Y | C.T | 182 |
| `G()` | FC | 4/9 | `{4,9,0,0}` | Y | C.z(list) | 189 |
| `G0(i)` | FC | 4/25 | `{4,25,0,0,4,0,0,i8,0}` | Y | C.o0 | 196 |
| `H()` | FC | 4/26 | `{4,26,0,0}` | Y | C.A | 203 |
| `H0(i)` | FC | 4/54 | `{4,54,0,0,i8}` | Y | — | 210 |
| `I()` | FC | 3/104 | `{3,104,0,0}` | Y | M.g0 | 217 |
| `I0(b)` | FC | 4/12 | `{4,12,0,0,b}` | Y | C.e0 | 224 |
| `J()` | FC | 4/38 | `{4,38,0,0}` | Y | C.B | 231 |
| `J0(i,i,i,i)` | FC | 4/24 | `{4,24,0,0,i8,i8,i8,i8}` | Y | C.f0 | 238 |
| `K()` | FC | 1/22 | `{1,22,0,0}` | Y | M.F1 | 245 |
| `K0(b)` | FC | 3/b | `{3,b,0,0}` | Y | C.g0 | 252 |
| `L()` | FC | 4/4 | `{4,4,0,0}` | Y | C.C | 259 |
| `L0(i)` | FC | 3/105 | `{3,105,0,0,i8}` | Y | C.h0 | 266 |
| `M()` | FC | 4/34 | `{4,34,0,0}` | Y | C.D | 273 |
| `M0(b)` | FC | 4/1 | `{4,1,0,0,b}` | Y | C.i0 | 280 |
| `N(b)` | FC | 3/b | `{3,b,0,0}` | Y | M.N(21)/m2(24) | 287 |
| `N0(f,f)` | FC | 8/6 | `{8,6,0,0, f32LE, f32LE}` | N | M.B | 294 |
| `O(…)` | FC | 3/i15 | OneKeyVideo, `{3,i15,0,0,i8,i8, i16LE, i16LE, i8}` (+1s) | Y | M.q2 | 310 |
| `O0(f)` | FC | 4/8 | `{4,8,0,0, f32LE}` | Y | C.k0 | 318 |
| `P()` | FC | 4/63 | `{4,63,0,0}` | Y | C.G | 328 |
| `P0()` | FC | 3/102 | `{3,102,0,0}` | Y | M.H1 | 335 |
| `Q(i)` | **RC** | 11/14 | `{11,14,0,0,i8}` | Y | C.H | 342 |
| `Q0(r8.h)` | FC | 3/103 | `{3,103,0,0, i16LE, b, b, b}` | Y | M.S1 | 349 |
| `R()` | FC | 21/0x80 | `{21,0x80,0,0, yrLE, mon, day, hr, min, sec, 0}` (RTC) | Y | M.y1 | 356 |
| `R0()` | FC | 3/99 | `{3,99,0,0}` | Y | M.f0 | 364 |
| `S()` | FC | 4/0x89 | `{4,0x89,0,0,1}` | Y | C.I | 371 |
| `S0(i,b[])` | FC | 1/23 | activation, see §2.6 | Y | M.i0 | 378 |
| `T(i)` | FC | 4/(51 or 52) | `{4, i==1?51:52, 0,0}` | Y | C.E(1)/d(0) | 417 |
| `T0(i)` | FC | 4/3 | `{4,3,0,0,7,i8}` | Y | C.l0 | 433 |
| `U(i)` | FC | 3/i | `{3,i8}` | Y | M.G/Y/z/N1 | 440 |
| `U0()` | FC | 8/4 | `{8,4,0,0, yrLE, mon, day, hr, min, sec}` (RTC) | N | M.f1 | 447 |
| `V(i)` | FC | 4/10 | `{4,10,0,0,i8}` | Y | C.J | 461 |
| `V0(i)` | FC | 4/25 | `{4,25,0,0,8,0,0,0,i8}` | Y | C.m0 | 468 |
| `W(i)` | FC | 3/85 | `{3,85,0,0,i8}` | Y | M.X | 475 |
| `W0()` | FC | 4/50 | `{4,50,0,0}` | Y | C.n0 | 482 |
| `X(d,d,i,i)` | FC | 3/52 | see §2.2.4 (25 B) | Y | M.G0 | 489 |
| `X0(i)` | FC | 4/37 | `{4,37,0,0,4,0,0,i8,0}` | Y | C.o0→? (d.o0) | 507 |
| `Y(b)` | FC | 3/b | `{3,b}` | Y | M.I2(51)/m0(53)/x0(48) | 514 |
| `Y0(i)` | FC | 4/33 | `{4,33,0,0,4,0,0,i8,0}` | Y | C.p0 | 521 |
| `Z(i)` | FC | 3/88 | `{3,88,0,0, i16LE}` | Y | M.R1 | 528 |
| `Z0(i)` | FC | 3/107 | `{3,107,0,0,i8}` | N | M.l2 | 535 |
| `a()` | **GIMBAL** | 9/45 | `{9,45,0,0}` | Y | C.a | 543 |
| `a0(i)` | FC | 3/i | `{3,i8}` (+10ms) | Y | M.V(97)/Z(98)/o2(96) | 550 |
| `a1(b)` | FC | 3/b | `{3,b,0,0}` | Y | M.g2(16)/s1(19) | 558 |
| `b(i)` | FC | 13/8 | `{13,8,0,0,i8}` | Y | C.b | 565 |
| `b0()` | FC | 3/32 | `{3,32}` | Y | M.E1 | 572 |
| `b1(b,str,str)` | **REPEATER_VEHICLE** | 14/7 | `{14,7,0,0,b, str1[40], str2[20]}` (WiFi cfg) | Y(+500ms) | M.B1 | 579 |
| `c()` | FC | 13/7 | `{13,7}` | Y | C.b2 | 602 |
| `c0()` | FC | 3/35 | `{3,35}` | Y | M.u2 | 609 |
| `c1(s,b[])` | FC | 21/0x82 | uBlox AGPS data chunk `{21,0x82,0,0, seq16LE, len16LE, data}` | Y | M.x2 | 616 |
| `d()` | **RC** | 11/15 | `{11,15,0,0}` | Y | C.c | 634 |
| `d0()` | FC | 3/33 | `{3,33}` | Y | M.v0 | 641 |
| `d1(i,s)` | FC | 21/0x81 | **uBlox AGPS upload begin**, see §2.3a (NOT gimbal) | Y | M.e0 | 648 |
| `e()` | FC | 4/64 | `{4,64,0,0}` | Y | C.e | 655 |
| `e0(s8.b)` | FC | 3/36 | waypoint upload, see §2.2.1 | Y | M.L0 | 662 |
| `e1(s,s,s,s,i)` | FC (src **RC**) | 11/2 | virtual stick, see §3.1a | N | M.L | 705 |
| `f(bool)` | FC | 4/59 | `{4,59,0,0,0/1}` | Y | C.g | 721 |
| `f0(s8.c)` | FC | 3/37 | point-action, see §2.2.2 | Y | M.D0 | 728 |
| `g()` | FC | 4/53 | `{4,53,0,0}` | Y | C.h | 749 |
| `g0()` | FC | 3/34 | `{3,34}` | Y | M.b0 | 756 |
| `h()` | FC | 4/11 | `{4,11,0,0}` | Y | C.i | 763 |
| `h0(i)` | FC | 3/i | `{3,i8}` | Y | M.H(29)/f(26) | 770 |
| `i()` | FC | 3/86 | `{3,86,0,0}` | Y | M.e | 777 |
| `i0(b)` | FC | 3/b | `{3,b}` | Y | M.M(64)/c(67) | 784 |
| `j()` | FC | 3/89 | `{3,89,0,0}` | Y | M.X0 | 791 |
| `j0(b,i)` | FC | 3/b | `{3,b,0,0,i8,0,0,0}` | Y | M.V1(72) | 798 |
| `k(i)` | FC | 3/38 | `{3,38,0,0,i8}` | Y | M.j2 | 805 |
| `k0(i,…)` | FC | 3/i | orbit?, see §2.2.4 (44 B) | Y | M.Q(68) | 812 |
| `l(i)` | FC | 3/39 | `{3,39,0,0,i8}` | Y | M.b | 837 |
| `l0(b,i)` | FC | 3/b | `{3,b,0,0, i16LE,0,0}` | Y | M.D(70) | 844 |
| `m()` | FC | 3/69 | `{3,69}` | Y | M.J | 851 |
| `m0(i,i,i)` | FC | 13/5 | `{13,5,0,0,i8,i8,i8}` (+5s) | Y | C.K | 858 |
| `n()` | FC | 3/71 | `{3,71}` | Y | M.k | 866 |
| `n0(i,i)` | FC | 4/37 | `{4,37,0,0,3,i8,i8,0,0}` | Y | C.L | 873 |
| `o(i,i)` | FC | 13/6 | `{13,6,0,0,i8,i8}` (+60s,+10) | N | C.j | 880 |
| `o0(i)` | FC | 4/39 | `{4,39,0,0,i8}` | Y | C.M | 889 |
| `p()` | FC | 4/40 | `{4,40,0,0}` | Y | C.k | 896 |
| `p0(i,i)` | FC | 4/35 | `{4,35,0,0,3,i8,i8,0,0}` | Y | C.N | 903 |
| `q()` | FC | 4/36 | `{4,36,0,0}` | Y | C.l | 910 |
| `q0(i,i,i)` | FC | 13/5 | `{13,5,0,0,i8,i8,i8}` | Y | C.O | 917 |
| `r(i,i)` | FC | 13/6 | `{13,6,0,0,i8,i8}` | Y | C.m | 924 |
| `r0(i)` | **GIMBAL** | 9/44 | `{9,44,0,0,i8}` | Y | C.f | 931 |
| `s()` | FC | 4/60 | `{4,60,0,0}` | Y | C.n | 938 |
| `s0(b)` | **RC** | 11/17 | `{11,17,0,0,b}` | Y | C.j0 | 945 |
| `t()` | **RC** | 11/18 | `{11,18,0,0}` | Y | C.y | 952 |
| `t0()` | FC | 4/48 | `{4,48,0,0}` | Y | C.Q | 959 |
| `u0()` | FC | 4/46 | `{4,46,0,0}` | Y | C.R | 966 |
| `v0(i,i)` | FC | 4/3 | `{4,3,0,0,i8,i8}` | Y | C.S | 973 |
| `w0(i)` | FC | 4/(43 or 44) | `{4, i==1?43:44, 0,0}` | Y | C.U | 980 |
| `x(b)` | FC | 4/6 | `{4,6,0,0,b}` | Y | C.W0(3)/Y1(5)/c2(7) | 996 |
| `x0()` | FC | 4/47 | `{4,47,0,0}` | Y | C.V | 1003 |
| `y(i)` | FC | 1/21 | `{1,21,i8}` | Y | C.r | 1010 |
| `y0()` | FC | 4/45 | `{4,45,0,0}` | Y | C.W | 1017 |
| `z()` | FC | 21/0x85 | `{21,0x85,0,0}` | Y | M.O0 | 1024 |
| `z0(i)` | FC | 4/(41 or 42) | `{4, i==1?41:42, 0,0}` | Y | C.X | 1031 |

#### 3.1a Virtual stick — `e1(s roll,s pitch,s thr,s yaw,int)` — opcode `11/2`
`j8/c.java:705`. **Source module = RC (13)**, target FC, **no-ack**. Payload:
```
{11, 2, 0, 0, s0 i16LE, s1 i16LE, s2 i16LE, s3 i16LE, 0, 2, 0, 2, 30(0x1E), 31}
```
4 joystick channels (`s10..s13`), trailing constants `00 02 00 02 1E 1F`. The 5th arg `int`
is appended to a side buffer `h8.a.G().K(...)` (local stick state), not the wire payload.
Channel order (roll/pitch/throttle/yaw) and range ⚠ to confirm (see §4).

### 3.2 X8GimbalCollection `j8/i.java` — see §2.4 (all cmdset 9, target GIMBAL).
### 3.3 CameraCollection `j8/a.java` — see §2.5 (all cmdset 2, target CAMERA).

### 3.4 X8MediaCollection `j8/j.java`
Target **MODULE_CAMERA** (3), source GCS, cmdset = **7**.

| Enc | cmd | Payload | Line |
|---|---|---|---|
| `a(str)` | 7/8 | `{7,8,0,0, str bytes}` | `j8/j.java:35` |
| `b()` | 7/9 | `{7,9,0,0}` | `:49` |
| `d()` | 7/6 | `{7,6,0,0}` | `:56` |
| `e(str)` | 7/1 | `{7,1,0,0, str bytes}` (+timeout) | `:63` |
| `f(s,i,i)` | 7/2 | `{7,2,0,0, s16LE, i32LE, i32LE}` | `:79` |
| `g(s,i)` | 7/4 | `{7,4,0,0, s16LE, i32LE}` | `:99` |
| `h(s)` | 7/3 | `{7,3,0,0, s16LE}` | `:109` |

### 3.5 X8VisionCollection `j8/l.java` (tracking / CV)
Target **MODULE_CV** (10), source GCS, cmdset = **15** (PREPEND).

| Enc | cmd | Payload | Line |
|---|---|---|---|
| `b(i)` | 15/16 | `{15,16,0,0,i8}` | `j8/l.java:34` |
| `c(i,i,i,i,i)` | 15/3 | `{15,3,0,0, 5× i16LE}` (bounding box) | `:41` |

### 3.6 RcCollection `j8/f.java`
Target **MODULE_RC** (13), source GCS. `a(byte)` → opcode `14/46`, payload
`{14,46,0,0,b}`, no-ack (`j8/f.java:35`).

### 3.7 RcRelayCollection `j8/g.java`
Target **MODULE_REPEATER_RC** (16), source GCS, cmdset 14.
`b()` → `14/9` `{14,9,0,0}` (`:29`); `c(bool)` → `14/16` `{14,16,0,0,0/1}` (`:36`).

### 3.8 FcRelayCollection `j8/d.java`
Target **MODULE_REPEATER_VEHICLE** (14), source GCS, cmdset 14.

| Enc | cmd | Payload | Line |
|---|---|---|---|
| `a(i,i,i)` | 14/39 | `{14,39,0,0,i8,i8,i8}` | `j8/d.java:32` |
| `b()` | 14/40 | `{14,40,0,0}` (+1s) | `:40` |
| `c()` | 14/52 | `{14,52,0,0}` | `:48` |
| `e()` | 14/51 | `{14,51,0,0}` | `:55` |
| `f(b,b)` | 14/48 | `{14,48,0,0,b,b}` | `:62` |
| `g(b)` | 14/49 | `{14,49,0,0,b}` | `:69` |
| `h(i)` | 14/50 | `{14,50,0,0, i32LE}` | `:76` |

### 3.9 DynamicNFZCollection `j8/b.java`
Target **MODULE_NFZ** (17), source GCS, cmdset 17.

| Enc | cmd | Payload | Line |
|---|---|---|---|
| `b(s,s,b,b,str)` | 17/0x80 | `{17,0x80,0,0, s16LE, s16LE, b, b, i64LE(parseLong str)}` | `j8/b.java:36` |
| `c(s,s)` | 17/0x81 | `{17,0x81,0,0, s16LE, s16LE}` | `:58` |
| `d(s,str)` | 17/0x82 | `{17,0x82,0,0, s16LE, hex(str) bytes}` | `:71` |

### 3.10 FwUpdateCollection `j8/e.java`
cmdset = **16** (STAT) for control; target module varies (RC/FC/CAMERA/REPEATER_RC).
Data-chunk uploaders `q()/r()/t()` build a non-standard frame (`{4, seqLE, …}` via
`k.C(bytes, channel)` with channels 6/13) — see the firmware document for the full
update sequence. Control opcodes:

| Enc | Tgt | cmd | Payload | Line |
|---|---|---|---|---|
| `a()` | RC | 16/0xC8 | `{16,0xC8,0,0}` | `j8/e.java:34` |
| `b()` | FC | 16/0xB2 | `{16,0xB2,0,0,0,20}` | `:42` |
| `d(b,b)` | *b* | 16/0xB1 | `{16,0xB1,0,0,b11,0,0,0,1,-1,2,0}` (+2s) | `:49` |
| `e()` | RC | 16/0xC5 | `{16,0xC5,0,0}` | `:58` |
| `f()` | CAMERA | 16/1 | `{16,1,0,0}` | `:66` |
| `g()` | CAMERA | 16/8 | `{16,8,0,0}` | `:73` |
| `h()` | CAMERA | 16/6 | `{16,6,0,0}` | `:80` |
| `i()` | REPEATER_RC | 16/6 | `{16,6,0,0}` | `:88` |
| `j()` | RC | 16/0xC7 | `{16,0xC7,0,0}` | `:96` |
| `k()` | RC | 16/0xC3 | `{16,0xC3,0,0}` | `:104` |
| `l()` | CAMERA | 16/2 | `{16,2,0,0}` | `:112` |
| `m()` | REPEATER_RC | 16/2 | `{16,2,0,0}` (+A1) | `:120` |
| `n(b[],b[])` | CAMERA | 16/3 | `{16,3,0,0, bArr@4, bArr2@8}` | `:129` |
| `o(b[],b[])` | REPEATER_RC | 16/3 | `{16,3,0,0, …}` | `:143` |
| `p()` | RC | 16/0xC9 | `{16,0xC9,0,0}` | `:159` |
| `s(i,b[],n)` | RC | 16/0xC6 | `{16,0xC6,0,0, i16LE, data}` | `:218` |

---

## 4. Open questions / flagged uncertainties

**Resolved during this pass** (names now verified from UI callers):
- Flight actions §2.1 (takeoff `3/16`, cancel-takeoff `3/19`, land `3/21`, cancel-land
  `3/24`, RTH `3/26`, cancel-RTH `3/29`). **No emergency-stop/kill-motor command exists.**
- Mission lifecycle §2.2.3 (upload `3/36`/`3/37`, execute `3/32`, exit `3/35`, get-points
  `3/38`/`3/39`). **No waypoint pause/resume opcode exists** — only exit.
- Realtime gimbal pitch = `9/6` (§2.3); `d1`/`21/0x81` is **AGPS upload**, not gimbal.
- Gimbal `9/44`/`9/45` are **calibration**, not recenter; no recenter/attitude-mode opcode.
- `E0` (`3/90`) and `O` (OneKeyVideo) have **no caller** — legacy/unused.

**Still open:**
1. **Units** — now settled from the route-upload caller (`a5/d0.java` `T()`): altitude and
   POI altitude are **decimetres** (`(int)alt * 10`), speed is **decimetres/s** (route m/s × 10),
   yaw is whole degrees × 100, gimbal pitch is passed through as degrees × 100. Coordinates are
   WGS-84 unless the app is using its AMap (China) provider, which converts first.
   - Waypoint `altitude`/`POI altitude` i16 (`e0` off 24/56) — decimetres.
   - Waypoint `gimbalPitch` i16 (`e0` off 28) and **realtime gimbal pitch** (`m`, off 13–14)
     — inferred **centidegrees**; confirm.
   - Waypoint `speed` u8 (`e0` off 30) — raw; unit per route DB.
   - Yaw (`e0` off 26) = `angle × 100` → **centidegrees** (certain from code).
2. **Scaling site varies**: orbit `k0` scales radius/height **×10 in the encoder**; tap-to-fly
   `X` passes alt/speed pre-scaled by the **caller** (`a5/g.java`). Verify the X() scale.
3. **Coordinate pair order**: waypoint `e0` proven (lon@8, lat@16). Within `X()`/`k0()` coord
   pairs the lat-vs-lon order is from the caller and not byte-proven here.
4. **Virtual stick (`e1`, `11/2`)** channel order (roll/pitch/throttle/yaw?) and value range
   not proven from the encoder alone.
5. **`s8.c` point-action fields** (`f28661a..g`, §2.2.2) have no recovered names — positional.
6. **Cross-mode toggles** `a0(96/97)`=`3/96`/`3/97` (likely AI auto-record on/off) and
   `a0(98)`=`3/98` (likely clear AI target) — semantics inferred, opcodes certain.
7. **Unlabeled run-overlay pair** `d0()`=`3/33`, `g0()`=`3/34` (`z4/h0.java:682,689`) — only
   possible route pause/resume, but no label confirms it (layout XML absent).
8. **Camera**: key `2/99` (auto/manual-exposure toggle) label unconfirmed; `C()`=`2/56`
   session enable/disable inferred; `N0(f,f)` (`8/6`) purpose unconfirmed.
9. Many no-arg `{3,cmdid}`/`{4,cmdid}` encoders are **queries** (get-X) matched by the reply
   dispatchers (`z8/c.java:151`, `z8/b.java:170`) — that mapping is telemetry scope.

> Verified-name sources: flight ops/settings/calibration from **LeftFragment** `d6/z0.java`,
> **LeftModel** `k6/b0.java`, **SettingFcFragment** `h6/p4.java`+`k6/m1.java`, calibration
> fragments `h6/k.java`/`h6/r.java`/`h6/r1.java`; mission from **AiLineCmdDataManager**
> `a5/d0.java` + **X8mAiflyExcuteController** `z4/h0.java` + per-mode `a5/{g,d1,s0,p}.java`;
> gimbal from **GimbalRuler** `d6/g0.java` + `h6/{e5,k1,k2,s1}.java`; camera from
> **MainRightFragment** `d6/e3.java` + `d6/{b4,s3}.java` + `h6/{l7,p7}.java`.
