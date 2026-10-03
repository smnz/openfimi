# openfimi — Receive Path: Telemetry & ACK Protocol (drone → app)

FIMI X8M Android app, reverse-engineered for a Linux reimplementation. Scope: the
**receive path** — telemetry (`Auto*`) and command-ack (`Ack*`) messages decoded by the
app from frames sent by the aircraft. Send path and the full frame/CRC/encryption layer
are specified elsewhere; this document only touches the header fields needed to demux.

Decompiled sources root:
a jadx decompile of `com.fimi.app.x8m` V1.1.43.20703 (not included in this repo) (obfuscated packages; original class name is in each file's
`compiled from:` comment). All `file:line` cites below are relative to that root.

**Everything here was read from actual decode-method bodies**, not inferred from field names.
Where a unit/meaning is not provable from code it is flagged **(UNCERTAIN)**.

---

## 1. Buffer read primitives — `a7/c.java` (orig `LinkPayLoad4.java`)

The payload is a `ByteBuffer` (`f802a`, 1024 B) with a running read cursor `f803b` (`a7/c.java:11,14`).
Decoders pull fields sequentially via these methods. **All multi-byte integers and floats are
little-endian** (low byte first):

| Method | Width | Returns | Endian | Notes | Cite |
|--------|-------|---------|--------|-------|------|
| `b()`  | 1 | `byte` (signed); callers usually `& 0xFF` | — | advances cursor +1 | `a7/c.java:20` |
| `n()`  | 2 | `short` (signed int16); callers may `& 0xFFFF` | **LE** | advances +2 | `a7/c.java:85` |
| `g()`  | 4 | `int` (int32) | **LE** | advances +4 | `a7/c.java:43` |
| `j()`  | 8 | `long` (int64) | **LE** | advances +8 | `a7/c.java:64` |
| `d()`  | 8 | `Double` (`longBitsToDouble(j())`) | **LE** | IEEE-754 f64, advances +8 | `a7/c.java:31` |
| `e()`  | 4 | `float` (`intBitsToFloat(g())`) | **LE** | IEEE-754 f32, advances +4 | `a7/c.java:35` |
| `c(byte[] dst)` | len | copies `dst.length` bytes | — | advances +len | `a7/c.java:26` |
| `q(int pos)` | — | sets cursor `f803b = pos` | — | **seek** | `a7/c.java:99` |

**Fixed-position accessors** (do NOT use/advance the cursor — read absolute payload offsets):
`f()` = payload byte[0] & 0xFF (`:39`); `k()` = payload byte[1] & 0xFF (`:70`);
`o()` = payload byte[2] & 0x0F (`:91`); `l()` = `((byte[3]&0xFF)<<4) | ((byte[2]>>4)&0x0F)` (`:74`).
These four back the payload-header fields (§4).

> Implementer note: `b()` returns a Java signed byte. Enumerations/counters should be read as
> unsigned (`& 0xFF`). `n()` is signed int16 — correct for attitude angles and relative altitude
> (can be negative); mask `& 0xFFFF` only where the code does (noted per field).

---

## 2. Frame wire format (header) — only the demux-relevant fields

Framing/CRC/encryption are the frame spec's job. The 16-byte header is parsed by the
`Parser4` byte-state-machine `a7/d.java:189` (`b(int)`), fields stored in `Header4` `a7/a.java`.
Layout (little-endian), confirmed from `Header4.f()` serializer `a7/a.java:85-112` and the parser:

| Off | Size | Field | Meaning | Cite |
|-----|------|-------|---------|------|
| 0 | 1 | StartFlag | const `0xFE` (`f780q = -2`) | `a7/a.java:12,87` |
| 1–2 | 2 (LE short) | Ver+Len | bits[0..4]=version(=4), bits[6..14]=**Len** (9-bit, total frame length incl. 16-B header) | `a7/a.java:88`, `a7/d.java:221-224` |
| 3 | 1 | Type/Encry | enc fields: bits0-1, bits2-4, bits5-7 (encryption/type flags) | `a7/d.java:228-237` |
| **4** | **1** | **SrcId** | **source module** (ordinal of `j8.h$a`, §5) — **selects decoder handler** | `a7/d.java:239-241`, `a7/a.java:95` |
| **5** | **1** | **DestId** | **dest module** — app only processes frames where DestId == `MODULE_GCS` (=7) | `a7/d.java:244-245`, `a7/a.java:96` |
| 6 | 1 | Reserve2 | | `a7/d.java:249` |
| 7 | 1 | Reserve3 | | `a7/d.java:254` |
| 8–9 | 2 (LE short) | Seq | frame sequence number | `a7/d.java:259-265` |
| 10–11 | 2 (LE) | CrcHeader | header CRC16 | `a7/d.java:268-275` |
| 12–15 | 4 (LE) | CrcFrame | whole-frame CRC32 | `a7/d.java:277-291` |
| 16.. | Len−16 | Payload | message (payload header + body) | `a7/d.java:294` |

`Header4.b()` = payload length = `Len − 16` (`a7/a.java:69`). `Header4.e()` = SrcId, `Header4.c()` = DestId,
`Header4.d()` = Seq.

---

## 3. Payload header (first 4 bytes of payload) — `X8BaseMessage.f()` `r8/n3.java:49-57`

Every message decoder calls `super.f(bVar)` first. It reads a **4-byte payload header** using the
fixed-position accessors, then **seeks the cursor to offset 4** so the body is read from there:

```
f28076a = header.SrcId          (= frame byte 4)        n3.java:50
f28077b = header.DestId         (= frame byte 5)        n3.java:51
version = payload[2] & 0x0F     (c().o())               n3.java:52
groupID = payload[0] & 0xFF     (c().f())               n3.java:53   ← cmd-set / "group"
msgId   = payload[1] & 0xFF     (c().k())               n3.java:54   ← command id
msgRpt  = (payload[3]<<4)|(payload[2]>>4)  12-bit (c().l())  n3.java:55
c().q(4)   // seek to payload offset 4; body decode starts here      n3.java:56
```

Payload byte map:

| Payload off | Field |
|-------------|-------|
| 0 | `groupID` (cmd-set) |
| 1 | `msgId` (cmd id) |
| 2 | low nibble = `version`; high nibble = low 4 bits of `msgRpt` |
| 3 | high 8 bits of `msgRpt` |
| 4.. | message body (all §6 offsets are relative to **here**) |

**`msgRpt`** is a 12-bit field and is the **command result code** on replies. The generic ack
`AckNormalCmds` (`r8/l1.java`) reads no body, and its callers branch on `msgRpt` (exposed as `b()`):
e.g. the activation dialog (`j6/f.java`) treats `0` as success and `2` as "land first". Other codes
are command-specific and unmapped.

---

## 4. Demux algorithm — `m8/a.java` (orig `FmLinkDataChanel.java`)

Entry `b(byte[])` `m8/a.java:1320` feeds each byte to `Parser4.b()`; a completed `a7.b` frame goes to
`p()` `m8/a.java:1253`. Dispatch is a **3-level key: (SrcId, groupID, msgId)**:

```
p(frame):
  msgId   = payload[1] & 0xFF     # = i10 in the handlers
  groupID = payload[0] & 0xFF     # = i11 in the handlers
  destId  = header.DestId
  if destId != MODULE_GCS(7):  not for app → reroute/drop  (m8/a.java:1261-1272)
  switch header.SrcId (module ordinal, §5):     # m8/a.java:1274-1317
     MODULE_CAMERA(3)            -> d(msgId, groupID, frame)
     MODULE_REPEATER_RC(16)      -> l(...)
     MODULE_REPEATER_VEHICLE(14) -> g(...)
     MODULE_FC(2)                -> f(...)      # ← main flight telemetry
     MODULE_NFZ(17)              -> i(...)
     MODULE_CV(10)               -> n(...)
     MODULE_ESC(18)              -> e(...)
     MODULE_GIMBAL(8)            -> h(...)
     MODULE_BATTERY(15)          -> c(...)
     MODULE_RC(13)               -> k(...)
     MODULE_ULTRASONIC(22)       -> m(...)
     MODULE_OPTFLOW(4)           -> j(...)
```

Each handler (`c,d,e,f,g,h,i,j,k,l,m,n`) takes `(int msgId=i10, int groupID=i11, frame)`, branches on
`groupID` then `msgId`, constructs the message class, calls its decode method, and forwards the object.
> Note the handler param order is `(i10=msgId, i11=groupID)` but callers invoke `o(i11, i10, …)`, i.e. the
> downstream key tuple is **(groupID, msgId)**. Within the FC handler `f()`, group `12` is the hot
> push-telemetry set and group `3` is the FC command/AI set — both arrive with SrcId = FC(2).

`r2 AutoBlackBox31` is **not a decodable message**: `a(frame)` `m8/a.java:129-131` passes the raw frame
bytes (`i(bVar)` = whole payload copy, `r8/n3.java:68`) to a black-box logger after each FC telemetry decode.

---

## 5. Module ID enum — `j8/h.java$a` (orig `X8BaseCmd.java`), ordinals = wire values

Used for frame SrcId (byte 4) and DestId (byte 5). `j8/h.java:15-37`:

```
0 MODULE_IDLE            6 MODULE_HTTP        12 MODULE_SV_FW            18 MODULE_ESC
1 MODULE_UAV             7 MODULE_GCS (app)   13 MODULE_RC               19 MODULE_SERVO
2 MODULE_FC              8 MODULE_GIMBAL      14 MODULE_REPEATER_VEHICLE 20 MODULE_Default0X14
3 MODULE_CAMERA          9 MODULE_BLACKBOX    15 MODULE_BATTERY          21 MODULE_Default0X15
4 MODULE_OPTFLOW        10 MODULE_CV          16 MODULE_REPEATER_RC      22 MODULE_ULTRASONIC
5 MODULE_OBSAVOID       11 MODULE_SV_DWN      17 MODULE_NFZ
```

---

## 6. Flight-critical message layouts

Offsets are **relative to body start (payload offset 4)**. "LE" per §1. Units/scale are from the
*getter* methods (the raw stored field is the wire value; getters apply scaling). Cites are the decode method.

### 6.1 `AutoFcSportState` — attitude, position, speed, home-distance  ★primary OSD
Key `(SrcId=FC 2, group=12, msgId=2)` `m8/a.java:713-720`. Decoder `y2.v()` `r8/y2.java:101-117`.

| Off | Rd | Field | Units / scaling (getter) | Cite |
|----|----|-------|--------------------------|------|
| 0  | d() f64 | **longitude** | degrees (raw, WGS84) — getter `l()` | `y2.java:103` |
| 8  | d() f64 | **latitude**  | degrees — getter `k()` | `y2.java:104` |
| 16 | e() f32 | **height** (altitude rel. takeoff) | metres — getter `o()` | `y2.java:107` |
| 20 | n() i16 | **groundSpeed** (`groupSpeed`) | raw int16; scale **(UNCERTAIN)** — getter `n()` returns raw (likely cm/s or 0.1 m/s) | `y2.java:108` |
| 22 | n() i16 | **downVelocity** (vertical speed) | raw int16; scale **(UNCERTAIN)**, same as above — getter `m()` | `y2.java:109` |
| 24 | n() i16 | **rollAngle** | degrees = raw/10 — getter `u()` | `y2.java:110,97` |
| 26 | n() i16 | **pitchAngle** | degrees = raw/10 — getter `t()` | `y2.java:111,88` |
| 28 | n() i16 | **headingAngle** (yaw) | degrees = raw/10 — getter `j()` | `y2.java:112,48` |
| 30 | b()     | reserve1 | — | `y2.java:113` |
| 31 | b()     | reserve2 | — | `y2.java:114` |
| 32 | e() f32 | **homeDistance** | metres — getter `p()` | `y2.java:115,72` |
| 36 | n() i16 | field `f28266t` | **(UNCERTAIN)** meaning — getter `q()` returns raw (candidate: distance/flags) | `y2.java:116` |

Body size = 38 B. **Wire order is longitude THEN latitude** (both f64) — do not swap.
(`FLatLng` is built as `e9.a.a(lat, lon)`.)

### 6.2 `AutoFcHeart` — flight phase / control mode / arm-capability  ★flight state
Key `(FC 2, group=12, msgId=1)` `m8/a.java:703-712`. Decoder `v2.v()` `r8/v2.java:112-126`.

| Off | Rd | Field | Meaning | Cite |
|----|----|-------|---------|------|
| 0  | n() i16 | flightTime | seconds **(UNCERTAIN unit)** | `v2.java:114` |
| 2  | n() i16 | startUpTime | | `v2.java:115` |
| 4  | b() | ctrlType | current control source | `v2.java:116` |
| 5  | b() | candidateCtrlType | | `v2.java:117` |
| 6  | b() | **flightPhase** | flight state code (§7.2) | `v2.java:118` |
| 7  | b() | ctrlModel | control model/mode | `v2.java:119` |
| 8  | b() | systemPhase | | `v2.java:120` |
| 9  | b() | disarmCount | | `v2.java:121` |
| 10 | b() | powerConRate | | `v2.java:122` |
| 11 | b() | takeOffCap | take-off capability flag | `v2.java:123` |
| 12 | b() | autoTakeOffCap | auto-takeoff capability flag | `v2.java:124` |

Helper predicates on `flightPhase`: `r()` flying = phase∈{2,3,4}; `u()` = phase==2; `s()` = phase==4;
`t()` on-ground/idle = phase∈{0,1,5} (`v2.java:77-98`). `q()` "can't take off" if takeOffCap==0 ∥ autoTakeOffCap==0.

### 6.3 `AutoFcBattery` — cells, capacity, temperature, percent  ★battery
Key `(FC 2, group=12, msgId=5)` `m8/a.java:735-742`. Decoder `t2.w()` `r8/t2.java:111-135`.

| Off | Rd | Field | Units / scaling (getter) | Cite |
|----|----|-------|--------------------------|------|
| 0  | b()&0xFF | cell1Voltage | **volts = raw/100 + 2.0** — `k()` | `t2.java:114,58` |
| 1  | b()&0xFF | cell2Voltage | volts = raw/100 + 2.0 — `l()` | `t2.java:115,62` |
| 2  | b()&0xFF | cell3Voltage | volts = raw/100 + 2.0 — `m()` | `t2.java:116,66` |
| 3  | b()&0xFF | cell4Voltage | volts = raw/100 + 2.0 (no getter) | `t2.java:117` |
| 4  | n() i16 | currentCapacity | mAh (raw) — `n()` | `t2.java:118` |
| 6  | n() i16 | totalCapacity | mAh (raw) — `t()` | `t2.java:119` |
| 8  | n() i16 | currents | current, raw **(UNCERTAIN unit; likely 0.1 A or mA)** | `t2.java:120` |
| 10 | n() i16 | temperature | **°C = raw/10** — `s()` | `t2.java:121,90` |
| 12 | n() i16 | remainingTime | **(UNCERTAIN unit; s or min)** | `t2.java:122` |
| 14 | b() | remainPercentage | **percent 0-100** — `q()` | `t2.java:123` |
| 15 | b() | uvc | — `u()` | `t2.java:124` |
| 16 | b() | rcNotUpdateCnt | — `p()` | `t2.java:125` |
| 17 | b() | (skipped) | reserved | `t2.java:126` |
| 18 | n() i16 | field `f28190t` | — `r()` | `t2.java:127` |
| 20 | n() i16 | field `f28191u` | — `o()` | `t2.java:128` |
| 22 | n() i16 | cc | cycle count / coulomb count **(UNCERTAIN)** — `j()` | `t2.java:129` |

Pack voltage (`v()`) is computed as cell1+cell2 volts only (`t2.java:107`), so **4-cell detection is
via the 4 cell bytes**; total pack voltage = sum of nonzero cells.

### 6.4 `AutoFcSignalState` — GPS satellite quality + RC/handle signal  ★GPS + link
Key `(FC 2, group=12, msgId=3)` `m8/a.java:721-727`. Decoder `x2.q()` `r8/x2.java:91-101`.
8 bytes, all `b()`:

| Off | Field | Meaning (from getters/thresholds) | Cite |
|----|-------|-----------------------------------|------|
| 0 | f28245h | **satellite count** (GPS) — `k()`; GPS-quality thresholds use `>10`/`>8` | `x2.java:93,35,41-50` |
| 1 | f28246i | GPS accuracy metric **(UNCERTAIN: HDOP-like)** | `x2.java:94` |
| 2 | f28247j | GPS accuracy metric **(UNCERTAIN: PDOP/VDOP-like)** | `x2.java:95` |
| 3 | f28248k | reserved/unused | `x2.java:96` |
| 4 | f28249l | GPS accuracy metric **(UNCERTAIN)** | `x2.java:97` |
| 5 | f28250m | reserved/unused | `x2.java:98` |
| 6 | f28251n | **RC/handle link signal, 0–100 (%)** — `m()` maps >80 STRONG / 30-80 MIDDLE / <30 LOW | `x2.java:99,56-89` |
| 7 | f28252o | RC quality aux; `j()` = `abs(raw/10 − 1)*100` **(UNCERTAIN)** | `x2.java:100,31` |

GPS quality enum `l()` → `i8.j X8GpsNumState {IDEL,STRONG,MIDDLE,LOW}` from sat-count + accuracy
(`i8/j.java`). Link enum `m()` → `i8.k X8HandleSignalState {IDEL,STRONG,MIDDLE,LOW,NOSIGNAL}` (`i8/k.java`).
**No explicit GPS fix-type byte** is decoded here; "has fix" is inferred from sat count/quality.

### 6.5 `AutoHomeInfo` — home point  ★home
Key `(FC 2, group=12, msgId=6)` `m8/a.java:743-752`. Decoder `b3.o()` `r8/b3.java:50-88`.

| Off | Rd | Field | Units | Cite |
|----|----|-------|-------|------|
| 0  | d() f64 | **homeLongitude** | degrees — `m()` | `b3.java:53` |
| 8  | d() f64 | **homeLatitude** | degrees — `l()` | `b3.java:61` |
| 16 | e() f32 | **height** | metres — `k()` | `b3.java:66` |
| 20 | b() | homePointAccuracy | — | `b3.java:71` |
| 21 | b() | homePointType | — | `b3.java:76` |
| 22 | b() | homePointStatus | — | `b3.java:81` |

(Same lon-then-lat order as SportState.) `n()` returns a "changed" flag, not wire data.

### 6.6 `AutoFcErrCode` — system status / error bitmasks  ★faults
Key `(FC 2, group=12, msgId=4)` `m8/a.java:728-734`. Decoder `u2.n()` `r8/u2.java:38-60`.
Four `g()` int32 read as **unsigned 32-bit** (negative → `& 0xFFFFFFFF`):

| Off | Field | Cite |
|----|-------|------|
| 0  | systemStatusCodeA (bitmask) — `j()` | `u2.java:40` |
| 4  | systemStatusCodeB (bitmask) — `k()` | `u2.java:45` |
| 8  | systemStatusCodeC (bitmask) — `l()` | `u2.java:50` |
| 12 | systemStatusCodeD (bitmask) — `m()` | `u2.java:55` |

Individual bit meanings are not resolved in these classes **(UNCERTAIN — bit table lives elsewhere)**.

### 6.7 `AckGetIMUInfo` — raw IMU/baro/ToF (sent in FC group 12)
Key `(FC 2, group=12, msgId=7)` `m8/a.java:753-757`. Decoder `v0.y()` `r8/v0.java:123-142`:
two bytes, thirteen consecutive `n()` int16, two bytes (30-byte body).

| Off | Rd | Field |
|----|----|-------|
| 0 | b() | imuType |
| 1 | b() | imuTemp |
| 2, 4, 6 | n() | gyroX, gyroY, gyroZ |
| 8, 10, 12 | n() | accelX, accelY, accelZ |
| 14, 16, 18 | n() | magX, magY, magZ |
| 20 | n() | baroTemp |
| 22 | n() | baroAltitude |
| 24 | n() | tofDistance |
| 26 | n() | tofAmp |
| 28 | b() | tofTemp |
| 29 | b() | tofAmb |

Raw sensor units, no scaling applied (field names from the class `toString`).

### 6.8 `AutoGimbalState` — gimbal angles  ★gimbal
Key `(SrcId=GIMBAL 8, group=9, msgId=1)` `m8/a.java:1003-1005`. Decoder `a3.o()` `r8/a3.java:42-50`.

| Off | Rd | Field | getter | Cite |
|----|----|-------|--------|------|
| 0  | n() i16 | errorCode | `j()` | `a3.java:44` |
| 2  | b() | stateCode | `m()` | `a3.java:45` |
| 3  | b() | (skipped) | reserved | `a3.java:46` |
| 4  | n() i16 | **rollAngle** | `l()` (raw; scale **UNCERTAIN**, likely 0.1° or 0.01°) | `a3.java:47` |
| 6  | n() i16 | **pitchAngle** | `k()` | `a3.java:48` |
| 8  | n() i16 | **yawAngle** | `n()` | `a3.java:49` |

### 6.9 `AutoNavigationState` — autopilot/mission state  ★mission
Key `(FC 2, group=3, msgId=1)` `m8/a.java:543-547`. Decoder `c3.n()` `r8/c3.java:35-41`.

| Off | Rd | Field | getter | Cite |
|----|----|-------|--------|------|
| 0 | b()&0xFF | taskMode | `l()` | `c3.java:37` |
| 1 | b()&0xFF | naviTaskSta | `k()` | `c3.java:38` |
| 2 | b()&0xFF | apStatus (autopilot status) | `j()` | `c3.java:39` |
| 3 | n()&0xFFFF | wpNUM (waypoint count/index) | `m()` | `c3.java:40` |

### 6.10 `AckGetAiLinePoint` — AI waypoint (reference example, confirmed)
Key `(FC 2, group=3, msgId=38)` `m8/a.java:581-584`. Decoder `j0.v()` `r8/j0.java:127-155`.
Matches the brief: number b, totalnumber b, 2×b skip, lon f64, lat f64, altitude n/10, yaw n,
gimbalPitch n, speed (b & 0xFF), 4× b reserved, nibble byte (hi nibble=`f27951y`, lo nibble=yawMode),
gimbalMode b, trajectoryMode b, missionFinishAction b, rCLostAction b, POI lon f64, POI lat f64, POI alt n/10.

### 6.11 Link/relay telemetry (secondary)
- `AutoFcRelayInfo` `(REPEATER_VEHICLE 14, group=14, msgId=4)` `m8/a.java:898-902`; decoder `w2.k()`
  `r8/w2.java:29-44` — 6 bytes `b()` (f28233h..f28238m); only `f28237l` exposed (`j()`); rest feed blackbox.
- `AutoRelayHeart` `(REPEATER_RC 16, group=14, msgId=5)` `m8/a.java` handler `l()`; decoder `i3.n()`
  `r8/i3.java:33-43` — 1× `n()` i16 `status`; signal = `(status>>12)&3` → `i8.i {LOW,MIDDLE,STRONG}` (`i3.java:10-27`).
- `AutoRCMatchRt` `(RC 13, group=14, msgId=2)` `m8/a.java:1076-1079`; decoder `f3.j()` `r8/f3.java:10-13` — 1 byte.
- `AutoRcState` `(RC 13, group=11, msgId=4)` `m8/a.java:1117-1119`; decoder `h3.k()` `r8/h3.java:19-25` —
  byte rcState, byte rcErrorState (both &0xFF).
- `AutoRelayLinkType` (decoded outside main demux, via `y8/f.java`); decoder `j3.r()` `r8/j3.java:72-83`:
  off0 n() p2pLinkStatus, off2 b() dataLinkType, off3 b() fpvLinkType, off4 b() wifiConnected,
  off5 b() linkMode, off6 b() rsrq (`p()=raw*0.5−19.5`), off7 b() rsrp (`o()=raw−140`), off8 b(), off9 b().
- `AutoRcHeart` (decoded via `y8/j.java`); decoder `g3.k()` `r8/g3.java:23-46`: n() i16 + 2× b() (change-tracked).

### 6.12 Camera / vision (context)
- `AutoCameraStateADV` `(CAMERA 3, group=2, msgId=21)` `m8/a.java:204-209`; decoder `s2.y()` `r8/s2.java:139-164`:
  off0 b() state, off1 b() mode, off2 b() info, off3 n() recTime (packed: sec=`&63`, min=`>>6&63`, hour=`>>12&63`),
  off5 g() freeSpace, off9 g() totalSpace, then **branch on `n6.a.f24481e`** (two body variants for fps/bitrate) —
  `r8/s2.java:151-163`. **(Variant-dependent tail — flag.)**
- `AutoVcTracking` `(CV 10, group=15, msgId=4)` `m8/a.java` handler `n()`; decoder `k3.p()` `r8/k3.java:55-68`:
  g() time, 5× n()&0xFFFF (x,y,w,h,?), b()&0xFF confidence, 4× b() skip, g() trackErrorCode.

---

## 7. Flight-state / mode enumerations

### 7.1 `X8mAiFlyStatus` — `i5/e.java` (AI/mission state machine; app-side, ordinals)
Full ordered value list `i5/e.java:5-62` (ordinal = index):
`IDLE(0), ALL_ITEMS, POINT2POINT_CONFIRM, LINRE_CHILDREN_CONFIRM, LINRE_STRAIGHT_ROUNT_CONFIRM,
LINRE_FLY_ROUNT_CONFIRM, LINRE_HISTORY_ROUNT_CONFIRM, LINRE_OFFLINE_ROUNT_CONFIRM, FOLLOW_CONFIRM,
FOLLOW_NORMAL_CONFIRM, FOLLOW_PARALLEL_CONFIRM, FOLLOW_LOCK_CONFIRM, SURROUNDPOINT_CONFIRM,
SCREW_CONFIRM, AERIALPHOTOGRAPH_CONFIRM, TRIPOD_CONFIRM, HEADINGLOCK_CHOICE_CONFIRM, HEADINGLOCK_CONFIRM,
FIXEDWING_CONFIRM, SAR_CONFIRM, TRAJECTORY_DELAY_COMFIRM, POINT2POINT_RUNNING, LINRE_RUNNING,
FOLLOW_RUNNING, SURROUNDPOINT_RUNNING, SCREW_RUNNING, TRAJECTORY_DELAY_RUNNING, AERIALPHOTOGRAPH_RUNNING,
TRIPOD_RUNNING, HEADINGLOCK_RUNNING, FIXEDWING_RUNNING, AUTO_TAKEOFF_RUNNING(31), AUTO_LANDING_RUNNING,
AUTO_RETURN_RUNNING, FAILSAFE_RETURN_RUNNING, LOWPOWER_LANDING_RUNNING, CRUISE_CONTROL_RUNNING,
POINT2POINT_FINISH, LINRE_FINISH, LINER_HISTORY_SAVE_FINISH, LINRE_FINISH_CONTINUE, FOLLOW_FINISH,
SURROUNDPOINT_FINISH, SCREW_FINISH, AERIALPHOTOGRAPH_FINISH, TRIPOD_FINISH, HEADINGLOCK_FINISH,
FIXEDWING_FINISH, TRAJECTORY_DELAY_FINISH, AUTO_TAKEOFF_FINISH, AUTO_LANDING_FINISH, AUTO_RETURN_FINISH,
FAILSAFE_RETURN_FINISH, LOWPOWER_LANDING_FINISH, CRUISE_CONTROL_FINISH, FOLLOW_3_CONFIRM(56)`.
This is an app-level derived status; the **on-wire** mission state comes from
`AutoNavigationState.apStatus/taskMode/naviTaskSta` (§6.9) — the exact numeric mapping between the two is
**(UNCERTAIN)** and should be derived empirically.

### 7.2 `flightPhase` (AutoFcHeart off 6) — codes inferred from predicates `r8/v2.java:77-110`
Only these are provable from code: `{0,1,5}` = on-ground/idle (`t()`); `2` = a flying state (`u()`, likely
hover/manual); `{3,4}` = other flying states, `4` singled out by `s()`; `{2,3,4}` collectively = "flying"
(`r()`). Individual names for 3/4 and any value >5 are **(UNCERTAIN)** — flag for live-capture confirmation.

---

## 8. Full decoder enumeration (all `r8/*.java`)

Legend: **key** = `(SrcId module, groupID, msgId)` from the `m8/a.java` demux where determinable;
"dec" = decode-method name taking `a7.b`. `—` = not routed through the main `FmLinkDataChanel` demux
(decoded by a secondary handler or request/response path), id unknown from this file.

### Telemetry push (`Auto*`)
| File | Class | dec | Key (Src,grp,msg) | Notes |
|------|-------|-----|-------------------|-------|
| v2 | AutoFcHeart | v() | FC2,12,1 | §6.2 flight phase |
| y2 | AutoFcSportState | v() | FC2,12,2 | §6.1 attitude/pos/speed |
| x2 | AutoFcSignalState | q() | FC2,12,3 | §6.4 GPS sats + RC signal |
| u2 | AutoFcErrCode | n() | FC2,12,4 | §6.6 fault bitmasks |
| t2 | AutoFcBattery | w() | FC2,12,5 | §6.3 battery |
| b3 | AutoHomeInfo | o() | FC2,12,6 | §6.5 home point |
| c3 | AutoNavigationState | n() | FC2,3,1 | §6.9 mission state |
| z2 | AutoFixedwingState | k() | FC2,4,49 | off0 b, off1 b (`m8/a.java:487`) |
| n2 | AutoAccurateLandingResult | j() | FC2,5,9 | precision-land result (`m8/a.java:333`) |
| q2 | AutoBlackBox30 | j() | FC2,10,* | raw→blackbox, no fields (`r8/q2.java`) |
| r2 | AutoBlackBox31 | j(frame,bool) | — | raw frame logger, not a message (§4) |
| p2 | AutoAiSurroundState | k() | FC2,3,74 | `m8/a.java:619` |
| o2 | AutoAiFollowErrorCode | k() | FC2,3,87 | `m8/a.java:637` |
| a3 | AutoGimbalState | o() | GIMBAL8,9,1 | §6.8 gimbal angles |
| s2 | AutoCameraStateADV | y() | CAMERA3,2,21 | §6.12 variant tail |
| k3 | AutoVcTracking | p() | CV10,15,4 | §6.12 tracking box |
| l3 | AutoVcTrackingObjs | m() | CV10,15,38 | tracking objects list (`m8/a.java:1217`) |
| m2 | AutoAccurateLandingRectF | o() | OPTFLOW4,5,6 | `m8/a.java:1047` |
| w2 | AutoFcRelayInfo | k() | REPVEH14,14,4 | §6.11 |
| i3 | AutoRelayHeart | n() | REPRC16,14,5 | §6.11 relay signal |
| f3 | AutoRCMatchRt | j() | RC13,14,2 | §6.11 |
| h3 | AutoRcState | k() | RC13,11,4 | §6.11 |
| g3 | AutoRcHeart | k() | — | via y8/j.java (RC heartbeat) |
| j3 | AutoRelayLinkType | r() | — | via y8/f.java; §6.11 layout |
| d3 | AutoNfzState | k() | NFZ17,17,3 | off0 b()&0xFF (`r8/d3.java`) |
| e3 | AutoNotifyFwFile | m() | CAMERA3,16,5 / REPRC16,16,5 | firmware-file notify |

### Command acks (`Ack*`) — flight/nav/FC group
| File | Class | dec | Key (Src,grp,msg) |
|------|-------|-----|-------------------|
| j0 | AckGetAiLinePoint | v() | FC2,3,38 (§6.10) |
| k0 | AckGetAiLinePointsAction | l() | FC2,3,39 |
| l0 | AckGetAiPoint | l() | FC2,3,53 |
| m0 | AckGetAiSurroundPoint | n() | FC2,3,69 |
| i | AckAiSurrounds | k() | FC2,3,71/73 |
| i0 | AckGetAiFollowMode | k() | FC2,3,86 |
| f | AckAiFollowGetSpeed | k() | FC2,3,89 |
| h | AckAiScrewPrameter | r() | FC2,3,104 |
| p1 | AckPanoramaPhotographType | l() | FC2,3,106 |
| g | AckAiGetGravitationPrameter | j() | FC2,3,117 |
| n1 | AckOnekeyVideoState | j() | FC2,3,118 |
| o1 | AckOnekeyVideoStatus | k() | FC2,3,119-123 |
| c2 | AckTakeOffAndLand | j() | FC2,3,16 / 3,21 (header-only) |
| c | AckAccurateLandingState | l() | FC2,3,108 |
| z0 | AckGetPilotMode | k() | FC2,4,2 |
| e1 | AckGetSportMode | m() | FC2,4,4 |
| u1 | AckSetFcParam | j() | FC2,4,5 |
| p0 | AckGetFcParam | l() | FC2,4,6 |
| w1 | AckSetRetHeight | j() | FC2,4,8 |
| c1 | AckGetRetHeight | k() | FC2,4,9 |
| e | AckAiFollowGetEnableBack | k() | FC2,4,11 |
| v1 | AckSetLostAction | j() | FC2,4,12 |
| w0 | AckGetLostAction | k() | FC2,4,13 |
| y0 | AckGetOpticFlow | j() | FC2,4,15 |
| x0 | AckGetLowPowerOpt | l() | FC2,4,23 |
| d1 | AckGetSensitivity | n() | FC2,4,26/34/36/38 |
| b | AckAccurateLanding | k() | FC2,4,53 |
| d | AckActivateDevice | j() | FC2,4,113 |
| r8/d (dup) | AckActivateDevice | j() | FC2,4,113 |
| o0 | AckGetCaliState (AckGetAircrftCalistate) | r() | FC2,13,6 |
| w | AckCheckIMUException | l() | FC2,13,8 |
| v0 | AckGetIMUInfo | y() | FC2,12,7 (§6.7) |
| a2 | AckSyncTime | j() | FC2,8,4 |
| q0 | AckGetFormatStorageState | k() | FC2,1,21 |
| x1 | AckSnActivateState | k() | FC2,1,22 |
| i2 | AckUploadData | j() | FC2,21,130 |
| f1 | AckGpsModel | k() | FC2,21,133 (off0 b() model) |
| c0 | AckDeviceSN | k() | FC2,16,178 |

### Command acks — camera / gimbal / RC / NFZ / relay / CV / media
| File | Class | dec | Key (Src,grp,msg) |
|------|-------|-----|-------------------|
| d2 | AckTakePhoto | j() | CAMERA3,2,4 |
| y1 | AckStartRecord | j() | CAMERA3,2,2 |
| z1 | AckStopRecord | j() | CAMERA3,2,3 |
| u | AckCameraVideoMode | j() | CAMERA3,2,11 |
| r | AckCameraPhotoMode | j() | CAMERA3,2,10 |
| o | AckCameraInterestMetering | j() | CAMERA3,2,12 |
| b2 | AckTFCarddCap | j() | CAMERA3,2,8 |
| q | AckCameraParameterIndex | j() | CAMERA3,2,{23,25,26,28,30,...} |
| k | AckCameraCurrentParameters | M() | CAMERA3,2,64 |
| s | AckCameraTimelaspePhoto | j() | CAMERA3,2,90/92 |
| t | AckCameraTimelaspeVideo | j() | CAMERA3,2,78 |
| v | AckCameraVideoResolution | k() | CAMERA3,2,24 |
| m | AckCameraFocusePhoto | k() | CAMERA3,2,108 |
| l | AckCameraExposureMode | k() | CAMERA3,2,106 |
| p | AckCameraMeteringMode | m() | CAMERA3,2,72 |
| n | AckCameraFpvInfo | j() | CAMERA3,2,115 |
| m1 | AckOneKeyVideoScene | j() | CAMERA3,2,110 |
| d0 | AckDisConfig | p() | CAMERA3,2,227 |
| g1 | AckMediaFileRequestDownload | l() | CAMERA3,7,1 |
| h1 | AckMediaFileRequestSend | j() | CAMERA3,7,2 |
| j1 | AckMediaFileStopDownload | j() | CAMERA3,7,3 |
| i1 | AckMediaFileSendSpeedControl | j() | CAMERA3,7,4 |
| b0 | AckCloudParamsNew | m() | GIMBAL8,9,29 |
| s0 | AckGetGimbalGain | k() | GIMBAL8,9,31 |
| a1 | AckGetPitchSpeed | k() | GIMBAL8,9,41 |
| u0 | AckGetHorizontalAdjust | k() | GIMBAL8,9,43 |
| y | AckCloudCali | j() | GIMBAL8,9,44 |
| z | AckCloudCaliState | v() | GIMBAL8,9,45 |
| a0 | AckCloudParams | m() | GIMBAL8,9,96/106 |
| r0 | AckGetGimbalCaliState | r() | GIMBAL8,9,51 |
| t1 | AckSetCloudParams | j() | GIMBAL8,9,105 |
| t0 | AckGetGimbalSensorInfo | G() | GIMBAL8,9,96 |
| q1 | AckRcCalibrationState | r() | RC13,11,14/15 |
| h0 | AckFiveKeyDefine | j() | RC13,11,16 |
| b1 | AckGetRcMode | l() | RC13,11,18 |
| s1 | AckRightRoller | j() | RC13,11,19 |
| r1 | AckRcUpgradeData | k() | RC13,16,198 |
| k1 | AckNoFlyNormal | z() | NFZ17,17,1/2 |
| f0 | AckFC0x80 | k() | NFZ17,17,128 |
| g0 | AckFC0x82 | j() | NFZ17,17,130 |
| d3 | AutoNfzState | k() | NFZ17,17,3 |
| l2 | AckWifiUpdate | l() | REPVEH14,14,7 |
| j | AckBandFrequency | l() | REPVEH14,14,52 |
| a | Ack4GModuleID | k() | REPVEH14,14,40 |
| j2 | AckVcSetRectF | v() | CV10,15,3 |
| k2 | AckVersion | s() | *,16,177 (all modules) |
| h2 | AckUpdateSystemStatus | k() | CAMERA3/REPRC16,16,1 |
| f2 | AckUpdateRequest | k() | …,16,2 |
| g2 | AckUpdateRequestPutFile | k() | …,16,3 |
| e2 | AckUpdateCurrentProgress | m() | …,16,6 |
| l2/f2/g2/h2/e2/e3 | firmware-update family | — | group 16 |
| l1 | AckNormalCmds | j() | generic ack (many ids) — header-only, result in `msgRpt` (§3) |
| l2… | | | |

### Not wire messages / helpers
`n3` = `X8BaseMessage` (base, `f()` header parse). `m3` = `MediaFileDownLoadPacket` (download helper).

---

## 9. Implementation checklist & open items

1. **Byte-feed parser**: port `Parser4` (`a7/d.java`) — a per-byte state machine keyed on start flag
   `0xFE`, Len (bits 6-14 of LE short at off1), fixed 16-B header, then `payloadLen = Len−16` bytes;
   verify header CRC16 (off10-11) and frame CRC32 (off12-15). Emit frame on completion.
2. **Demux**: `(SrcId=byte4, groupID=payload[0], msgId=payload[1])`; process only when `DestId==7 (GCS)`.
3. **Body cursor starts at payload offset 4**; read fields LE per §1.
4. **Hot telemetry set** to implement first = FC group 12, msgIds 1-7 (§6.1-6.7) + GimbalState (§6.8)
   + NavigationState (§6.9) + HomeInfo (§6.5).
5. **Flagged uncertainties** (need live capture to pin down):
   - groundSpeed / downVelocity scale (§6.1 off20,22) and `f28266t` meaning (off36).
   - battery `currents`, `remainingTime`, `cc` units (§6.3).
   - `AutoFcSignalState` off1-5 DOP-like fields and off7 formula (§6.4); no explicit GPS fix-type byte.
   - `flightPhase` codes 3/4 and any >5 (§7.2); mapping to `X8mAiFlyStatus` (§7.1).
   - `AutoFcErrCode` individual status bits (§6.6).
   - `msgRpt` 12-bit field role (ack code vs seq) (§3).
   - `AutoCameraStateADV` has a build-flag-dependent tail (`n6.a.f24481e`) — two body variants (§6.12).
   - gimbal angle scaling (§6.8).
