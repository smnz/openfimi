# openfimi — Part 04: FPV Video Stream (aircraft → app)

Scope: how the live camera (FPV) video reaches the FIMI X8 Mini Android app, in enough
detail to receive and decode it on Linux.

Source tree (decompiled, obfuscated — original names from each file's `compiled from:` comment):
a jadx decompile of `com.fimi.app.x8m` V1.1.43.20703 (not included in this repo) . Native libs: the APK's `lib/arm64-v8a/`.

Confidence is tagged **[CONFIRMED]** (read directly in code), **[LIKELY]** (strongly implied),
**[UNKNOWN]** (not determinable from static analysis).

---

## 0. TL;DR

The live video is **NOT** a separate RTSP/HTTP/second-socket stream. It is **multiplexed onto
the same link that carries commands and telemetry**, as a distinct "channel" identified by a
one-byte type field in a lightweight framing header ("FmLink"). The decoder is fed from a
demux chain that is **entirely in Java** (not hidden in a .so):

```
transport (AOA-USB bytestream  OR  Wi-Fi UDP datagrams  OR  4G/peergine)
   └─ FmLink framing  (sync 0xAE, len, TYPE byte, checksum)      y6/d.java  (USB)  /  l8/c.java (UDP)
        └─ TYPE==2  →  VideoDataChanel                            m8/d.java
             └─ NoticeManager.l(bytes)                            v6/c.java
                  └─ FimiH264Video.b(bytes)                        com/fimi/app/x8s/media/FimiH264Video.java
                       └─ FPVUnpack.execute(bytes)  = RTP depacketizer  com/fimi/media/FPVUnpack.java
                            └─ FPVPacket (a full NAL access unit)
                                 ├─ H.265 → Android MediaCodec "video/hevc"  com/fimi/media/FPVDecoder.java   ← ACTIVE path
                                 └─ H.264 → libfpvplayer.so (FFmpeg)         com/fimi/media/FPVPlayer.java    ← present but DORMANT (see §6)
```

Video payload (after FmLink header is stripped) is **RTP**: a standard 12-byte RTP header +
a 2-byte proprietary sub-header + NAL bytes. Codec (H.264 vs H.265/HEVC) is auto-detected
**inband** from the NAL start-code; **SPS/PPS/VPS are sent inband**. Typical frame = 1280×720.

A start command IS sent on connect: camera module, **command 114**, args `(enable=1, w=1280,
h=720)` (§5).

Good news for reimplementation: the whole receive/demux/depacketize path is reconstructable
Java; decoding is standard H.264/H.265 you can hand to ffmpeg.

---

## 1. Transport — same link as commands, multiplexed [CONFIRMED]

There is a single `SessionManager` (`v6/e.java`) holding one primary `BaseConnect`
(`q6/b.java`) that carries **everything**. The concrete transport is chosen in
`CommunicationManager.d()` (`l8/a.java:72`) by a `ConnectType` enum (`l8/b.java`: `Tcp, Aoa, Udp`):

| Type | Class created | Endpoint | Notes |
|------|---------------|----------|-------|
| **Aoa** | `p8/a.java` → `y6/a.java` (AOAConnect) | USB accessory (RC tethered) | `l8/a.java:98` |
| **Udp** | `o8/a.java` → `x6/a.java` (UdpConnect) **and** a 2nd `x6.a` | **192.168.40.210**, ports **10051** (cmd+data, bidir) and **9397** (recv-only) | `l8/a.java:80,83-86` |
| **Tcp** | `n8/a.java` → `w6/a.java` | **192.168.42.1:10010** | alternate/older AP; same router (`n8/a.java:20-23`) |
| 4G | `q6/h.java` (peergine P2P) | peergine relay, carries video as **TYPE 11** | `l8/g.java` PgConnectManager |

Key point: in **every** case the received bytes are fed into the **same** `DataChanel`
router (`l8/c.java`, an instance of `q6.i`). So the FmLink multiplex (commands + video +
telemetry) is identical across USB, UDP and TCP. For a Linux client the **Wi-Fi UDP path is
easiest** (plain `recvfrom`, no AOA): associate to the drone AP (SSID contains `X8Min_`,
`X8mMainActivity.java:376`), drone is `192.168.40.210`.

### 1a. AOA (USB) read loop [CONFIRMED]
`y6/a.java` (AOAConnect): `UsbManager.openAccessory()` → `ParcelFileDescriptor` in/out streams
(`y6/a.java:148-166`). A reader thread loops `inputStream.read(buf[16384])` and calls
`v(buf,len)` (`y6/a.java:110-115`), which runs the bytestream through the **NetDecoder**
framer (`this.f33065q = new d("Decoder1")`, i.e. `y6/d.java`) and dispatches each decoded
frame to the `q6.i` handler (= `l8.c`). Commands are written back on the same output stream.
So over USB **video and commands share one bidirectional accessory pipe**.

### 1b. Wi-Fi UDP read loop [CONFIRMED]
`x6/a.java` (UdpConnect): a thread does `DatagramSocket.receive()` into a 2048-byte buffer and
calls `handler.a(datagram, -1)` (`x6/a.java:79-84`). The `-1` triggers the self-framing branch
in `l8/c.java:23` (one datagram == one FmLink frame; see §3).

---

## 2. FmLink framing (the multiplex header) [CONFIRMED]

Defined by the USB framer `NetDecoder` (`y6/d.java`). Over a bytestream (USB) it scans for the
sync byte and length; over UDP each datagram is already one frame and `l8/c.java` parses the
same header. Fields are **little-endian** (note: the inner RTP header in §4 is big-endian).

```
offset  size  meaning
  0       1    SYNC = 0xAE (174)                                   y6/d.java:57  (-82 signed)
  1..2    2    uint16 LE "hdrfield"                                y6/d.java:69
                 low nibble (hdrfield & 0x0F) = header form:
                   == 1 → SHORT header, total len = (hdrfield & 0xFFF0) >> 4  (12-bit)  y6/d.java:85 ; header size 5
                   == 2 → LONG  header, total len = uint16 LE @ offset 4                y6/d.java:82 ; header size 8
  3       1    TYPE (channel id)                                   y6/d.java:104 (byte @ idx+3)
  (len-1) 1    header checksum = (sum of header bytes[0 .. hdrsize-2]) & 0xFF   y6/d.java:91-98
  hdrsize ...  PAYLOAD  (length = total_len - hdrsize)             y6/d.java:105-107
```
Valid total length range `[hdrsize, 8692]` (`y6/d.java:87`). Bad checksum / length → skip 1 byte
and resync (`y6/d.java:100,112`).

**UDP variant** (`l8/c.java:23-34`): when called with code `-1`, it assumes a **5-byte** short
header, reads `TYPE = bArr[3]`, and strips 5 bytes to get the payload. (So Wi-Fi uses the short
header form; `[0]=0xAE, [1..2]=len/type field, [3]=TYPE, [4]=checksum, [5..]=payload`.)

---

## 3. Channel routing — which TYPE is video [CONFIRMED]

`DataChanel.a(bytes, type)` (`l8/c.java:36-59`) routes by the TYPE byte:

| TYPE | Handler | Meaning |
|------|---------|---------|
| 0  | `m8/a.java` CommandDataChanel → `NoticeManager` telemetry/ack parsers (`r8.*`) | **commands / telemetry / acks** (huge dispatcher, module+cmd id) |
| **2**  | `m8/d.java` VideoDataChanel → `NoticeManager.l()` | **LIVE FPV VIDEO** (the RTP stream) |
| 5  | (ignored) | — |
| 6  | `m8/b.java` FwUploadDataChanel | firmware-upload progress |
| 7  | `m8/c.java` MediaDataChanel → `NoticeManager.i()` | recorded-media **download** (SD files/thumbnails), not live FPV |
| 11 | `NoticeManager.h()` (`l8/c.java:51`) | **4G video** (complete HEVC packets, peergine path) |

So **live FPV = FmLink TYPE 2**. `m8/d.java` is trivially:
```java
public void a(byte[] bArr){ v6.c.g().l(bArr); }   // → g.b(bArr) → FimiH264Video.b()
```
`NoticeManager` (`v6/c.java`) just forwards to the registered video listener `g`
(`v6/g.java`, VideodDataListener): `.l()`→`g.b()` (main RTP video), `.h()`→`g.a()` (4G video).
The listener is `FimiH264Video` (`FimiH264Video.java`, registered via `c.g().r(this)` at
`onAttachedToWindow`).

> Note on the USB dispatcher `y6/a.java:117-160`: it switches on the frame TYPE and additionally
> always re-dispatches each frame as type 0. This looks like a jadx control-flow artifact; the
> authoritative routing table is `l8/c.java` above. **[LIKELY]**

> Naming caution: there is an unrelated enum `y6/c.java LinkMsgType {FmLink4,JsonData,FwUploadData,
> MediaDownData}` used on the **send** side (`q6.a.m()`), whose ordinals do NOT equal the receive
> TYPE codes above. Don't conflate them.

---

## 4. Video payload = RTP + 2-byte sub-header [CONFIRMED]

The TYPE-2 payload handed to `FimiH264Video.b()` is depacketized by `FPVUnpack`
(`com/fimi/media/FPVUnpack.java`). It is **RTP** (big-endian), 14-byte combined header
(`HEAD_LENGTH=14`, `RTP_HEAD_LENGTH=12`):

```
RTP header (12 bytes, big-endian)                     FPVUnpack.execute()
  [0]      0x80  = RTP v2, no pad/ext/CSRC            line 46  (getUnsignedByte(0)==128)
  [1]      payload type; bit 0x80 = RTP marker        line 117 (getUnsignedByte(1)&0x80)
  [2..3]   uint16 sequence number                     line 47  (getShort(2))
  [4..7]   uint32 timestamp  (used as decoder PTS)     line 48  (getUnsignedInt(4))
  [8..11]  SSRC (ignored)
Proprietary sub-header (2 bytes @ 12..13)
  [12]     0x7C(124) = normal fragment; other = special packet   line 78
  [13]     frame flags:
              0x08 → AI tracking RECT payload (isRect)           line 84
              0x09 → AI object-detection payload (isObjs)        line 97
              0x80 → START of a new access unit (+ codec sniff)  line 108
              0x40 → END of access unit (emit when RTP marker set) line 117
Payload (NAL bytes) from offset 14                     line 109-115
```

`FPVUnpack` reassembles a complete NAL **access unit** across RTP fragments (accumulating into a
netty `ByteBuf`), detecting loss via sequence gaps (`isLost`) and splitting on PTS change
(`seqPre/ptsPre`). Output = `FPVPacket` objects with `type` (1=H.264, 2=H.265), `pts`, and the
raw bytes. `FimiH264Video.b()` (`FimiH264Video.java:181-196`) then:
* `isRect` → AI target rectangle → `X8Camera9GridView` overlay (parsed in `o()`),
* `isObjs` → AI detections (parsed in `p()`, boxes normalized by /65535),
* else → `decodeH265Packet()` (type 2) or `decodeH264Packet()` (type 1).

So **AI tracking boxes are carried inband in the same RTP stream** as the video — a Linux client
can ignore sub-header `[13]==0x08/0x09` packets if it only wants pixels.

---

## 5. Starting the stream [CONFIRMED]

Live video begins flowing once the link is up, but the app also sends an explicit FPV
**config/start** command during post-connect init:

```
X8mMainActivity.s3()  (called at X8mMainActivity.java:775, in the on-connect sequence)
  → q8.b.s(1, 1280, 720, cb)            q8/b.java:116
  → z8.a.u0(1,1280,720,cb)              z8/a.java:154
  → j8.a (CameraCollection).s(1,1280,720)   j8/a.java:221
```
which builds (`j8/a.java:223`):
```
payload = { 0x02, 0x72, 0x00, 0x00,   // module=2 (camera), cmd=114 (0x72)
            01 00 00 00,              // arg0 = enable/mode = 1   (LE int)
            00 05 00 00,              // arg1 = width  = 1280     (LE int)
            D0 02 00 00 }             // arg2 = height = 720      (LE int)
```
This is sent as a normal command on the **command channel** (`k.D()` → `w(y6.c.FmLink4)`,
`j8/k.java:28-35`), i.e. it goes out as an FmLink frame and is answered on TYPE 0; the callback
logs `"set fpv result"` (`X8mMainActivity.java:363`).

Related camera commands (same `module=2`, `[0]=module,[1]=cmd`): request current camera params
(`AckCameraCurrentParameters`, cmd via dispatcher id **177**-ish in module 16 is FC telemetry —
camera params come back as `r8.k`), and **FPV info** query → `AckCameraFpvInfo` = **cmd 115**
(`m8/a.java:254`, parser `r8/n.java`) which returns `{enType, width, height}`.

**[LIKELY]** The drone starts pushing TYPE-2 frames as soon as the session is up; the cmd-114
call mainly sets resolution/mode. A minimal Linux receiver can probably just start reading
UDP/USB and demux TYPE 2 without sending anything, but sending cmd 114 is the documented path.
**[UNKNOWN]** whether cmd 114 is strictly required before frames flow.

---

## 6. Codec, resolution, decode boundary (JNI) [CONFIRMED]

### Codec detection (inband) — `FPVUnpack.checkEncodeFormat()` (`FPVUnpack.java:17-31`)
On a start-of-AU packet it inspects the NAL start code at payload offset `00 00 00 01`
(bytes [14..17]) and the first NAL header byte [18]:
* **H.264** if `(b[18] & 0x1F) == 7` (SPS)  → `FPVPacket.type = 1`
* **H.265/HEVC** if `((b[18] & 0x7E) >> 1) ∈ {32,33,34,39,19,1}`
  (VPS=32, SPS=33, PPS=34, SEI_PREFIX=39, IDR_W_RADL=19, TRAIL_R=1) → `FPVPacket.type = 2`

→ **SPS/PPS(/VPS) are transmitted inband**, in Annex-B start-code form. Default assumption is
H.265 (`FPVUnpack.java:15` `new FPVPacket(2)`). A Linux decoder can simply feed the reassembled
Annex-B AUs to ffmpeg and let it detect the codec.

### Resolution / fps
* Typical/default **1280×720** (`FPVDecoder.java:24-25`, `FPVPlayer.java:17-18`, start cmd args,
  `FpvFrame.java` buffers sized 1280×720). `MediaFormat.createVideoFormat("video/hevc",1280,720)`
  is just an initial format; real dims come from the decoder's `onOutputFormatChanged`
  (`FPVDecoder.java:117-126`). **[CONFIRMED default; actual stream dims from AckCameraFpvInfo/
  decoder output]**
* **fps: [UNKNOWN]** — not a static constant. `FPVDecoder` only *measures* it (logs
  `码流帧率`/"stream frame rate", `FPVDecoder.java:fpsCount`). Render pacing caps ~1 frame / 31 ms
  (`FPVDecoder.java:doRender`, ≈32 fps ceiling). **[LIKELY ~30 fps.]**
* `AckCameraFpvInfo` (`r8/n.java`, cmd 115) reports `{enType, width, height}`; `enType`
  numeric→codec mapping **[UNKNOWN]**, but irrelevant since codec is auto-detected inband.
* H.264↔H.265 and quality are **camera encode settings** on the drone
  (`r8/k.java` `AckCameraCurrentParameters.cameraVideoEncode / cameraVideoResolution /
  cameraVideoQuality`; UI string `x8_video_encode_h265`). `FPVPlayer.setDecodeWay()` only selects
  HW (`MediaCodec.createDecoderByType`) vs SW (`OMX.google.hevc.decoder`) HEVC decoder on the
  phone (`FPVDecoder.java:289-296`) — it does NOT change what the drone sends.

### Decode boundary / native methods
**Active live-FPV decoder = Android MediaCodec**, MIME `"video/hevc"` (`FPVDecoder.java:22`).
This is platform Android, *not* a FIMI .so — a Linux reimplementation substitutes any
H.265/H.264 decoder (e.g. ffmpeg).

`libfpvplayer.so` JNI (declared by `com.fimi.media.FPVPlayer`, verified as exported symbols):
```
Java_com_fimi_media_FPVPlayer_native_1init         long native_init(Surface)                 FPVPlayer.java:56
Java_com_fimi_media_FPVPlayer_native_1decodeData    boolean native_decodeData(byte[],int,long) FPVPlayer.java:54  (H.264 path)
Java_com_fimi_media_FPVPlayer_native_1renderData    boolean native_renderData(byte[],int,w,h,fmt,long) FPVPlayer.java:58 (YUV→GL)
Java_com_fimi_media_FPVPlayer_native_1unInit        void native_unInit(long)                  FPVPlayer.java:60
```
Internally libfpvplayer is **FFmpeg-based** (strings: `FFmpegDecoder::InitFFDecoder`,
`avcodec_find_decoder/alloc_context3/open2/send_packet/receive_frame`), paired with
`libffmpeg.so` (both `System.loadLibrary` at `FPVPlayer.java:22-23`).

> **[CONFIRMED] Caveat:** in this build the libfpvplayer H.264 path is **dormant**. `native_init`
> is never called, so `mPlayerHandle` stays 0, and `decodeH264Packet()` early-returns on the
> `mPlayerHandle != 0` guard (`FPVPlayer.java:16,78-80`). Live FPV therefore runs through the
> **MediaCodec HEVC** path only here. The H.264 machinery exists but is unused at runtime.

Other libs are **not** live FPV:
* `libQMedia.so` = Qiniu `com_qiniu_droid_media_*` (media-info/short-video) — used for
  recorded/album video, grep shows no live-FPV caller.
* `libijkplayer.so / libijkffmpeg.so / libijksdl.so` = ijkplayer, used by
  `com/fimi/media/FimiVideoPlayer.java` (plays a file/URL `mPath`) — recorded-media playback.

---

## 7. What a Linux "openfimi" FPV receiver needs

1. **Get the bytes.** Easiest: join drone Wi-Fi AP (`X8Min_*`), bind UDP and read datagrams from
   **192.168.40.210** (ports **10051** and/or **9397**). (AOA-USB is the alternative; it's the
   same framing over a USB-accessory pipe.) **[LIKELY 9397 = video, 10051 = cmd+telemetry; both
   demux identically — UNKNOWN which strictly carries TYPE-2 — just demux both.]**
2. **Parse FmLink frame** (§2): check `0xAE`, read header form from low nibble of the LE uint16 at
   [1], get `TYPE=[3]`, verify header checksum, slice payload. (UDP: one datagram = one frame,
   5-byte header.)
3. **Keep TYPE==2** payloads (and optionally TYPE==11 for 4G).
4. **RTP depacketize** (§4): verify `[0]==0x80`, track seq/timestamp, use sub-header `[13]`
   flags 0x80/0x40 (+ RTP marker) to reassemble Annex-B access units; drop `[13]==0x08/0x09`
   (AI overlay) unless wanted.
5. **Feed AUs to ffmpeg** — codec auto-detects from inband SPS/PPS/VPS (§6). Use RTP timestamp
   as PTS.
6. (Optional) Send the start/config command (§5): FmLink TYPE-0 frame wrapping
   `{02 72 00 00  01000000 00050000 D0020000}` to request 1280×720.

Everything in steps 2–4 is pure Java in the APK and fully reconstructable; nothing about the
framing or depacketization is hidden inside a .so. The only native piece (libfpvplayer FFmpeg)
is both standard and, in this build, unused.

---

## 8. Honest limits of static analysis
* **[UNKNOWN]** exact split of traffic between UDP 10051 vs 9397 (both routed through the same
  demux; not disambiguated in code).
* **[UNKNOWN]** whether the drone needs cmd-114 before it emits video, or streams unconditionally.
* **[UNKNOWN]** precise fps and `enType` numeric codec codes (not needed — inband detection).
* **[LIKELY]** the `y6/a.java` double-dispatch (always also type 0) is a decompiler artifact.
* The RTP sub-header bit meanings (0x7C/0x08/0x09/0x80/0x40) are inferred from `FPVUnpack`
  branch behavior; they are **[CONFIRMED]** as used by the app, but field *names* are ours.
* On-wire bytes should be validated against a real capture (USB or UDP) before trusting offsets.

### Primary evidence files
`com/fimi/media/FPVUnpack.java`, `FPVPlayer.java`, `FPVDecoder.java`, `FpvFrame.java`;
`com/fimi/app/x8s/media/FimiH264Video.java`; `v6/c.java`, `v6/e.java`, `v6/g.java`;
`l8/a.java`, `l8/b.java`, `l8/c.java`, `l8/g.java`; `y6/a.java`, `y6/c.java`, `y6/d.java`;
`x6/a.java`, `o8/a.java`, `n8/a.java`, `p8/a.java`, `q6/b.java`, `q6/h.java`, `q6/i.java`;
`m8/a.java`, `m8/b.java`, `m8/c.java`, `m8/d.java`; `q8/b.java`, `q8/c.java`, `z8/a.java`,
`j8/a.java`, `j8/h.java`, `j8/k.java`; `r8/n.java`, `r8/k.java`;
libs `libfpvplayer.so`, `libffmpeg.so` (live), `libQMedia.so`/`libijk*.so` (recorded only).
