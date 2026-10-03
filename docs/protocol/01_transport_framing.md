# openfimi — Transport & Framing Layer Specification

Reverse-engineered from the decompiled FIMI X8M Android app (authorized, user's own
drone). All citations are `file:line` relative to the decompiled source root
a jadx decompile of `com.fimi.app.x8m` V1.1.43.20703 (not included in this repo). Obfuscated class names are given with the original name from
each file's `compiled from:` comment in parentheses.

> **Scope:** USB transport + framing only. The command/payload semantics (what each
> module command does) are the command layer and are out of scope, except where needed
> to explain addressing, sequencing and ACK matching.

---

## 0. TL;DR layering

Three nested layers sit between a command's payload bytes and the USB wire:

```
  [ application payload ]                      e.g. {0x10, 0xB2, 0x00, 0x00, ...}
        │  wrapped by LinkPacket4  (a7.*, class "LinkPacket4"/"Header4")
        ▼
  [ 0xFE | 16-byte header (len, src, dst, seq, crc16, crc32) | payload ]   "FmLink4"
        │  wrapped by UsbLinkPacket (c7.e) — ALWAYS present on the AOA/UDP wire
        ▼
  [ 0xAE | 5-byte header (ver, len, type, checksum) | FmLink4-frame ]
        │  written to the AOA output stream
        ▼
  [ USB Android Open Accessory bulk endpoint ]
```

On receive the two wrappers are peeled in the opposite order: `y6.d` (NetDecoder) finds
`0xAE`, validates the outer header and strips it; the inner bytes are then streamed one
at a time into `a7.d` (Parser4), which finds `0xFE`, validates both CRCs and emits the
decoded frame.

**There is NO byte-stuffing / escape mechanism anywhere in this protocol.** Both layers
resynchronize purely by scanning for their start byte and validating length + checksum.
(One might expect an "escape layer" in `c7/e.java`; that class is a plain
length-prefixed wrapper, not an escaper — see §3.)

---

## 1. USB transport — Android Open Accessory (AOA)

### 1.1 Roles: which side is host, which is peripheral

The phone runs the **Android AOA *accessory* API**: it calls
`UsbManager.getAccessoryList()` and `UsbManager.openAccessory(UsbAccessory)`
(`q8/c.java:40,47` ConnectManager; `y6/a.java:185` AOAConnect). In USB-OTG/AOA terms
this means:

- **The phone is the USB peripheral / "accessory".**
- **The RC (remote controller) is the USB host** — it is the device that detects the
  phone, performs the AOA handshake, and puts the phone into accessory mode. The RC's
  identity is what appears as the `UsbAccessory` (manufacturer/model/version strings).

Confirmed by `res/xml/accessory_filter.xml` (the `<usb-accessory>` filter the app
registers for `USB_ACCESSORY_ATTACHED`), which lists the **RC** hardware:

| manufacturer | model | version |
|---|---|---|
| `Beijing FIMI Technology Limited` | `RCX6B` | `V020SP11B160602R101` |
| `Beijing FIMI Technology Limited` | `RCX6E` | `V020SP11B160602R105` |
| `Beijing FIMI Technology Limited` | `RCX6F` | `V020SP11B160602R106` |

`res/xml/accessory_filter.xml:3-14`.

> The `version` string is also used as a hardware discriminator at runtime: version
> `V020SP11B160602R105` (RCX6E) selects a different config branch
> (`n6.a.f24481e=1, n6.a.f24485i="9"` vs `0 / "3"`), `q8/c.java:49-55`.

### 1.2 Attach → connect flow

1. USB attach fires `android.hardware.usb.action.USB_ACCESSORY_ATTACHED`, caught by
   `FimiAoaSplashActivity` which calls `q8.c.c().a(this)`
   (`com/fimi/app/ui/FimiAoaSplashActivity.java:16-17,28-31`).
2. `ConnectManager.a(Context)` (`q8/c.java:30`): gets the `UsbManager`, takes
   `getAccessoryList()[0]` (`q8/c.java:40-45`), and if it lacks permission requests it
   with a `PendingIntent` whose action is **`com.google.android.DemoKit.action.USB_PERMISSION`**
   (`q8/c.java:38,60`). When permission is held it calls
   `l8.a.b().c(accessory)` then `l8.a.b().d(context, l8.b.Aoa)` (`q8/c.java:57-58`).
3. `CommunicationManager.d(ctx, Aoa)` (`l8/a.java:73`) constructs
   `new p8.a(context, accessory, this)` (UsbConnectThread) (`l8/a.java:98`).
4. `p8.a.run()` (`p8/a.java:46`) constructs `new y6.a(ctx, accessory, l8.c, y6.b)`
   (AOAConnect) and, if `t()` (connected) is true, calls `y()` to start the threads and
   registers the session via `v6.e.e().c(...)` (`p8/a.java:49-53`).

### 1.3 Opening the stream + the initial `write(0)` byte

`AOAConnect.u(UsbAccessory)` (`y6/a.java:184-200`):

```java
ParcelFileDescriptor pfd = usbManager.openAccessory(usbAccessory);      // y6/a.java:185
AutoCloseInputStream  in  = new AutoCloseInputStream(pfd);               // y6/a.java:188
AutoCloseOutputStream out = new AutoCloseOutputStream(pfd);              // y6/a.java:189
out.write(0);           // <-- single 0x00 byte, immediately after open    y6/a.java:192
connected = true;       // f33060l = true                                  y6/a.java:193
```

- The stream is a `ParcelFileDescriptor` from `openAccessory`, read via
  `AutoCloseInputStream` and written via `AutoCloseOutputStream`
  (`y6/a.java:32-38,186-190`).
- **Initial control byte:** exactly **one `0x00` byte** is written right after opening
  the output stream (`y6/a.java:192`). It gates the "connected" flag (`f33060l`). Its
  semantic purpose is not documented in the code — most likely a stream-wake / sync byte
  to the RC. *(Flagged as a minor uncertainty — see §12.)*
- No other transport-level control bytes exist on open. There is **no login/auth
  handshake at the transport layer** after this (see §11).

### 1.4 Read / write threads & buffer size

`AOAConnect.y()` (`y6/a.java:378-388`) starts:

- **Read thread** (`y6/a.java:105-132`): blocking loop `in.read(f33061m)` where
  `f33061m = new byte[16384]` (**16384-byte read buffer**, `y6/a.java:65`). Each chunk is
  handed to `v(buf, n)` (`y6/a.java:117-120`) which feeds the RX decoder (§8). On
  `IOException` it signals `y6.b.a(1)` (disconnect) and exits (`y6/a.java:125-128`).
- **Write thread** (`y6/a.java:137-173`): drains a `LinkBlockingDeque<q6.a>`
  (`f33057i`, `y6/a.java:59`); for each command calls `w(cmd)` which writes
  `cmd.j()` (the fully framed bytes) to the output stream
  (`y6/a.java:236-243,154-161`). FmLink4 commands that expect a reply (`r()==true`) are
  also registered with the retransmission/ACK thread `u6.a` (`y6/a.java:157-159`, §10).
- A raw escape hatch `g(byte[])` writes bytes straight to the output stream
  (`y6/a.java:327-337`), and `h(q6.a)` enqueues normal commands or writes
  `FwUploadData` frames immediately (`y6/a.java:339-347`).

### 1.5 Linux implication

To replace the phone, a Linux box must behave as the phone did: **a USB gadget /
peripheral implementing the AOA (Android Open Accessory) protocol in accessory mode.**
Concretely, against the RC acting as USB host it must:

1. Answer the AOA handshake control requests on EP0: `GET_PROTOCOL` (bRequest `51`),
   the `SEND_STRING` identity strings (bRequest `52`), and `START_ACCESSORY`
   (bRequest `53`).
2. Re-enumerate as the Google accessory interface (VID `0x18D1`, PID `0x2D00`/`0x2D01`)
   exposing the bulk **IN** and **OUT** endpoints.
3. On connect, send one `0x00` byte on the bulk OUT endpoint (mirrors `write(0)`), then
   speak the framing in §3–§6.
   Linux can do this with the `g_accessory`/ConfigFS USB gadget or a raw gadget driver.
   Note the RC drives enumeration; the Linux side is passive until the host starts AOA.

> The same framing is also used over **UDP** (`CommunicationManager` can instead open
> `192.168.40.210:9397` / `:10051`, `l8/a.java:80-88`) and over **TCP** (`n8.a`,
> `l8/a.java:103-105`). AOA is the RC path and the focus here. The outer `c7` wrapper
> (§3) is applied for **both AOA and UDP**, not TCP (see §3.2).

---

## 2. Link types enum

`y6/c.java` (LinkMsgType) — ordinal values matter, they index a switch in the write
thread (`y6/a.java:84-99,157`):

| ordinal | name | meaning |
|---|---|---|
| 0 | `FmLink4` | normal command / telemetry frame (the main protocol) |
| 1 | `JsonData` | JSON blob stream |
| 2 | `FwUploadData` | firmware upload (written immediately, bypasses queue, `y6/a.java:341`) |
| 3 | `MediaDownData` | media download stream |

A command's link type defaults to `FmLink4` (`q6/a.java:23`) and is set in
`k.D()` via `w(y6.c.FmLink4)` (`j8/k.java:29`).

---

## 3. Outer wrapper — UsbLinkPacket (`c7.e` / `c7.d`)

### 3.1 Byte layout (5-byte header + body)

Built by `c7.e.b()` (`c7/e.java:24-34`) using header `c7.d` (`c7/d.java`):

```
offset  size  field
0       1     START = 0xAE          (c7/d.java:25,  byte literal -82)
1       1     low nibble  = version ; high nibble = bits[0:4] of length
2       1     bits[4:12] of length
3       1     type                  (c7/d.java:30 ; = the int passed to new c7.e(type))
4       1     checksum = (byte0+byte1+byte2+byte3) & 0xFF   (c7/d.java:31-35)
5..     N     body = the inner FmLink4 frame
```

Field encoding, from `c7/d.java:22-37`:

- `bArr[0] = 0xAE`.
- `bArr[1] = (version & 0x0F) | ((length & 0x0F) << 4)` where `version` is the value
  passed to `d.d(..)` and is **always 1** (`c7/e.java:30` calls `f7802a.d(1)`).
- `bArr[2] = (length >> 4) & 0xFF`.
- **`length` = total outer frame size including the 5-byte header** = `bodyLen + 5`
  (`c7/e.java:26` `i10 = c10 + 5`, then `d.b(i10)` at `c7/e.java:28`). It is a **12-bit**
  field: nibble of byte1 + all of byte2. Little-endian when read as the u16 at
  bytes[1..2]: `len = (u16 >> 4) & 0x0FFF`, `version = u16 & 0x0F`.
- `bArr[3] = type & 0xFF`. For `FmLink4` the type is **0** (`j8/k.java:15` `new c7.e(0)`).
- `bArr[4] = sum(bArr[0..3]) & 0xFF` — an **8-bit additive checksum** over the 4 header
  bytes (NOT a CRC despite the log message "crc header" in the decoder).

The body (`c7.f` UsbPayLoad, a 1425-byte `ByteBuffer`, `c7/f.java:10`) is the inner
FmLink4 frame bytes, copied verbatim after the header (`c7/e.java:31-32`). **No escaping
is performed on the body.**

### 3.2 When the outer wrapper is applied

`j8.k.D()` (X8SendCmd, `j8/k.java:27-36`):

```java
byte[] f10 = this.f21258m.f();     // the inner FmLink4 frame (a7.b.f(), §4)
w(y6.c.FmLink4);
if      (k8.c.b().f())  f10 = B(f10);   // link == AOA  -> wrap  (j8/k.java:30)
else if (k8.c.b().k())  f10 = B(f10);   // link == UDP  -> wrap  (j8/k.java:32)
t(f10);                             // store as the command's output bytes
```

- `B(byte[])` wraps with `new c7.e(0)` (type 0) (`j8/k.java:14-18`).
- `k8.c.b().f()` ≡ "current link is AOA"; `.k()` ≡ "current link is UDP"
  (`k8/c.java:164-166,184-186`; `GlobalConfig.f22281k` holds the active `l8.b`).
- So on the **AOA wire the outer `0xAE` wrapper is ALWAYS present.** For TCP it is
  omitted (the raw `0xFE` frame is sent).

### 3.3 Alternate 8-byte outer header (RX only)

The decoder also accepts a `version == 2` variant with an 8-byte header and a 16-bit LE
length at offset 4 (`y6/d.java:74-82`). The app only ever *sends* version 1; version 2
is a receive-side variant (used by larger media/fw streams). Documented for completeness.

---

## 4. Inner frame — LinkPacket4 / "FmLink4" (`a7.*`)

This is the core protocol frame. Header class `a7.a` (Header4), payload class `a7.c`
(LinkPayLoad4), assembler `a7.b` (LinkPacket4), parser `a7.d` (Parser4).

### 4.1 Complete on-wire layout

```
offset  size  field              encoding / source
0       1     START = 0xFE       a7/a.java:12,87  (static byte f780q = -2)
1       2     ver+len (u16 LE)   bits[0:5]=version(=4); bits[6:15]=length  a7/a.java:88-91
3       1     flags              bits[0:2]=encrypt; bits[2:5]=f784d; bits[5:8]=f785e  a7/a.java:92-94
4       1     srcId              module id of sender          a7/a.java:95
5       1     destId             module id of recipient       a7/a.java:96
6       1     reserve2           a7/a.java:97 (f788h)
7       1     reserve3           a7/a.java:98 (f789i)
8       2     seq (u16 LE)       sequence number              a7/a.java:99-101
10      2     crcHeader (u16 LE) CRC-16/MCRF4XX over bytes[0..9]   a7/a.java:102-106
12      4     crcFrame (u32 LE)  CRC-32 (custom) over the payload       a7/a.java:107-111
16      L     payload            L = length - 16 bytes
```

Total header = **16 bytes** (`a7.b.f()` uses `r10 + 16`, `a7/b.java:48-49`).

### 4.2 Field details

- **START** = `0xFE` (`a7/a.java:12`, `f780q = -2`). The parser's sync byte
  (`a7/d.java:194`).
- **ver+len** (`a7/a.java:88`): `u16 = (version & 0x1F) | ((length & 0x1FF) << 6)`,
  stored little-endian at bytes[1..2].
  - `version` = `f781a` = **4** (`a7/a.java:57`), occupies bits 0–4.
  - bit 5 is unused.
  - **`length`** occupies bits 6–14 — a **9-bit** field, so `length ≤ 511`. **It counts
    the TOTAL frame size including the 16-byte header** (`a7/b.java:48-50`:
    `k(r10 + 16)`). Payload length = `length - 16` (`a7/a.java:69-71` `b()`).
  - Decode: `length = (u16 >> 6) & 0x1FF` (`a7/d.java:221-223`).
  - *Implication: a single FmLink4 frame carries at most 511-16 = 495 payload bytes.*
- **flags byte[3]** (`a7/a.java:92`): `(encrypt & 0x03) | (f784d & 0x1C) | (f785e & 0xE0)`.
  - bits 0–1 **encrypt** (`r()`, `a7/a.java:159`). Builders set it to **1**
    (`j8/c.java:31`, `j8/e.java:22`, etc.). **Despite the name, no payload encryption is
    performed in the AOA command path** — `B()`/`c7.e` contain no cipher. Treat it as a
    constant `0x01` flag for normal commands. *(Firmware/anti-tamper streams — type ≠ 0 —
    may differ; out of scope.)*
  - bits 2–4 `f784d` (`m()`); builders leave it 0.
  - bits 5–7 `f785e` (`j()`, `a7/a.java:127`); builders set `j((byte)0)`.
  - **Net: byte[3] = 0x01 for all normal app→drone commands.**
  - ⚠️ Encode/decode asymmetry on the middle/high bits: encode masks the value in place
    (`f784d & 0x1C`) while decode right-shifts (`(b>>2)&7`, `a7/d.java:232`). Irrelevant
    while these bits are 0 (always, for the app), but a reimplementation should keep them
    0 and not rely on round-tripping nonzero values. *(Flagged §12.)*
- **srcId byte[4]** = sending module (`q()`/`e()`, `a7/a.java:95,81,155`).
- **destId byte[5]** = receiving module (`i()`/`c()`, `a7/a.java:96,73,123`).
  - TX (app→drone): `srcId = MODULE_GCS (7)`, `destId = target module`
    (`j8/c.java:33-34`, `j8/e.java:24-26`, `j8/a.java:38-39`, `j8/b.java:27-28`,
    `j8/d.java:23-24`).
  - RX (drone→app): `srcId = originating module`, `destId = MODULE_GCS (7)`
    (dispatch in `m8/a.java:1260-1317`).
- **reserve2 / reserve3** bytes[6..7]: set to 0 by builders (never written); echoed/parsed
  as `f788h`/`f789i` (`a7/a.java:97-98,143-149`; parse `a7/d.java:249-256`).
- **seq** bytes[8..9]: `u16 LE` (`a7/a.java:99-101`; parse `a7/d.java:259-265`). See §7.
- **crcHeader** bytes[10..11]: `u16 LE`, CRC-16/MCRF4XX over the **first 10 header
  bytes** (`a7/a.java:102`). See §5.1. *(The inner parser reads it but does not verify
  it — `a7/d.java:268-275` stores it and moves on. Only the outer additive checksum and
  the inner frame CRC32 are verified.)*
- **crcFrame** bytes[12..15]: `u32 LE`, custom CRC-32 over the **payload only**
  (`a7/b.java:53` computes it via `a7.c.h(..)`; stored at `a7/a.java:107-111`). This one
  **is** verified on RX (`a7/d.java:304-308`). See §5.2.
- **payload**: the application bytes (command id + args, or telemetry). Written verbatim
  (`j8/k.java:42` `this.f21258m.c().p(bArr)`).

### 4.3 Assembler `a7.b.f()` (build order)

`a7/b.java:47-56`:

```java
int r10 = payload.position();        // payload length
header.l(r10);                        // stash payload len (unused in f())
byte[] frame = new byte[r10 + 16];
header.k(r10 + 16);                   // total length field
header.g( payload.h(frame, 16) );     // copy payload to frame[16..], set crcFrame = CRC32(payload)
System.arraycopy(header.f(), 0, frame, 0, 16);   // build & copy 16-byte header (incl. both CRCs)
return frame;
```

Note the ordering: `crcFrame` (payload CRC32) is computed and stored **before**
`header.f()` runs, so the header's bytes[12..15] already contain it; the header CRC16 is
then computed over bytes[0..9] inside `f()`.

---

## 5. Checksums

### 5.1 Header CRC — CRC-16/MCRF4XX (`n7.g`, CRCUtil)

`n7/g.java:6-18`:

```java
int a(int data, int crc) {           // per-byte update
    int x  = data ^ (crc & 0xFF);
    int x4 = (x ^ (x << 4)) & 0xFF;
    return (x4 >> 4) ^ ((crc >> 8) ^ (x4 << 8) ^ (x4 << 3));
}
int b(byte[] buf, int len) {         // crc over buf[0..len)
    int crc = 0xFFFF;
    for (i=0; i<len; i++) crc = a(buf[i] & 0xFF, crc & 0xFFFF);
    return crc;
}
```

This is avr-libc's `_crc_ccitt_update`, which is the **reflected** CCITT CRC, i.e.
**CRC-16/MCRF4XX**: poly `0x1021` reflected (`0x8408`), init `0xFFFF`, refin/refout true, no
final XOR, check value `0x6F91` for `"123456789"`. *(An earlier revision of this document
called it CCITT-FALSE; that was wrong, and was caught by running the app's own code.)*

Called as `g.b(headerBytes, 10)` — **over header bytes[0..9]** (start, ver+len, flags,
src, dst, res2, res3, seqLo, seqHi) — result stored little-endian at bytes[10..11]
(`a7/a.java:102-106`).

### 5.2 Frame CRC — custom word-wise CRC-32 (`b7.a`, ByteArrayToIntArray)

`b7/a.java:18-64`. Polynomial table built with **`0x04C11DB7`** (`79764919`,
`b7/a.java:52`), MSB-first, init `0xFFFFFFFF` (`i12 = -1`, `b7/a.java:24`), **no final
XOR**, **no reflection**. BUT with a non-standard twist: input is consumed **4 bytes at a
time, little-endian, and each 4-byte word's bytes are fed high-to-low (i.e. byte order
within each word is reversed)**:

```java
int a(byte[] buf, int len) {
    int crc = 0xFFFFFFFF;
    int words = len / 4;
    for (w = 0; w < words; w++) {
        int le = LE32(buf, w*4);                  // b7/a.java:27, c(): little-endian u32
        for (k = 3; k >= 0; k--)                  // process bytes b3,b2,b1,b0
            crc = (crc << 8) ^ table[((crc >> 24) ^ ((le >> (k*8)) & 0xFF)) & 0xFF];
    }
    int rem = len % 4;                            // tail: zero-pad to 4 bytes, same treatment
    if (rem > 0) {
        byte[] t = new byte[4];
        for (i=0;i<rem;i++) t[i] = buf[words*4 + i];
        int le = LE32(t, 0);
        for (k = 3; k >= 0; k--)
            crc = table[(byte)((crc >> 24) ^ ((le >> (k*8)) & 0xFF)) & 0xFF] ^ (crc << 8);
    }
    return crc;
}
```

where `LE32(b,i) = (b[i+3]<<24)|(b[i+2]<<16)|(b[i+1]<<8)|b[i]` (`b7/a.java:62-63`) and
`table` is the standard forward `0x04C11DB7` table (`b7/a.java:46-60`).

**Net effect:** it is CRC-32/MPEG-2 (poly `0x04C11DB7`, init `0xFFFFFFFF`, refin/refout
false, xorout 0) computed over the payload **with each aligned 4-byte group
byte-swapped**, and the final partial group zero-padded up to 4 bytes. A faithful
reimplementation must reproduce the per-word reversal and the zero-padded tail exactly —
you cannot substitute an off-the-shelf CRC-32 without the byte-swap. Computed over the
**payload bytes only** (`a7.c.h()`/`a7.c.i()` pass just the payload, `a7/c.java:49-62`).

> `b7/a.java:66-84` also has a `CRC32` (java.util.zip, standard reflected CRC-32)
> `d(String)` helper, but that is for **file** checksums (firmware), **not** the frame
> CRC. Do not confuse the two.

---

## 6. Addressing — module ids (`j8.h.a`, X8BaseCmd)

`j8/h.java:15-39` — enum `h.a`, used by ordinal. Full table:

| id | name | | id | name |
|---|---|---|---|---|
| 0 | MODULE_IDLE | | 12 | MODULE_SV_FW |
| 1 | MODULE_UAV | | 13 | MODULE_RC |
| 2 | MODULE_FC (flight controller) | | 14 | MODULE_REPEATER_VEHICLE |
| 3 | MODULE_CAMERA | | 15 | MODULE_BATTERY |
| 4 | MODULE_OPTFLOW | | 16 | MODULE_REPEATER_RC |
| 5 | MODULE_OBSAVOID | | 17 | MODULE_NFZ (no-fly-zone) |
| 6 | MODULE_HTTP | | 18 | MODULE_ESC |
| 7 | **MODULE_GCS** (the app / ground station) | | 19 | MODULE_SERVO |
| 8 | MODULE_GIMBAL | | 20 | MODULE_Default0X14 |
| 9 | MODULE_BLACKBOX | | 21 | MODULE_Default0X15 |
| 10 | MODULE_CV | | 22 | MODULE_ULTRASONIC |
| 11 | MODULE_SV_DWN | | | |

The app is always `MODULE_GCS (7)`. These ids populate the `srcId`/`destId` header bytes
(§4.2). The payload's own first two bytes are the command selector (§9), **not** a module
id.

---

## 7. Sequence numbers

`j8/h.java:41-49` (X8BaseCmd constructor): a process-wide static counter `f21228b` is
read into the per-command field `f21229a`, then incremented; it wraps to 0 at 32766:

```java
h() { this.f21229a = (short) f21228b;  if (++f21228b == 32766) f21228b = 0; }
```

Each command object is constructed fresh per send (e.g. `new j8.c(...)` in `z8/c.java`),
so **every command gets the next sequence value, 0..32765, monotonically wrapping.** The
builder copies it into the header via `bVar.a().p(this.f21229a)`
(`j8/c.java:35`, `j8/e.java:27`, etc.), landing at header bytes[8..9] (§4.2). The drone
**echoes the same seq** in its reply, which is used for ACK matching (§10).

---

## 8. RX — framing, parsing, dispatch

### 8.1 Outer de-wrap: `y6.d` (NetDecoder, "Decoder1")

`y6/d.java:45-119`. Fed from the read thread (`y6/a.java:117-120`). Accumulates into a
32 KB `ByteBuf` and emits zero or more `(payload, type)` records per call:

```
loop:
  scan forward until the next byte == 0xAE (start)              y6/d.java:56-62
  need ≥5 bytes else discardReadBytes and wait                 y6/d.java:63-67
  u16 = LE(bytes[1..2]); version = u16 & 0x0F                   y6/d.java:68-70
  if version == 1:  hdr=5; len = (u16 & 0xFFF0) >> 4            y6/d.java:84-86
  elif version == 2: hdr=8; need ≥8; len = LE(bytes[4..5])      y6/d.java:74-82
  else: skip 1 byte, continue                                  y6/d.java:72-73
  validate hdr ≤ len ≤ 8692, else skip 1 byte                  y6/d.java:87,110-112
  need ≥len bytes buffered else discardReadBytes and wait       y6/d.java:88-91
  verify additive checksum: sum(bytes[0..hdr-2]) & 0xFF == bytes[hdr-1]
                                       else "crc header error", skip 1   y6/d.java:92-100
  emit record: type = bytes[3]; payload = bytes[hdr .. len)     y6/d.java:102-108
  skip len bytes; repeat
```

Each emitted record `a` carries `a.e()` = **outer type byte** and `a.d()` = the inner
FmLink4 frame bytes (`y6/d.java:32-38,104-107`).

### 8.2 Type routing

`AOAConnect.v(buf,n)` (`y6/a.java:203-233`) hands each record to the `q6.i` handler
(`l8.c`, DataChanel) with a type code. For **every** record it calls `handler.a(bytes, 0)`
(`y6/a.java:228-231`), and additionally `handler.a(bytes, type)` for types 2/6/7
(`y6/a.java:207-226`). `l8.c.a(bytes, type)` (`l8/c.java:21-60`) routes:

| type | target | meaning |
|---|---|---|
| 0 | `m8.a.b()` | **FmLink4** command/telemetry → inner parser (§8.3) |
| 2 | `m8.d.a()` | (data stream) |
| 5 | `v6.c.g().h()` | (stream to NoticeManager) |
| 6 | `m8.b.a()` | media-down |
| 7 | `m8.c.a()` | firmware-up |

(The spurious extra `type 0` call for non-zero records is harmless: non-FmLink4 bytes
simply never pass the `0xFE`+CRC32 checks in the inner parser and are dropped.)

### 8.3 Inner parse: `a7.d` (Parser4) — byte-at-a-time state machine

`m8.a.b(byte[])` streams each byte into `a7.d.b(int)` (`m8/a.java:1320-1329`). `a7.d`
(`a7/d.java:189-324`) is a per-byte FSM over states
`Idle→Ver→Len→TypeAndRes1Encry→SrcId→DestId→Reserve2→Reserve3→Seq1→Seq2→CrcHeader1→
CrcHeader2→CrcFrame1..4→PlayLoad`:

```
Idle:   if byte == 0xFE -> new LinkPacket4; state=Ver                 a7/d.java:192-198
Ver:    f=byte&0xFF; version = f & 0x1F; if version!=4 -> Idle        a7/d.java:200-219
        else state=Len
Len:    u16 = (byte<<8)|prev; length=(u16>>6)&0x1FF;                  a7/d.java:220-227
        payloadLen = length-16; state=TypeAndRes1Encry
flags:  encrypt=b&3; f784d=(b>>2)&7; f785e=(b>>5)&7; -> SrcId         a7/d.java:228-238
SrcId:  srcId=b -> DestId                                             a7/d.java:239-243
DestId: destId=b -> Reserve2                                          a7/d.java:244-248
Res2:   -> Reserve3        Res3: -> Seq1                              a7/d.java:249-257
Seq1/2: seq = u16 LE       -> CrcHeader1                              a7/d.java:259-266
CrcHdr1/2: crcHeader = u16 LE (stored, NOT verified) -> CrcFrame1     a7/d.java:268-276
CrcFrame1..4: crcFrame = u32 LE -> PlayLoad                           a7/d.java:277-293
PlayLoad: append byte to payload buffer; when payloadLen bytes in:   a7/d.java:294-313
          if CRC32(payload) == crcFrame -> emit frame                a7/d.java:304-308
          state=Idle
```

Only a frame whose recomputed payload CRC32 (`a7.c.i()`) equals the header `crcFrame`
is emitted (`a7/d.java:304-308`); otherwise it is silently dropped and the FSM returns to
`Idle`. (The `f804r` flag is the "valid frame ready" signal; the method returns the
`LinkPacket4` only when set — `a7/d.java:320-323`.)

### 8.4 Dispatch of a decoded frame: `m8.a.p()`

`m8/a.java:1253-1318`:

```java
int cmd  = frame.payload[1] & 0xFF;      // a7.c.k()  (m8/a.java:1258)  "msgId"
int ack0 = frame.payload[0] & 0xFF;      // a7.c.f()  (m8/a.java:1259)  "class/ack byte"
int dst  = frame.header.destId;          // a7.a.c()  (m8/a.java:1260)
if (dst != MODULE_GCS) {                 // not for the app
    if (dst == MODULE_RC || dst == MODULE_REPEATER_RC)   // relay toward RC
        v6.e.e().k( new c7.e(0).wrap(frame.raw) );        // m8/a.java:1268-1270
    else  l8.g.f().p(frame.raw);                          // m8/a.java:1264
    return;
}
// for the app:
f23702b.e(ack0, cmd, frame.header.seq, frame);   // ACK match (§10)   m8/a.java:1273
switch (frame.header.srcId) {                    // route by SOURCE module
    MODULE_CAMERA -> d(cmd, ack0, frame);         // m8/a.java:1274-1275
    MODULE_FC     -> f(cmd, ack0, frame);         // m8/a.java:1286-1288
    MODULE_BATTERY-> c(...); MODULE_GIMBAL-> h(...); MODULE_RC-> k(...);
    MODULE_NFZ-> i(...); MODULE_CV-> n(...); MODULE_ESC-> e(...);
    MODULE_ULTRASONIC-> m(...); MODULE_OPTFLOW-> j(...);
    MODULE_REPEATER_RC-> l(...); MODULE_REPEATER_VEHICLE-> g(...);   // m8/a.java:1278-1317
}
```

Each module handler then switches on `cmd` (payload[1]) to decode a specific response
struct (e.g. `m8/a.java:373-605`). Decoded objects are posted to UI listeners via
`v6.c` (NoticeManager) `y()/a()/j()/k()` on the main-looper handler
(`v6/c.java:106-109,143-165,226-229`; `m8/a.java:1244-1251`).

---

## 9. Payload command selector (just enough for ACK/addressing)

The first two payload bytes are the command selector that the drone echoes back:

- **payload[0]** = a "class"/group byte. In builders these are literal constants the
  decompiler rendered as netty names, e.g. `BinaryMemcacheOpcodes.STAT` = `0x10`
  (query/status), `BinaryMemcacheOpcodes.APPEND` = `0x0E`, `2` for camera, etc.
  (`j8/e.java:36,44`, `j8/a.java:59`, `j8/d.java:34`). Exposed on the frame as `a7.c.f()`.
- **payload[1]** = the command/message id within that module+class (`a7.c.k()`).

Example TX payloads (`E(byte[])`, `j8/k.java:38-43`): `{0x10,0xB2,0,0,0,0x14}` (FC),
`{0x10,0x01,0,0}` (camera), `{2,5}` (camera short), `{0x0E,0x27,0,0,…}` (repeater).
Exact command catalogue is command-layer (out of scope).

---

## 10. ACK matching & retransmission (`u6.a`, RetransmissionThread)

`u6/a.java`. A command that expects a reply is added to a pending deque when the write
thread sends it (`y6/a.java:157-159`, guarded by `cmd.r()==true`, default true
`q6/a.java:32,83`). The deque entry is the `q6.a` command object.

**Match key = (payload[0], payload[1], seq)** — `u6/a.java:34-62`:

```java
boolean c(int p0, int p1, int seq, LinkPacket4 frame) {
    for (cmd : pending)
        if (cmd.f()==p0 && cmd.g()==p1 && cmd.n()==seq) {   // u6/a.java:43
            frame.setResponseCallback(cmd.p());             // u6/a.java:51 (s6.d)
            frame.setUiCallback(cmd.d());                    // u6/a.java:52 (c7.c)
            pending.remove(cmd); return true;
        }
    return false;
}
```

- `cmd.f()` / `cmd.g()` are the command's stored payload[0]/payload[1], set at build time
  in `k.E()`: `h(bArr[0])`, `i(bArr[1])` (`j8/k.java:39-40`; getters `b7/c.java:13-19`).
- `cmd.n()` is the command's seq, set in `k.E()` via `x(header.seq)`
  (`j8/k.java:41` → `q6/a.java:107-109,67-69`).
- The incoming side supplies these from the decoded frame in `m8.a.p()`:
  `f23702b.e(payload[0], payload[1], header.seq, frame)` (`m8/a.java:1273`), which reaches
  `u6.a.c(..)` via `AOAConnect.e(..)` (`y6/a.java:291-298`, the `s6.e` interface).

**So an ACK is matched by (payload[0], payload[1], sequence) — NOT by the module id.**
The module (src/dest) is used only to route the decoded response to the right parser
(§8.4), and all three key fields are byte-for-byte echoed by the drone.

**Retransmission** (`u6/a.java:64-98`): a background thread resends any pending command
whose `uptime - lastSent ≥ timeout` (`o()`, default **500 ms**, `q6/a.java:26,72`) up to
`q()` retries (default **5**, `q6/a.java:28,80`). After exhausting retries it reports
failure via `s6.c.a(..)` and drops the command (`u6/a.java:73-89`). Idle sleep 400 ms
(`u6/a.java:92-93`). Started in `AOAConnect.y()` (`y6/a.java:384-387`).

---

## 11. Connection lifecycle & heartbeat

### 11.1 Connect handshake (transport)

The only transport-level "handshake" is the AOA sequence of §1.2–§1.3:
attach → permission (`com.google.android.DemoKit.action.USB_PERMISSION`) →
`openAccessory` → `write(0)` → start read/write/retransmission threads + register session
(`y6/a.java:378-388`; `p8/a.java:46-57`). **No login, password, pairing or activation
exchange happens at the transport/framing layer** after `write(0)`; normal FmLink4
command frames begin flowing immediately. Any activation/binding logic that exists is
command-layer (HTTP/`MODULE_HTTP` or specific FC/RC commands) and was not found in the
transport code paths examined.

### 11.2 Heartbeat / keepalive

- **No dedicated transport-level keepalive frame was found.** The AOA link has no
  periodic "ping" in `y6.a`/`l8.*`/`c7.*`/`a7.*`. The link is kept warm by continuous
  application traffic (RC-stick commands, 1 Hz status-poll timers in the presenters, e.g.
  `z8/o.java:252`, `z8/m.java:320`, `z8/l.java:252`) plus the retransmission thread.
- **The drone pushes an FC heartbeat/telemetry frame** that the app consumes: interface
  `w8.e` is literally `FcHeartListener` (`w8/e.java:5-8`, method `I0(v2, boolean)`), fed
  from the FC-telemetry dispatch `y8.l.G()` → `I0(fcState, ...)` (`y8/l.java:230-231`)
  whenever a `MODULE_FC` frame is decoded (`m8/a.java:1286-1288`). So "heartbeat" here is
  an inbound FC state stream, not an app-originated keepalive.
- The exact FC heartbeat command id and whether the app sends any explicit periodic
  keepalive to the FC/RC were **not pinned down** in the transport layer — flagged as a
  gap (§12). Watch the live USB capture for the dominant periodic frame to confirm.

### 11.3 Disconnect

Read-thread `IOException` → `y6.b.a(1)` → `CommunicationManager.e()` tears down the
session (`y6/a.java:125-128`, `l8/a.java:65-66,108-127`). `AOAConnect.f()` stops threads,
the retransmission thread and closes the three streams (`y6/a.java:300-325,349-372`).

---

## 12. Encode & decode pseudocode

### 12.1 Encode (command payload → USB bytes)

```
function build_fmlink4(srcId, destId, seq, payload):            # a7.b.f()
    L = len(payload)
    total = L + 16                                              # 9-bit field, require total<=511
    crc32 = fimi_crc32(payload)                                 # §5.2 (word-swapped MPEG-2)
    hdr = bytearray(16)
    hdr[0]  = 0xFE
    v16     = (4 & 0x1F) | ((total & 0x1FF) << 6)               # version=4
    hdr[1]  = v16 & 0xFF;  hdr[2] = (v16 >> 8) & 0xFF
    hdr[3]  = 0x01                                              # encrypt=1, rest 0
    hdr[4]  = srcId & 0xFF                                      # e.g. MODULE_GCS=7
    hdr[5]  = destId & 0xFF
    hdr[6]  = 0x00; hdr[7] = 0x00                               # reserve2, reserve3
    hdr[8]  = seq & 0xFF;  hdr[9] = (seq >> 8) & 0xFF
    crc16   = crc_ccitt_false(hdr[0:10])                        # §5.1
    hdr[10] = crc16 & 0xFF;  hdr[11] = (crc16 >> 8) & 0xFF
    hdr[12] = crc32        & 0xFF;  hdr[13] = (crc32 >> 8)  & 0xFF
    hdr[14] = (crc32 >> 16)& 0xFF;  hdr[15] = (crc32 >> 24) & 0xFF
    return hdr + payload

function wrap_usb(inner, type=0):                               # c7.e.b(), AOA/UDP only
    total = len(inner) + 5                                      # 12-bit field
    out = bytearray(5)
    out[0] = 0xAE
    out[1] = (1 & 0x0F) | ((total & 0x0F) << 4)                 # version=1
    out[2] = (total >> 4) & 0xFF
    out[3] = type & 0xFF                                        # 0 = FmLink4
    out[4] = (out[0]+out[1]+out[2]+out[3]) & 0xFF               # additive checksum
    return out + inner

function send_command(destId, payload):
    seq = next_seq()                                            # §7, 0..32765 wrap
    inner = build_fmlink4(MODULE_GCS=7, destId, seq, payload)
    bytes = wrap_usb(inner, type=0)                             # AOA: always wrap
    usb_out.write(bytes)
    register_pending(key=(payload[0], payload[1], seq))        # for ACK/retransmit §10
```

### 12.2 Decode (USB bytes → command payload)

```
# Outer (y6.d): scan a byte stream, peel 0xAE wrappers
on usb_in bytes -> feed into ring buffer; then:
  loop:
    drop bytes until buffer[0]==0xAE; need >=5 bytes
    u16 = LE16(buffer[1:3]); version = u16 & 0x0F
    if version==1: hdr=5; total=(u16>>4)&0x0FFF
    elif version==2: hdr=8; need>=8; total=LE16(buffer[4:6])
    else: drop 1; continue
    require hdr<=total<=8692 and buffered>=total  (else wait/drop)
    require (sum(buffer[0:hdr-1]) & 0xFF) == buffer[hdr-1]      # checksum
    type  = buffer[3]; inner = buffer[hdr:total]; consume total
    if type==0: feed inner byte-by-byte to fmlink4_parser()    # §8.3
    else: route to media/fw/json handler by type               # §8.2

# Inner (a7.d): byte-at-a-time FSM
function fmlink4_parser(byte):   # see §8.3 state table
    ... accumulate header fields ...
    on last payload byte:
        if fimi_crc32(payload) == crcFrame_from_header:        # verify
            emit frame(srcId,destId,seq,payload)
        reset to Idle

on frame:                                                      # m8.a.p()
    if frame.destId != MODULE_GCS: relay/drop; return
    match_ack(frame.payload[0], frame.payload[1], frame.seq, frame)  # §10
    dispatch_by_module(frame.srcId, frame.payload[1], frame.payload[0], frame)
```

---

## 13. Gaps & uncertainties (not guessed)

1. **`write(0)` purpose** (`y6/a.java:192`): confirmed it is sent and gates the connected
   flag, but the code gives no explicit reason. Likely an AOA stream wake/sync byte.
   Verify against a live capture.
2. **Explicit app→drone heartbeat/keepalive**: none found at the transport layer. The
   drone pushes FC telemetry (`FcHeartListener`, `w8/e.java`); the app relies on
   continuous traffic + retransmission. The specific keepalive command id (if any) is
   unconfirmed — command-layer.
3. **flags byte[3] middle/high bits** (`f784d` bits2-4, `f785e` bits5-7): semantics
   unknown; builders always leave them 0. Encode uses in-place mask, decode right-shifts
   (`a7/a.java:92` vs `a7/d.java:230-233`) — a round-trip inconsistency that is moot while
   the bits are 0. Keep them 0.
4. **"encrypt" field = 1 but no cipher** in the AOA command path (`c7.e`/`a7.*` contain no
   encryption). Firmware/anti-tamper streams (outer type ≠ 0) are a separate concern and
   out of scope .
5. **Header CRC16 is not verified on RX** (`a7/d.java:268-276` stores but never checks);
   only the outer additive checksum and the inner frame CRC32 are enforced. A
   reimplementation must still *produce* a correct CRC16 (the RC may check it).
6. **9-bit inner length (≤511)**: a single FmLink4 frame is capped at 495 payload bytes.
   Larger transfers (media/firmware) use the other outer types (2/5/6/7), whose internal
   chunking was not analysed here.
7. **reserve2/reserve3** always 0 from the app; unknown whether the RC/drone ever uses
   them.
