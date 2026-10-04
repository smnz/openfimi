package io.github.smnz.openfimi.bridge

/**
 * The bits of the FIMI wire format the bridge itself needs (a port of
 * openfimi's crc.py / framing.py): encoding a few FmLink4 commands, encoding
 * bridge notices, and finding outer-frame boundaries so injected frames never
 * land inside another frame.
 *
 *     0xAE | ver/len | TYPE | sum8 | <body>              outer wrapper
 *     0xFE | ver/len | flags | src | dst | r2 | r3 | seq | crc16 | crc32 | payload
 *                                                         inner FmLink4 (TYPE 0)
 */
object Wire {
    const val OUTER_SYNC = 0xAE.toByte()
    const val INNER_SYNC = 0xFE.toByte()
    const val OUTER_MAX_LEN = 8692
    const val INNER_HEADER_LEN = 16
    const val INNER_MAX_LEN = 0x1FF

    const val TYPE_FMLINK = 0
    const val TYPE_VIDEO = 2
    /** openfimi's own stream type: a UTF-8 JSON notice from the bridge (never sent by FIMI hardware). */
    const val TYPE_BRIDGE_NOTICE = 0x40

    const val GCS = 7
    const val FC = 2

    /** Client sequence numbers run 0..32765. */
    const val SEQ_WRAP = 32766

    // -- checksums ---------------------------------------------------------------

    /** CRC-16/MCRF4XX: reflected poly 0x8408, init 0xFFFF, no xorout (check 0x6F91). */
    fun crc16(data: ByteArray, off: Int = 0, len: Int = data.size - off): Int {
        var crc = 0xFFFF
        for (k in off until off + len) {
            var x = (data[k].toInt() and 0xFF) xor (crc and 0xFF)
            x = (x xor (x shl 4)) and 0xFF
            crc = ((crc ushr 8) xor (x shl 8) xor (x shl 3) xor (x ushr 4)) and 0xFFFF
        }
        return crc
    }

    private val CRC32_TABLE = IntArray(256) { i ->
        var c = i shl 24
        repeat(8) { c = if (c and 0x80000000.toInt() != 0) (c shl 1) xor 0x04C11DB7 else c shl 1 }
        c
    }

    /**
     * The frame CRC: CRC-32/MPEG-2 over the payload with each aligned 4-byte
     * little-endian word fed most-significant byte first, tail zero-padded.
     */
    fun crc32(data: ByteArray, off: Int = 0, len: Int = data.size - off): Int {
        var crc = -1
        var i = 0
        while (i < len) {
            for (j in 3 downTo 0) {
                val b = if (i + j < len) data[off + i + j].toInt() and 0xFF else 0
                crc = (crc shl 8) xor CRC32_TABLE[((crc ushr 24) xor b) and 0xFF]
            }
            i += 4
        }
        return crc
    }

    /** 8-bit additive checksum of the outer header. */
    fun additive8(data: ByteArray, off: Int = 0, len: Int = data.size - off): Int {
        var s = 0
        for (k in off until off + len) s += data[k].toInt() and 0xFF
        return s and 0xFF
    }

    // -- encoding ----------------------------------------------------------------

    fun encodeInner(src: Int, dst: Int, seq: Int, payload: ByteArray, flags: Int = 0x01): ByteArray {
        val total = payload.size + INNER_HEADER_LEN
        require(total <= INNER_MAX_LEN) { "payload too long for one FmLink4 frame" }
        val verlen = 4 or (total shl 6)
        val out = ByteArray(total)
        out[0] = INNER_SYNC
        out[1] = verlen.toByte()
        out[2] = (verlen ushr 8).toByte()
        out[3] = flags.toByte()
        out[4] = src.toByte()
        out[5] = dst.toByte()
        out[6] = 0
        out[7] = 0
        out[8] = seq.toByte()
        out[9] = (seq ushr 8).toByte()
        val c16 = crc16(out, 0, 10)
        out[10] = c16.toByte()
        out[11] = (c16 ushr 8).toByte()
        val c32 = crc32(payload)
        for (k in 0 until 4) out[12 + k] = (c32 ushr (8 * k)).toByte()
        System.arraycopy(payload, 0, out, INNER_HEADER_LEN, payload.size)
        return out
    }

    fun encodeOuter(body: ByteArray, type: Int): ByteArray {
        val total = body.size + 5
        require(total <= 0xFFF) { "outer frame too long for the 12-bit length field" }
        val w = 0x01 or (total shl 4)
        val out = ByteArray(total)
        out[0] = OUTER_SYNC
        out[1] = w.toByte()
        out[2] = (w ushr 8).toByte()
        out[3] = type.toByte()
        out[4] = additive8(out, 0, 4).toByte()
        System.arraycopy(body, 0, out, 5, body.size)
        return out
    }

    /** A complete GCS -> flight controller command with an empty body, ready for the RC. */
    fun command(group: Int, msg: Int, seq: Int): ByteArray =
        encodeOuter(encodeInner(GCS, FC, seq, byteArrayOf(group.toByte(), msg.toByte())), TYPE_FMLINK)

    val MISSION_STOP = 0x03 to 0x23
    val RETURN_HOME = 0x03 to 0x1A
    val FLY_TO_EXIT = 0x03 to 0x33

    fun notice(json: String): ByteArray = encodeOuter(json.toByteArray(Charsets.UTF_8), TYPE_BRIDGE_NOTICE)

    // -- decoding ----------------------------------------------------------------

    /**
     * Validates an outer header starting at [off] (data[off] must be 0xAE) using
     * the bytes up to [end]. Returns the frame's total length (header included)
     * if valid, 0 if more bytes are needed to decide, or -1 if it is not a header.
     */
    fun outerHeader(data: ByteArray, off: Int, end: Int): Int {
        val n = end - off
        if (n < 3) return 0
        val w = (data[off + 1].toInt() and 0xFF) or ((data[off + 2].toInt() and 0xFF) shl 8)
        val hl: Int
        val total: Int
        when (w and 0x0F) {
            1 -> {
                if (n < 5) return 0
                hl = 5
                total = (w ushr 4) and 0xFFF
            }
            2 -> {
                if (n < 8) return 0
                hl = 8
                total = (data[off + 4].toInt() and 0xFF) or ((data[off + 5].toInt() and 0xFF) shl 8)
            }
            else -> return -1
        }
        if (total < hl || total > OUTER_MAX_LEN) return -1
        if (additive8(data, off, hl - 1) != (data[off + hl - 1].toInt() and 0xFF)) return -1
        return total
    }

    /** Header length of a header already accepted by [outerHeader]. */
    fun outerHeaderLen(data: ByteArray, off: Int): Int = if (data[off + 1].toInt() and 0x0F == 2) 8 else 5

    /** One decoded FmLink4 frame (only what the bridge looks at). */
    class Fm(val src: Int, val dst: Int, val seq: Int, val payload: ByteArray) {
        val group get() = if (payload.isNotEmpty()) payload[0].toInt() and 0xFF else -1
        val msg get() = if (payload.size > 1) payload[1].toInt() and 0xFF else -1
        /** The 12-bit result in payload[2..3]; on ACKs, 0 means OK. */
        val result get() = if (payload.size < 4) 0
            else ((payload[3].toInt() and 0xFF) shl 4) or ((payload[2].toInt() and 0xFF) ushr 4)
    }
}

/**
 * Streaming decoder for FmLink4 frames carried in TYPE 0 bodies (frames may
 * be split across or batched within records), as openfimi's InnerDecoder.
 */
class InnerDecoder {
    private var buf = ByteArray(1024)
    private var len = 0

    fun feed(data: ByteArray): List<Wire.Fm> {
        if (len + data.size > buf.size) buf = buf.copyOf(maxOf(buf.size * 2, len + data.size))
        System.arraycopy(data, 0, buf, len, data.size)
        len += data.size
        val out = ArrayList<Wire.Fm>(1)
        var i = 0
        while (true) {
            while (i < len && buf[i] != Wire.INNER_SYNC) i++
            if (len - i < 3) break
            val verlen = (buf[i + 1].toInt() and 0xFF) or ((buf[i + 2].toInt() and 0xFF) shl 8)
            val total = (verlen ushr 6) and 0x1FF
            if (verlen and 0x1F != 4 || total < Wire.INNER_HEADER_LEN) { i++; continue }
            if (len - i < total) break
            val crc = (buf[i + 12].toInt() and 0xFF) or ((buf[i + 13].toInt() and 0xFF) shl 8) or
                ((buf[i + 14].toInt() and 0xFF) shl 16) or ((buf[i + 15].toInt() and 0xFF) shl 24)
            val plen = total - Wire.INNER_HEADER_LEN
            if (Wire.crc32(buf, i + Wire.INNER_HEADER_LEN, plen) != crc) { i++; continue }
            out += Wire.Fm(
                src = buf[i + 4].toInt() and 0xFF,
                dst = buf[i + 5].toInt() and 0xFF,
                seq = (buf[i + 8].toInt() and 0xFF) or ((buf[i + 9].toInt() and 0xFF) shl 8),
                payload = buf.copyOfRange(i + Wire.INNER_HEADER_LEN, i + total),
            )
            i += total
        }
        System.arraycopy(buf, i, buf, 0, len - i)
        len -= i
        if (len > 4096) len = 0 // never grow without bound on garbage
        return out
    }
}

/**
 * Tracks outer-frame boundaries in the RC -> client stream without buffering
 * it, so the relay can insert a bridge notice between two frames.
 *
 * [feed] consumes bytes; with `stopAtBoundary` it stops at the first position
 * where a frame may be inserted ([atBoundary]: the previous frame is complete
 * and the next one has not started, or a validated header is about to start).
 * Bodies of the stream types accepted by [collect] are handed to [onRecord].
 * Out of sync (garbage), it rescans for 0xAE with a valid header checksum.
 */
class OuterTracker(
    private val collect: (Int) -> Boolean = { false },
    private val onRecord: (type: Int, body: ByteArray) -> Unit = { _, _ -> },
) {
    private val hdr = ByteArray(8)
    private var hdrLen = 0
    private var bodyLeft = 0
    private var type = 0
    private var body: ByteArray? = null
    private var bodyPos = 0
    private var synced = true
    var skipped = 0L
        private set

    val atBoundary: Boolean get() = synced && hdrLen == 0 && bodyLeft == 0

    /** Consumes data[off until off+len]; returns how many bytes were consumed. */
    fun feed(data: ByteArray, off: Int, len: Int, stopAtBoundary: Boolean = false): Int {
        var i = off
        val end = off + len
        while (i < end) {
            if (bodyLeft > 0) {
                val k = minOf(bodyLeft, end - i)
                body?.let { System.arraycopy(data, i, it, bodyPos, k) }
                bodyPos += k
                bodyLeft -= k
                i += k
                if (bodyLeft == 0) finishFrame()
                continue
            }
            if (hdrLen == 0) {
                if (!synced) {
                    // Scanning: skip to the next 0xAE, and if its whole header is
                    // here and valid, the position before it is a boundary.
                    while (i < end && data[i] != Wire.OUTER_SYNC) { i++; skipped++ }
                    if (i == end) break
                    val h = Wire.outerHeader(data, i, end)
                    if (h < 0) { i++; skipped++; continue }
                    if (h > 0) synced = true
                    // h == 0: the header runs past this chunk; collect it below.
                }
                if (stopAtBoundary && atBoundary) break
                if (data[i] != Wire.OUTER_SYNC) {
                    synced = false
                    continue
                }
            }
            hdr[hdrLen++] = data[i++]
            resolveHeader()
        }
        return i - off
    }

    private fun resolveHeader() {
        while (hdrLen > 0) {
            val total = Wire.outerHeader(hdr, 0, hdrLen)
            if (total == 0) return
            if (total > 0) {
                val hl = Wire.outerHeaderLen(hdr, 0)
                if (hdrLen < hl) return
                type = hdr[3].toInt() and 0xFF
                hdrLen = 0
                synced = true
                bodyLeft = total - hl
                bodyPos = 0
                body = if (collect(type)) ByteArray(bodyLeft) else null
                if (bodyLeft == 0) finishFrame()
                return
            }
            // Not a header: drop its first byte and rescan what we hold.
            synced = false
            skipped++
            var k = 1
            while (k < hdrLen && hdr[k] != Wire.OUTER_SYNC) { k++; skipped++ }
            System.arraycopy(hdr, k, hdr, 0, hdrLen - k)
            hdrLen -= k
        }
    }

    private fun finishFrame() {
        val b = body ?: return
        body = null
        onRecord(type, b)
    }
}

/**
 * Re-frames one client's TCP byte stream into whole outer frames, so that a
 * write to the RC never ends inside a frame (and a frame the bridge injects
 * cannot land mid-frame). Bytes that are not a frame are passed straight on.
 */
class ClientFramer {
    private var buf = ByteArray(16384)
    private var len = 0
    /** When the oldest held (incomplete) byte arrived, or 0 if nothing is held. */
    var heldSince = 0L
        private set
    val held: Int get() = len

    /** Adds data; returns the bytes that can go to the RC now (complete units), or null. */
    fun push(data: ByteArray, off: Int, n: Int, now: Long = System.nanoTime()): ByteArray? {
        if (len + n > buf.size) buf = buf.copyOf(maxOf(buf.size * 2, len + n))
        System.arraycopy(data, off, buf, len, n)
        len += n
        var i = 0
        while (i < len) {
            if (buf[i] != Wire.OUTER_SYNC) { i++; continue }
            val total = Wire.outerHeader(buf, i, len)
            if (total < 0) { i++; continue }
            if (total == 0 || len - i < total) break
            i += total
        }
        val ready = if (i > 0) buf.copyOf(i) else null
        System.arraycopy(buf, i, buf, 0, len - i)
        len -= i
        heldSince = if (len == 0) 0L else if (i > 0 || heldSince == 0L) now else heldSince
        return ready
    }

    /** Hands over whatever is held (an incomplete frame that has gone stale). */
    fun drain(): ByteArray? {
        if (len == 0) return null
        val out = buf.copyOf(len)
        len = 0
        heldSince = 0L
        return out
    }
}

/**
 * Splices bridge notices into the RC -> client stream at outer-frame
 * boundaries (or anyway, once one has waited [maxWaitNs]). Not thread-safe:
 * the caller serialises [relay], [post] and [flushIfStale].
 */
class NoticeSplicer(
    private val tracker: OuterTracker,
    private val maxWaitNs: Long,
    private val out: (ByteArray) -> Unit,
) {
    private val pending = ArrayList<ByteArray>()
    private var since = 0L
    val hasPending: Boolean get() = pending.isNotEmpty()

    /** Passes on a chunk from the RC, inserting pending notices at the first boundary. */
    fun relay(chunk: ByteArray, now: Long = System.nanoTime()) {
        if (pending.isEmpty()) {
            tracker.feed(chunk, 0, chunk.size)
            out(chunk)
            return
        }
        var off = 0
        while (off < chunk.size) {
            if (pending.isNotEmpty() && (tracker.atBoundary || now - since > maxWaitNs)) flush()
            val used = tracker.feed(chunk, off, chunk.size - off, stopAtBoundary = pending.isNotEmpty())
            if (used > 0) {
                out(if (off == 0 && used == chunk.size) chunk else chunk.copyOfRange(off, off + used))
                off += used
            }
        }
        if (pending.isNotEmpty() && tracker.atBoundary) flush()
    }

    /** Queues a complete frame; it goes out at once if the stream is between frames. */
    fun post(frame: ByteArray, now: Long = System.nanoTime()) {
        if (pending.isEmpty()) since = now
        pending += frame
        if (tracker.atBoundary) flush()
    }

    fun flushIfStale(now: Long = System.nanoTime()) {
        if (pending.isNotEmpty() && now - since > maxWaitNs) flush()
    }

    private fun flush() {
        for (f in pending) out(f)
        pending.clear()
    }
}
