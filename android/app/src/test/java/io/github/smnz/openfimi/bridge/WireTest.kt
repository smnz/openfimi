package io.github.smnz.openfimi.bridge

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.ByteArrayOutputStream
import java.util.concurrent.Executors
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit

private fun hex(s: String): ByteArray = ByteArray(s.length / 2) { s.substring(2 * it, 2 * it + 2).toInt(16).toByte() }
private fun ByteArray.hex(): String = joinToString("") { "%02x".format(it) }
private fun cat(vararg parts: ByteArray): ByteArray = ByteArrayOutputStream().apply { parts.forEach { write(it) } }.toByteArray()

/** A version-2 (8-byte header) outer frame, as the RC can send. */
private fun outerV2(body: ByteArray, type: Int): ByteArray {
    val total = body.size + 8
    val h = byteArrayOf(0xAE.toByte(), 0x02, 0x00, type.toByte(), total.toByte(), (total ushr 8).toByte(), 0, 0)
    h[7] = Wire.additive8(h, 0, 7).toByte()
    return cat(h, body)
}

private fun video(n: Int, fill: Int = 0x5A) = Wire.encodeOuter(ByteArray(n) { (fill + it).toByte() }, Wire.TYPE_VIDEO)

/** Reference decoder (a straight port of openfimi's OuterDecoder) used to check output streams. */
private fun decodeOuter(buf: ByteArray): List<Pair<Int, ByteArray>> {
    val out = ArrayList<Pair<Int, ByteArray>>()
    var i = 0
    while (i < buf.size) {
        if (buf[i] != Wire.OUTER_SYNC) { i++; continue }
        val total = Wire.outerHeader(buf, i, buf.size)
        if (total <= 0) { i++; continue }
        if (buf.size - i < total) break
        val hl = Wire.outerHeaderLen(buf, i)
        out += (buf[i + 3].toInt() and 0xFF) to buf.copyOfRange(i + hl, i + total)
        i += total
    }
    return out
}

class WireTest {
    private val check = "123456789".toByteArray()

    @Test fun checksums() {
        assertEquals(0x6F91, Wire.crc16(check))
        assertEquals(0xAFF19057.toInt(), Wire.crc32(check))
        assertEquals(check.sumOf { it.toInt() } and 0xFF, Wire.additive8(check))
    }

    @Test fun commandVectors() {
        assertEquals("ae71010020fe840401070200000170298257920f980323",
            Wire.command(0x03, 0x23, 0x7001).hex())
        assertEquals("ae71010020fe8404010702000001702982ecbdacf0031a",
            Wire.command(0x03, 0x1A, 0x7001).hex())
        assertEquals("ae71010020fe840401070200000170298244095e880333",
            Wire.command(0x03, 0x33, 0x7001).hex())
    }

    @Test fun noticeVector() {
        val json = EmergencyRth.json("activated", null)
        assertEquals("""{"event":"emergency_rth","source":"bridge","stage":"activated"}""", json)
        assertEquals("ae410440337b226576656e74223a22656d657267656e63795f727468222c22736f75726365223a22627269646765222c227374616765223a22616374697661746564227d",
            Wire.notice(json).hex())
        assertEquals("""{"event":"emergency_rth","source":"bridge","stage":"refused","code":17}""",
            EmergencyRth.json("refused", 17))
    }

    @Test fun innerDecoderReadsAck() {
        val ack = Wire.encodeInner(Wire.FC, Wire.GCS, 1234, byteArrayOf(0x03, 0x1A, 0x10, 0x00))
        val dec = InnerDecoder()
        val frames = dec.feed(cat(byteArrayOf(1, 2), ack.copyOf(7))) + dec.feed(ack.copyOfRange(7, ack.size))
        assertEquals(1, frames.size)
        val f = frames[0]
        assertEquals(listOf(2, 7, 1234, 3, 0x1A, 1), listOf(f.src, f.dst, f.seq, f.group, f.msg, f.result))
    }
}

class OuterTrackerTest {
    private val frames = listOf(
        Wire.command(0x03, 0x23, 1),
        video(3000),
        outerV2(Wire.encodeInner(2, 7, 9, byteArrayOf(0x03, 0x1A, 0, 0)), Wire.TYPE_FMLINK),
        Wire.encodeOuter(ByteArray(0), 5), // empty body
        video(40),
        Wire.command(0x03, 0x1A, 2),
    )

    /** Feeds [stream] in chunks of [size], returning every boundary position seen and the TYPE-0 bodies. */
    private fun scan(stream: ByteArray, size: Int): Pair<Set<Int>, List<ByteArray>> {
        val bodies = ArrayList<ByteArray>()
        val t = OuterTracker({ it == 0 }) { _, b -> bodies += b }
        val seen = sortedSetOf<Int>()
        var pos = 0
        while (pos < stream.size) {
            val end = minOf(stream.size, pos + size)
            while (pos < end) {
                if (t.atBoundary) seen += pos
                val used = t.feed(stream, pos, end - pos, stopAtBoundary = true)
                if (used == 0) {
                    // At a boundary: step past it as the relay does after inserting.
                    pos += t.feed(stream, pos, minOf(1, end - pos))
                } else pos += used
            }
        }
        if (t.atBoundary) seen += pos
        return seen to bodies
    }

    private fun starts(parts: List<ByteArray>, base: Int = 0): Set<Int> {
        val s = sortedSetOf<Int>()
        var p = base
        for (f in parts) { s += p; p += f.size }
        s += p
        return s
    }

    @Test fun boundariesAcrossAnyChunking() {
        val stream = cat(*frames.toTypedArray())
        for (size in listOf(1, 2, 3, 5, 7, 8, 13, 64, 1000, 16384)) {
            val (seen, bodies) = scan(stream, size)
            assertEquals("chunk $size", starts(frames), seen)
            assertEquals(3, bodies.size)
            assertArrayEquals(frames[0].copyOfRange(5, frames[0].size), bodies[0])
            assertArrayEquals(frames[2].copyOfRange(8, frames[2].size), bodies[1])
        }
    }

    @Test fun garbageResync() {
        val junk = byteArrayOf(0x00, 0xAE.toByte(), 0x71, 0x01, 0x00, 0x00, 0x13, 0xAE.toByte(), 0xAE.toByte())
        val stream = cat(junk, *frames.toTypedArray())
        // In one chunk every frame start after the junk is found (plus the stream start).
        val (seen, bodies) = scan(stream, stream.size)
        assertEquals(setOf(0) + starts(frames, junk.size), seen)
        assertEquals(3, bodies.size)
        // In tiny chunks the first frame start may be missed, but sync is regained by its end.
        for (size in listOf(1, 3, 6)) {
            val (s2, b2) = scan(stream, size)
            assertTrue(s2.containsAll(starts(frames, junk.size) - (junk.size)))
            assertEquals(3, b2.size)
        }
    }

    @Test fun splicerInsertsBetweenFrames() {
        val notice = Wire.notice(EmergencyRth.json("activated", null))
        val stream = cat(*frames.toTypedArray())
        for (size in listOf(1, 4, 100, 1500)) {
            val out = ByteArrayOutputStream()
            val sp = NoticeSplicer(OuterTracker(), Long.MAX_VALUE) { out.write(it) }
            var pos = 0
            var posted = false
            while (pos < stream.size) {
                val end = minOf(stream.size, pos + size)
                sp.relay(stream.copyOfRange(pos, end))
                pos = end
                // Post in the middle of the big video frame.
                if (!posted && pos > frames[0].size + 10) { sp.post(notice); posted = true }
            }
            val recs = decodeOuter(out.toByteArray())
            assertEquals("chunk $size", frames.size + 1, recs.size)
            assertEquals(Wire.TYPE_BRIDGE_NOTICE, recs[2].first) // right after the video frame
            val orig = decodeOuter(stream)
            assertEquals(orig.map { it.second.hex() }, recs.filter { it.first != 0x40 }.map { it.second.hex() })
        }
    }

    @Test fun splicerSendsAnywayWhenStale() {
        val out = ByteArrayOutputStream()
        val sp = NoticeSplicer(OuterTracker(), 100) { out.write(it) }
        val v = video(500)
        sp.relay(v.copyOf(50), now = 0)
        sp.post(byteArrayOf(1), now = 0)
        sp.flushIfStale(now = 50)
        assertTrue(sp.hasPending)
        sp.flushIfStale(now = 200)
        assertEquals(51, out.size())
    }
}

class ClientFramerTest {
    @Test fun frameSplitAcrossReads() {
        val f = Wire.command(0x03, 0x1A, 5)
        val fr = ClientFramer()
        assertNull(fr.push(f, 0, 3))
        assertNull(fr.push(f, 3, 10))
        assertEquals(13, fr.held)
        assertArrayEquals(f, fr.push(f, 13, f.size - 13))
        assertEquals(0, fr.held)
    }

    @Test fun leadingJunkForwardedAtOnce() {
        val f = Wire.command(0x03, 0x23, 6)
        val g = Wire.command(0x03, 0x33, 7)
        val fr = ClientFramer()
        val data = cat(byteArrayOf(0), f, g.copyOf(4))
        assertArrayEquals(cat(byteArrayOf(0), f), fr.push(data, 0, data.size))
        // The held 4 bytes turn out not to be a header once the 5th arrives: all junk, sent on.
        assertArrayEquals(cat(g.copyOf(4), byteArrayOf(0x42)), fr.push(byteArrayOf(0x42), 0, 1))
        val fr2 = ClientFramer()
        assertArrayEquals(byteArrayOf(0), fr2.push(byteArrayOf(0), 0, 1))
        assertArrayEquals(cat(g, byteArrayOf(9)), fr2.push(cat(g, byteArrayOf(9)), 0, g.size + 1))
    }

    @Test fun badHeaderIsJunk() {
        val bad = byteArrayOf(0xAE.toByte(), 0x71, 0x01, 0x00, 0x00, 0x01, 0x02)
        val fr = ClientFramer()
        assertArrayEquals(bad, fr.push(bad, 0, bad.size))
    }

    @Test fun staleHeldBytesDrain() {
        val f = Wire.command(0x03, 0x1A, 5)
        val fr = ClientFramer()
        assertNull(fr.push(f, 0, 8, now = 100))
        assertEquals(100, fr.heldSince)
        assertArrayEquals(f.copyOf(8), fr.drain())
        assertEquals(0, fr.held)
    }
}

class EmergencyRthTest {
    @Test fun sequenceAndAck() {
        val exec = Executors.newSingleThreadScheduledExecutor()
        val sent = LinkedBlockingQueue<ByteArray>()
        val stages = LinkedBlockingQueue<String>()
        val rth = EmergencyRth(exec, { sent.put(it) }) { stage, code -> stages.put("$stage:$code") }
        rth.activate()
        assertEquals("activated:null", stages.poll(1, TimeUnit.SECONDS))
        val inner = InnerDecoder()
        fun next() = decodeOuter(sent.poll(2, TimeUnit.SECONDS)!!).single().let { inner.feed(it.second).single() }
        val stop = next()
        val exit = next()
        assertEquals(listOf(3, 0x23), listOf(stop.group, stop.msg))
        assertEquals(listOf(3, 0x33), listOf(exit.group, exit.msg))
        val home = next()
        assertEquals(listOf(7, 2, 3, 0x1A), listOf(home.src, home.dst, home.group, home.msg))
        val again = next() // retransmitted, same seq, after ~500 ms
        assertEquals(home.seq, again.seq)
        // A wrong-seq ACK is ignored; the right one settles it.
        rth.onFrame(Wire.Fm(2, 7, (home.seq + 1) % Wire.SEQ_WRAP, byteArrayOf(3, 0x1A, 0, 0)))
        rth.onFrame(Wire.Fm(2, 7, home.seq, byteArrayOf(3, 0x1A, 0x30, 0x00)))
        assertEquals("refused:3", stages.poll(1, TimeUnit.SECONDS))
        Thread.sleep(700)
        assertTrue(sent.size <= 1) // at most one retry was already in flight
        exec.shutdownNow()
    }

    @Test fun noReplyAfterSixTries() {
        val exec = Executors.newSingleThreadScheduledExecutor()
        val sent = LinkedBlockingQueue<ByteArray>()
        val stages = LinkedBlockingQueue<String>()
        EmergencyRth(exec, { sent.put(it) }) { stage, _ -> stages.put(stage) }.activate()
        assertEquals("activated", stages.poll(1, TimeUnit.SECONDS))
        assertEquals("no_reply", stages.poll(5, TimeUnit.SECONDS))
        assertEquals(2 + EmergencyRth.TRIES, sent.size)
        exec.shutdownNow()
    }
}
