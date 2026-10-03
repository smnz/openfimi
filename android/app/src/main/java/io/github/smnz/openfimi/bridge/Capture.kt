package io.github.smnz.openfimi.bridge

import java.io.BufferedOutputStream
import java.io.File
import java.io.FileOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Records both directions in openfimi's capture format (`openfimi decode`):
 * magic "OFCAP\0\u0001\n", then records of <u8 direction><f64 unix time><u32 length><bytes>,
 * little-endian; direction 0 = from the remote, 1 = to the remote.
 */
class Capture(val file: File) {
    companion object {
        const val RX = 0
        const val TX = 1
        private val MAGIC = byteArrayOf(0x4F, 0x46, 0x43, 0x41, 0x50, 0x00, 0x01, 0x0A)
    }

    private val out = BufferedOutputStream(FileOutputStream(file), 1 shl 16)
    @Volatile var bytes = 0L
        private set
    private var lastFlush = 0L

    init {
        out.write(MAGIC)
    }

    @Synchronized
    fun record(direction: Int, data: ByteArray) {
        val head = ByteBuffer.allocate(13).order(ByteOrder.LITTLE_ENDIAN)
        head.put(direction.toByte())
        head.putDouble(System.currentTimeMillis() / 1000.0)
        head.putInt(data.size)
        out.write(head.array())
        out.write(data)
        bytes += 13 + data.size
        val now = System.currentTimeMillis()
        if (now - lastFlush > 1000) { // bound what a crash or kill can lose
            out.flush()
            lastFlush = now
        }
    }

    @Synchronized
    fun close() {
        out.flush()
        out.close()
    }
}
