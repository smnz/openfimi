package io.github.smnz.openfimi.bridge

import android.util.Log
import java.util.concurrent.RejectedExecutionException
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit
import kotlin.random.Random

/**
 * The bridge's own emergency return-to-home: stop any running route, then
 * command return-and-land, retrying until the flight controller ACKs it.
 *
 * Runs on [exec] (all state is touched on that thread); [onFrame] is fed every
 * FmLink4 frame from the remote so the ACK can be matched by group/msg/seq.
 */
class EmergencyRth(
    private val exec: ScheduledExecutorService,
    private val send: (ByteArray) -> Unit,
    private val report: (stage: String, code: Int?) -> Unit,
) {
    companion object {
        const val STOP_TO_RTH_MS = 300L
        const val RETRY_MS = 500L
        const val TRIES = 6

        /** Notice body for a stage, e.g. {"event":"emergency_rth","source":"bridge","stage":"activated"}. */
        fun json(stage: String, code: Int?): String =
            "{\"event\":\"emergency_rth\",\"source\":\"bridge\",\"stage\":\"$stage\"" +
                (if (code != null) ",\"code\":$code" else "") + "}"
    }

    private var seq = Random.nextInt(Wire.SEQ_WRAP)
    /** seq of the outstanding return_home, or -1 when not waiting for an ACK. */
    @Volatile private var rthSeq = -1
    private var tries = 0
    private var pending: ScheduledFuture<*>? = null

    /** Runs [block] on [exec], unless the bridge has already shut it down. */
    private fun post(block: () -> Unit) {
        try {
            exec.execute(block)
        } catch (_: RejectedExecutionException) {
        }
    }

    private fun nextSeq(): Int = seq.also { seq = (seq + 1) % Wire.SEQ_WRAP }

    /** Starts (or restarts) the sequence. Safe to call from any thread. */
    fun activate() {
        post {
            pending?.cancel(false)
            rthSeq = -1
            report("activated", null)
            transmit(Wire.MISSION_STOP)
            transmit(Wire.FLY_TO_EXIT)
            pending = exec.schedule({
                rthSeq = nextSeq()
                tries = 0
                sendReturnHome()
            }, STOP_TO_RTH_MS, TimeUnit.MILLISECONDS)
        }
    }

    private fun transmit(cmd: Pair<Int, Int>, s: Int = nextSeq()) {
        try {
            send(Wire.command(cmd.first, cmd.second, s))
        } catch (e: Exception) {
            Log.w(BridgeService.TAG, "emergency: write to the remote failed: $e")
        }
    }

    private fun sendReturnHome() {
        val s = rthSeq
        if (s < 0) return
        if (tries >= TRIES) {
            rthSeq = -1
            report("no_reply", null)
            return
        }
        tries++
        transmit(Wire.RETURN_HOME, s)
        pending = exec.schedule({ if (rthSeq == s) sendReturnHome() }, RETRY_MS, TimeUnit.MILLISECONDS)
    }

    /** Called (on the RC read thread) for every frame from the remote. */
    fun onFrame(f: Wire.Fm) {
        val s = rthSeq
        if (s < 0 || f.seq != s || f.src != Wire.FC || f.dst != Wire.GCS) return
        if (f.group != Wire.RETURN_HOME.first || f.msg != Wire.RETURN_HOME.second) return
        val code = f.result
        post {
            if (rthSeq != s) return@post // already settled (duplicate ACK)
            rthSeq = -1
            pending?.cancel(false)
            if (code == 0) report("accepted", null) else report("refused", code)
        }
    }
}
