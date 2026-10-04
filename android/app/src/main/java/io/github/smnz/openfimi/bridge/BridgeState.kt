package io.github.smnz.openfimi.bridge

import android.hardware.usb.UsbAccessory
import android.util.Log
import java.net.Inet4Address
import java.net.NetworkInterface
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/** Shared state between the service and the UI (single process, so a plain object). */
object BridgeState {
    @Volatile var status: String = "Plug in the remote controller"
    @Volatile var connected = false
    @Volatile var accessory: String = ""
    @Volatile var clients = 0
    @Volatile var rxBytes = 0L
    @Volatile var txBytes = 0L
    @Volatile var dropped = 0L
    @Volatile var capture: Capture? = null
    @Volatile var sendHello = true

    /** Set when the user stops the bridge, so reopening the app does not reconnect. */
    @Volatile var userStopped = false

    /** Outcome of the last emergency return-to-home ("" if never pressed). */
    @Volatile var emergency: String = ""

    private val events = ArrayDeque<String>()

    /** Adds a timestamped line to the event log shown in the app (and logcat). */
    fun log(msg: String) {
        Log.i(BridgeService.TAG, msg)
        val t = SimpleDateFormat("HH:mm:ss", Locale.US).format(Date())
        synchronized(events) {
            events.addLast("$t $msg")
            while (events.size > 30) events.removeFirst()
        }
    }

    fun recentEvents(n: Int = 8): List<String> = synchronized(events) { events.toList().takeLast(n) }

    fun reset(acc: UsbAccessory) {
        connected = true
        accessory = listOfNotNull(acc.manufacturer, acc.model, acc.version).joinToString(" · ")
        rxBytes = 0
        txBytes = 0
        dropped = 0
        emergency = ""
    }

    /** IPv4 addresses a client could reach (Wi-Fi, hotspot), most useful first. */
    fun addresses(): List<String> {
        val out = mutableListOf<Pair<Int, String>>()
        try {
            for (nif in NetworkInterface.getNetworkInterfaces()) {
                if (!nif.isUp || nif.isLoopback) continue
                val rank = when {
                    nif.name.startsWith("wlan") -> 0
                    nif.name.startsWith("ap") || nif.name.startsWith("swlan") -> 1
                    nif.name.startsWith("rndis") || nif.name.startsWith("eth") -> 2
                    nif.name.startsWith("rmnet") || nif.name.startsWith("ccmni") -> 9
                    else -> 5
                }
                for (a in nif.inetAddresses) {
                    if (a is Inet4Address && !a.isLoopbackAddress) out += rank to a.hostAddress!!
                }
            }
        } catch (_: Exception) {
        }
        return out.sortedBy { it.first }.filter { it.first < 9 }.map { it.second }
    }
}
