package io.github.smnz.openfimi.bridge

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.ServiceInfo
import android.hardware.usb.UsbAccessory
import android.hardware.usb.UsbManager
import android.net.nsd.NsdManager
import android.net.nsd.NsdServiceInfo
import android.net.wifi.WifiManager
import android.os.Build
import android.os.IBinder
import android.os.ParcelFileDescriptor
import android.os.PowerManager
import android.util.Log
import java.io.FileInputStream
import java.io.FileOutputStream
import java.io.IOException
import java.net.InetSocketAddress
import java.net.ServerSocket
import java.net.Socket
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.CopyOnWriteArrayList
import java.util.concurrent.TimeUnit

/**
 * Relays the remote controller's USB accessory stream to TCP clients.
 *
 * Bytes are copied verbatim both ways: every client receives everything the RC
 * sends (telemetry and video), and anything a client sends goes to the RC.
 * Framing, sequence numbers and retransmission all live in the client
 * (the openfimi Python library), so this stays a dumb, fast pipe.
 */
class BridgeService : Service() {

    companion object {
        const val TAG = "openfimi"
        const val PORT = 10052
        const val EXTRA_ACCESSORY = "accessory"
        const val ACTION_STOP = "io.github.smnz.openfimi.bridge.STOP"
        private const val CHANNEL = "bridge"
        private const val NOTIFICATION_ID = 1
        private const val CLIENT_QUEUE = 1024 // chunks buffered per slow client
    }

    private var pfd: ParcelFileDescriptor? = null
    private var rcOut: FileOutputStream? = null
    private var server: ServerSocket? = null
    private val clients = CopyOnWriteArrayList<Client>()
    private val writeLock = Any()
    private var wakeLock: PowerManager.WakeLock? = null
    private var wifiLock: WifiManager.WifiLock? = null
    private var nsd: NsdManager? = null
    private var nsdListener: NsdManager.RegistrationListener? = null
    @Volatile private var running = false

    private val detachReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            if (intent.action == UsbManager.ACTION_USB_ACCESSORY_DETACHED) {
                BridgeState.status = "Remote unplugged"
                stopSelf()
            }
        }
    }

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        if (intent?.action == ACTION_STOP) {
            stopSelf()
            return START_NOT_STICKY
        }
        startInForeground()
        if (running) return START_NOT_STICKY
        val accessory = if (Build.VERSION.SDK_INT >= 33) {
            intent?.getParcelableExtra(EXTRA_ACCESSORY, UsbAccessory::class.java)
        } else {
            @Suppress("DEPRECATION") intent?.getParcelableExtra(EXTRA_ACCESSORY)
        }
        if (accessory == null) {
            BridgeState.status = "No remote"
            stopSelf()
            return START_NOT_STICKY
        }
        start(accessory)
        return START_NOT_STICKY
    }

    private fun start(accessory: UsbAccessory) {
        val usb = getSystemService(UsbManager::class.java)
        val fd = try {
            usb.openAccessory(accessory)
        } catch (e: SecurityException) {
            null
        }
        if (fd == null) {
            BridgeState.status = "Could not open the remote (permission?)"
            stopSelf()
            return
        }
        pfd = fd
        rcOut = FileOutputStream(fd.fileDescriptor)
        running = true
        BridgeState.reset(accessory)
        BridgeState.status = "Remote connected"

        registerReceiver(detachReceiver, IntentFilter(UsbManager.ACTION_USB_ACCESSORY_DETACHED),
            RECEIVER_NOT_EXPORTED)
        acquireLocks()

        // The stock app writes a single 0x00 right after opening the accessory.
        if (BridgeState.sendHello) writeToRc(byteArrayOf(0))

        Thread({ rcReadLoop(FileInputStream(fd.fileDescriptor)) }, "rc-read").start()
        Thread({ acceptLoop() }, "tcp-accept").start()
        registerNsd()
        updateNotification()
    }

    // -- RC -> clients ---------------------------------------------------------

    private fun rcReadLoop(input: FileInputStream) {
        val buf = ByteArray(16384)
        try {
            while (running) {
                val n = input.read(buf)
                if (n < 0) break
                if (n == 0) continue
                val chunk = buf.copyOf(n)
                BridgeState.rxBytes += n
                BridgeState.capture?.record(Capture.RX, chunk)
                for (c in clients) c.offer(chunk)
            }
        } catch (e: IOException) {
            Log.i(TAG, "RC read ended: $e")
        }
        if (running) {
            BridgeState.status = "Remote link closed"
            stopSelf()
        }
    }

    private fun writeToRc(data: ByteArray) {
        synchronized(writeLock) {
            rcOut?.write(data)
        }
        BridgeState.txBytes += data.size
        BridgeState.capture?.record(Capture.TX, data)
    }

    // -- clients -> RC -----------------------------------------------------------

    private fun acceptLoop() {
        try {
            val srv = ServerSocket()
            srv.reuseAddress = true
            srv.bind(InetSocketAddress(PORT))
            server = srv
            while (running) {
                val sock = srv.accept()
                sock.tcpNoDelay = true
                val c = Client(sock)
                clients.add(c)
                BridgeState.clients = clients.size
                updateNotification()
                c.start()
            }
        } catch (e: IOException) {
            if (running) {
                Log.w(TAG, "server failed: $e")
                BridgeState.status = "TCP server failed: ${e.message}"
            }
        }
    }

    private inner class Client(private val sock: Socket) {
        private val queue = ArrayBlockingQueue<ByteArray>(CLIENT_QUEUE)
        val peer: String = sock.remoteSocketAddress.toString()

        fun offer(chunk: ByteArray) {
            if (!queue.offer(chunk)) BridgeState.dropped++
        }

        fun start() {
            Log.i(TAG, "client $peer connected")
            Thread({ sendLoop() }, "tcp-tx").start()
            Thread({ recvLoop() }, "tcp-rx").start()
        }

        private fun sendLoop() {
            try {
                val out = sock.getOutputStream()
                while (running && !sock.isClosed) {
                    val chunk = queue.poll(500, TimeUnit.MILLISECONDS) ?: continue
                    out.write(chunk)
                }
            } catch (e: Exception) {
                Log.i(TAG, "client $peer send ended: $e")
            }
            close()
        }

        private fun recvLoop() {
            val buf = ByteArray(16384)
            try {
                val inp = sock.getInputStream()
                while (running) {
                    val n = inp.read(buf)
                    if (n < 0) break
                    if (n > 0) writeToRc(buf.copyOf(n))
                }
            } catch (e: IOException) {
                Log.i(TAG, "client $peer recv ended: $e")
            }
            close()
        }

        fun close() {
            try {
                sock.close()
            } catch (_: IOException) {
            }
            if (clients.remove(this)) {
                Log.i(TAG, "client $peer disconnected")
                BridgeState.clients = clients.size
                updateNotification()
            }
        }
    }

    // -- discovery, locks, notification ------------------------------------------

    private fun registerNsd() {
        val info = NsdServiceInfo().apply {
            serviceName = "openfimi-" + Build.MODEL.replace(' ', '-')
            serviceType = "_openfimi._tcp"
            port = PORT
        }
        val listener = object : NsdManager.RegistrationListener {
            override fun onServiceRegistered(i: NsdServiceInfo) {
                Log.i(TAG, "mDNS registered as ${i.serviceName}")
            }
            override fun onRegistrationFailed(i: NsdServiceInfo, err: Int) {
                Log.w(TAG, "mDNS registration failed: $err")
            }
            override fun onServiceUnregistered(i: NsdServiceInfo) {}
            override fun onUnregistrationFailed(i: NsdServiceInfo, err: Int) {}
        }
        nsd = getSystemService(NsdManager::class.java)
        nsd?.registerService(info, NsdManager.PROTOCOL_DNS_SD, listener)
        nsdListener = listener
    }

    private fun acquireLocks() {
        wakeLock = getSystemService(PowerManager::class.java)
            .newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "openfimi:bridge")
            .apply { acquire() }
        @Suppress("DEPRECATION")
        wifiLock = getSystemService(WifiManager::class.java)
            .createWifiLock(WifiManager.WIFI_MODE_FULL_HIGH_PERF, "openfimi:bridge")
            .apply { acquire() }
    }

    private fun startInForeground() {
        val nm = getSystemService(NotificationManager::class.java)
        nm.createNotificationChannel(
            NotificationChannel(CHANNEL, "Bridge", NotificationManager.IMPORTANCE_LOW))
        val n = buildNotification()
        if (Build.VERSION.SDK_INT >= 29) {
            startForeground(NOTIFICATION_ID, n, ServiceInfo.FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE)
        } else {
            startForeground(NOTIFICATION_ID, n)
        }
    }

    private fun buildNotification(): Notification {
        val open = PendingIntent.getActivity(this, 0, Intent(this, MainActivity::class.java),
            PendingIntent.FLAG_IMMUTABLE)
        val stop = PendingIntent.getService(this, 1,
            Intent(this, BridgeService::class.java).setAction(ACTION_STOP), PendingIntent.FLAG_IMMUTABLE)
        val text = if (running) {
            "${BridgeState.addresses().firstOrNull() ?: "no network"}:$PORT · ${clients.size} client(s)"
        } else BridgeState.status
        return Notification.Builder(this, CHANNEL)
            .setSmallIcon(R.drawable.ic_bridge)
            .setContentTitle("openfimi bridge")
            .setContentText(text)
            .setContentIntent(open)
            .setOngoing(true)
            .addAction(Notification.Action.Builder(null, "Stop", stop).build())
            .build()
    }

    private fun updateNotification() {
        getSystemService(NotificationManager::class.java).notify(NOTIFICATION_ID, buildNotification())
    }

    override fun onDestroy() {
        running = false
        try {
            unregisterReceiver(detachReceiver)
        } catch (_: IllegalArgumentException) {
        }
        nsdListener?.let { try { nsd?.unregisterService(it) } catch (_: Exception) {} }
        try { server?.close() } catch (_: IOException) {}
        for (c in clients) c.close()
        try { pfd?.close() } catch (_: IOException) {}
        BridgeState.capture?.close()
        BridgeState.capture = null
        BridgeState.connected = false
        BridgeState.clients = 0
        wakeLock?.let { if (it.isHeld) it.release() }
        wifiLock?.let { if (it.isHeld) it.release() }
        super.onDestroy()
    }
}
