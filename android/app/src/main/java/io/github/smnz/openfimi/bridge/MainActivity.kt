package io.github.smnz.openfimi.bridge

import android.app.Activity
import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.graphics.Typeface
import android.hardware.usb.UsbAccessory
import android.hardware.usb.UsbManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.Gravity
import android.widget.Button
import android.widget.CheckBox
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

class MainActivity : Activity() {

    companion object {
        private const val ACTION_PERMISSION = "io.github.smnz.openfimi.bridge.USB_PERMISSION"
        private const val FIMI = "Beijing FIMI Technology Limited"
    }

    private val ui = Handler(Looper.getMainLooper())
    private lateinit var statusView: TextView
    private lateinit var detailView: TextView
    private lateinit var recordButton: Button
    private lateinit var connectButton: Button

    private val permissionReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            if (intent.action != ACTION_PERMISSION) return
            val acc = accessoryFrom(intent)
            if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false) && acc != null) {
                startBridge(acc)
            } else {
                BridgeState.status = "Permission for the remote was refused"
            }
        }
    }

    private val tick = object : Runnable {
        override fun run() {
            refresh()
            ui.postDelayed(this, 500)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(buildUi())
        registerReceiver(permissionReceiver, IntentFilter(ACTION_PERMISSION), RECEIVER_NOT_EXPORTED)
        if (Build.VERSION.SDK_INT >= 33 &&
            checkSelfPermission(android.Manifest.permission.POST_NOTIFICATIONS) !=
            PackageManager.PERMISSION_GRANTED
        ) {
            requestPermissions(arrayOf(android.Manifest.permission.POST_NOTIFICATIONS), 1)
        }
        handle(intent)
    }

    override fun onNewIntent(intent: Intent) {
        super.onNewIntent(intent)
        handle(intent)
    }

    override fun onResume() {
        super.onResume()
        ui.post(tick)
    }

    override fun onPause() {
        super.onPause()
        ui.removeCallbacks(tick)
    }

    override fun onDestroy() {
        unregisterReceiver(permissionReceiver)
        super.onDestroy()
    }

    private fun handle(intent: Intent?) {
        if (intent?.action == UsbManager.ACTION_USB_ACCESSORY_ATTACHED) {
            // Launched by the system for the remote: permission is already granted.
            accessoryFrom(intent)?.let { startBridge(it) }
        } else if (!BridgeState.connected) {
            connect()
        }
    }

    private fun accessoryFrom(intent: Intent): UsbAccessory? =
        if (Build.VERSION.SDK_INT >= 33) {
            intent.getParcelableExtra(UsbManager.EXTRA_ACCESSORY, UsbAccessory::class.java)
        } else {
            @Suppress("DEPRECATION") intent.getParcelableExtra(UsbManager.EXTRA_ACCESSORY)
        }

    /** Find an attached remote and ask for permission if needed. */
    private fun connect() {
        val usb = getSystemService(UsbManager::class.java)
        val acc = usb.accessoryList?.firstOrNull { it.manufacturer == FIMI }
            ?: usb.accessoryList?.firstOrNull()
        if (acc == null) {
            BridgeState.status = "No remote found. Plug it in and switch it on."
            return
        }
        if (usb.hasPermission(acc)) {
            startBridge(acc)
        } else {
            val pi = PendingIntent.getBroadcast(this, 0,
                Intent(ACTION_PERMISSION).setPackage(packageName), PendingIntent.FLAG_MUTABLE)
            usb.requestPermission(acc, pi)
        }
    }

    private fun startBridge(acc: UsbAccessory) {
        val i = Intent(this, BridgeService::class.java).putExtra(BridgeService.EXTRA_ACCESSORY, acc)
        startForegroundService(i)
    }

    private fun toggleRecording() {
        val cap = BridgeState.capture
        if (cap != null) {
            BridgeState.capture = null
            cap.close()
            BridgeState.status = "Saved ${cap.file.name}"
        } else {
            val dir = File(getExternalFilesDir(null), "captures").apply { mkdirs() }
            val name = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date())
            BridgeState.capture = Capture(File(dir, "session-$name.ofcap"))
        }
        refresh()
    }

    private fun refresh() {
        val s = BridgeState
        statusView.text = if (s.connected) "Bridging" else s.status
        val addrs = s.addresses()
        val sb = StringBuilder()
        if (s.connected) {
            sb.append("Remote: ${s.accessory}\n\n")
            sb.append("Connect from your computer:\n")
            if (addrs.isEmpty()) sb.append("  (no Wi-Fi: join a network or turn on the hotspot)\n")
            for (a in addrs) sb.append("  openfimi monitor -u tcp://$a:${BridgeService.PORT}\n")
            sb.append("\nClients: ${s.clients}\n")
            sb.append("From remote: ${human(s.rxBytes)}   To remote: ${human(s.txBytes)}\n")
            if (s.dropped > 0) sb.append("Dropped chunks (slow client): ${s.dropped}\n")
            s.capture?.let { sb.append("\nRecording ${it.file.name} (${human(it.bytes)})\n${it.file.parent}\n") }
            sb.append("\n").append(s.status)
        } else {
            sb.append("1. Plug the phone into the remote's USB port.\n")
            sb.append("2. Switch the remote on; choose \"openfimi bridge\" if Android asks.\n")
            sb.append("3. Connect from your computer with the address shown here.\n\n")
            sb.append("The FIMI app cannot be open at the same time.")
        }
        detailView.text = sb.toString()
        recordButton.text = if (s.capture != null) "Stop recording" else "Record session"
        recordButton.isEnabled = s.connected
        connectButton.text = if (s.connected) "Stop bridge" else "Connect"
    }

    private fun human(n: Long): String = when {
        n >= 1 shl 20 -> String.format(Locale.US, "%.1f MB", n / 1048576.0)
        n >= 1 shl 10 -> String.format(Locale.US, "%.1f kB", n / 1024.0)
        else -> "$n B"
    }

    private fun buildUi(): ScrollView {
        val pad = (16 * resources.displayMetrics.density).toInt()
        val col = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(pad, pad, pad, pad)
        }
        col.addView(TextView(this).apply {
            text = getString(R.string.app_name)
            textSize = 22f
            setTypeface(typeface, Typeface.BOLD)
        })
        statusView = TextView(this).apply {
            textSize = 18f
            setPadding(0, pad, 0, pad / 2)
        }
        col.addView(statusView)
        detailView = TextView(this).apply {
            textSize = 14f
            typeface = Typeface.MONOSPACE
            setTextIsSelectable(true)
        }
        col.addView(detailView)
        val row = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.START
            setPadding(0, pad, 0, 0)
        }
        connectButton = Button(this).apply {
            setOnClickListener {
                if (BridgeState.connected) {
                    startService(Intent(this@MainActivity, BridgeService::class.java)
                        .setAction(BridgeService.ACTION_STOP))
                } else {
                    connect()
                }
            }
        }
        recordButton = Button(this).apply { setOnClickListener { toggleRecording() } }
        row.addView(connectButton)
        row.addView(recordButton)
        col.addView(row)
        col.addView(CheckBox(this).apply {
            text = "Send the app's 0x00 byte on connect"
            isChecked = BridgeState.sendHello
            setOnCheckedChangeListener { _, on -> BridgeState.sendHello = on }
        })
        return ScrollView(this).apply { addView(col) }
    }
}
