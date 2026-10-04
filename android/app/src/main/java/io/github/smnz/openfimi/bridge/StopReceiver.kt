package io.github.smnz.openfimi.bridge

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent

/** The notification's Stop button: stop the service and clear the notification. */
class StopReceiver : BroadcastReceiver() {
    override fun onReceive(context: Context, intent: Intent) {
        BridgeState.userStopped = true
        context.stopService(Intent(context, BridgeService::class.java))
        BridgeService.cancelNotification(context)
    }
}
