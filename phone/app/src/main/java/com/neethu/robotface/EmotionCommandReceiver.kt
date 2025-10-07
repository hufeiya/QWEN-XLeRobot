package com.neethu.robotface

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.util.Log

/**
 * Receives ADB broadcast commands to control the robot face emotion playback.
 * Actions:
 * - com.neethu.robotface.SET_EMOTION (extra: emotion -> string)
 * - com.neethu.robotface.RESET
 */
class EmotionCommandReceiver(private val listener: EmotionCommandListener) : BroadcastReceiver() {
    override fun onReceive(context: Context?, intent: Intent?) {
        if (intent == null) return
        val action = intent.action
        Log.i(TAG, "Received intent action=$action extras=${intent.extras}")
        when (action) {
            ACTION_SET_EMOTION -> {
                val emotion = intent.getStringExtra(EXTRA_EMOTION)
                listener.onSetEmotion(emotion)
            }
            ACTION_RESET -> {
                listener.onReset()
            }
        }
    }

    interface EmotionCommandListener {
        fun onSetEmotion(emotion: String?)
        fun onReset()
    }

    companion object {
        private const val TAG = "EmotionCmdReceiver"
        const val ACTION_SET_EMOTION = "com.neethu.robotface.SET_EMOTION"
        const val ACTION_RESET = "com.neethu.robotface.RESET"
        const val EXTRA_EMOTION = "emotion"
    }
}
