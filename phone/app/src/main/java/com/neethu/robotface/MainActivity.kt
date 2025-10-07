package com.neethu.robotface

import android.content.ComponentName
import android.content.Intent
import android.content.IntentFilter
import android.net.Uri
import android.os.Bundle
import android.util.Log
import android.view.View
import android.view.WindowManager
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsControllerCompat
import android.widget.VideoView
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat

/**
 * MainActivity: plays a fullscreen looping default video and switches to emotion videos
 * when receiving ADB broadcast intents. Uses traditional Android View system (VideoView).
 *
 * Expected ADB commands (example):
 * adb shell am broadcast -a com.neethu.robotface.SET_EMOTION --es emotion angry
 * adb shell am broadcast -a com.neethu.robotface.RESET
 */
class MainActivity : AppCompatActivity(), EmotionCommandReceiver.EmotionCommandListener {
    private lateinit var videoView: VideoView
    private lateinit var receiver: EmotionCommandReceiver

    // current emotion name (null means default)
    private var currentEmotion: String? = null

    // default resource name in res/raw (without extension). Place your default video as res/raw/default_face.mp4
    private val defaultResourceName = "default_face"

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContentView(R.layout.activity_main)

        // Keep screen on while activity is visible (useful for robot face)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        videoView = findViewById(R.id.faceVideoView)

        // Make the content draw behind system bars and enable immersive fullscreen.
        // We do not apply additional padding so the VideoView covers the full screen.
        WindowCompat.setDecorFitsSystemWindows(window, false)
        hideSystemUI()

        // Start playing the default looping video immediately
        playResourceByName(defaultResourceName, looping = true)

        // register broadcast receiver for adb-based commands
        receiver = EmotionCommandReceiver(this)
        val filter = IntentFilter().apply {
            addAction(EmotionCommandReceiver.ACTION_SET_EMOTION)
            addAction(EmotionCommandReceiver.ACTION_RESET)
        }
        ContextCompat.registerReceiver(
            this,
            receiver,
            filter,
            ContextCompat.RECEIVER_EXPORTED
        )
    }

    override fun onResume() {
        super.onResume()
        hideSystemUI()
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) hideSystemUI()
    }

    private fun hideSystemUI() {
        try {
            val controller = WindowInsetsControllerCompat(window, window.decorView)
            // Hide both the status bar and navigation bar
            controller.hide(WindowInsetsCompat.Type.systemBars())
            // Allow the bars to be revealed with a swipe
            controller.systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE

            // Fallback for older APIs: set deprecated systemUiVisibility flags on the decor view
            @Suppress("DEPRECATION")
            window.decorView.systemUiVisibility = (
                View.SYSTEM_UI_FLAG_FULLSCREEN or
                View.SYSTEM_UI_FLAG_HIDE_NAVIGATION or
                View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
            )
        } catch (e: Exception) {
            Log.w(TAG, "Failed to set immersive mode: ${e.message}")
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        try {
            unregisterReceiver(receiver)
        } catch (e: Exception) {
            Log.w(TAG, "Receiver already unregistered: ${e.message}")
        }
    }

    private fun playResourceByName(name: String, looping: Boolean) {
        val resId = resources.getIdentifier(name, "raw", packageName)
        if (resId == 0) {
            Log.e(TAG, "Resource not found: $name (expected in res/raw)")
            return
        }
        playResource(resId, looping)
    }

    private fun playResource(resId: Int, looping: Boolean) {
        val uri = Uri.parse("android.resource://$packageName/$resId")
        runOnUiThread {
            try {
                videoView.setVideoURI(uri)
                videoView.setOnPreparedListener { mp ->
                    mp.isLooping = looping
                    // remove any speed/alignment changes here if needed
                    videoView.start()
                }
                videoView.setOnErrorListener { _, what, extra ->
                    Log.e(TAG, "Video playback error what=$what extra=$extra")
                    true
                }
            } catch (e: Exception) {
                Log.e(TAG, "Failed to play video: ${e.message}")
            }
        }
    }

    // EmotionCommandListener implementations
    override fun onSetEmotion(emotion: String?) {
        if (emotion == null || emotion.isBlank()) return
        Log.i(TAG, "Set emotion: $emotion")
        currentEmotion = emotion
        // Expecting emotion resource files like res/raw/angry.mp4, happy.mp4, etc.
        playResourceByName(emotion, looping = true)
    }

    override fun onReset() {
        Log.i(TAG, "Reset to default emotion")
        currentEmotion = null
        playResourceByName(defaultResourceName, looping = true)
    }

    companion object {
        private const val TAG = "MainActivity"
    }
}