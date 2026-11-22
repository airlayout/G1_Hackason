package com.robotvlm.pepperbridge

import android.app.Activity
import android.os.Bundle
import android.util.Log
import android.widget.TextView
import com.aldebaran.qi.sdk.QiContext
import com.aldebaran.qi.sdk.QiSDK
import com.aldebaran.qi.sdk.RobotLifecycleCallbacks

class MainActivity : Activity(), RobotLifecycleCallbacks {
    private lateinit var controller: PepperMotionController
    private lateinit var server: MotionBridgeServer
    private lateinit var statusView: TextView

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        statusView = TextView(this).apply {
            textSize = 24f
            setPadding(32, 32, 32, 32)
            text = "Pepper Motion Bridge\nport 8765\nwaiting for robot focus"
        }
        setContentView(statusView)

        controller = PepperMotionController()
        server = MotionBridgeServer(8765, controller)
        try {
            server.start()
            Log.i(TAG, "Bridge listening on 0.0.0.0:8765")
        } catch (e: Exception) {
            statusView.text = "Bridge failed to start: ${e.message}"
            Log.e(TAG, "HTTP bridge startup failed", e)
        }
        QiSDK.register(this, this)
    }

    override fun onRobotFocusGained(qiContext: QiContext) {
        controller.onRobotFocusGained(qiContext)
        runOnUiThread { statusView.text = "Pepper Motion Bridge\nport 8765\nrobot focus: ready" }
    }

    override fun onRobotFocusLost() {
        controller.onRobotFocusLost()
        runOnUiThread { statusView.text = "Pepper Motion Bridge\nport 8765\nrobot focus: lost" }
    }

    override fun onRobotFocusRefused(reason: String) {
        controller.onRobotFocusLost()
        runOnUiThread { statusView.text = "Pepper Motion Bridge\nrobot focus refused: $reason" }
    }

    override fun onDestroy() {
        QiSDK.unregister(this, this)
        if (::server.isInitialized) server.stop()
        if (::controller.isInitialized) controller.shutdown()
        super.onDestroy()
    }

    companion object {
        private const val TAG = "PepperMotionBridge"
    }
}
