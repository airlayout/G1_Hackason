package com.robotvlm.pepperbridge

import com.aldebaran.qi.Future
import com.aldebaran.qi.sdk.QiContext
import com.aldebaran.qi.sdk.builder.GoToBuilder
import com.aldebaran.qi.sdk.builder.LookAtBuilder
import com.aldebaran.qi.sdk.builder.TransformBuilder
import com.aldebaran.qi.sdk.`object`.actuation.LookAtMovementPolicy
import com.aldebaran.qi.sdk.`object`.actuation.OrientationPolicy
import com.aldebaran.qi.sdk.`object`.geometry.Quaternion
import com.aldebaran.qi.sdk.`object`.geometry.Vector3
import org.json.JSONObject
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

class MotionRejectedException(message: String) : RuntimeException(message)

class PepperMotionController {
    private val lock = Any()
    private val timeoutExecutor: ScheduledExecutorService = Executors.newSingleThreadScheduledExecutor()
    @Volatile private var qiContext: QiContext? = null
    private var currentMotion: Future<Void>? = null
    private var currentLook: Future<Void>? = null

    fun onRobotFocusGained(context: QiContext) {
        synchronized(lock) { qiContext = context }
    }

    fun onRobotFocusLost() {
        stopAll()
        synchronized(lock) { qiContext = null }
    }

    fun status(): JSONObject = synchronized(lock) {
        JSONObject()
            .put("bridge", "ready")
            .put("robot_focus", qiContext != null)
            .put("motion_busy", currentMotion?.isDone == false)
            .put("look_active", currentLook?.isDone == false)
    }

    fun execute(path: String, payload: JSONObject): JSONObject {
        val expectedAction = path.substring(1).uppercase()
        if (payload.optString("action") != expectedAction) {
            throw IllegalArgumentException("action must be $expectedAction")
        }
        return when (path) {
            "/move" -> move(payload)
            "/turn" -> turn(payload)
            "/look" -> look(payload)
            "/stop" -> stop()
            else -> throw IllegalArgumentException("unsupported endpoint")
        }
    }

    private fun requireContext(): QiContext = qiContext
        ?: throw MotionRejectedException("robot focus is not available")

    private fun requireIdle() = synchronized(lock) {
        if (currentMotion?.isDone == false) {
            throw MotionRejectedException("another movement is already running")
        }
    }

    private fun requireMovementSafe(context: QiContext) {
        try {
            if (context.power.chargingFlap?.state?.open == true) {
                throw MotionRejectedException("charging flap is open")
            }
        } catch (e: MotionRejectedException) {
            throw e
        } catch (e: Exception) {
            throw MotionRejectedException("charging flap safety check failed: ${e.message}")
        }
    }

    private fun number(payload: JSONObject, key: String, min: Double, max: Double): Double {
        if (!payload.has(key)) throw IllegalArgumentException("missing $key")
        val value = payload.optDouble(key, Double.NaN)
        if (!value.isFinite() || value < min || value > max) {
            throw IllegalArgumentException("$key must be between $min and $max")
        }
        return value
    }

    private fun move(payload: JSONObject): JSONObject {
        val forward = number(payload, "forward_m", -0.5, 0.5)
        val sideways = number(payload, "sideways_m", -0.5, 0.5)
        val speed = number(payload, "speed_mps", 0.1, 0.3)
        if (forward == 0.0 && sideways == 0.0) throw IllegalArgumentException("distance must be non-zero")
        val context = requireContext()
        requireIdle()
        requireMovementSafe(context)
        val robotFrame = context.actuation.robotFrame()
        val targetFrame = context.mapping.makeFreeFrame()
        targetFrame.update(robotFrame, TransformBuilder.create().from2DTranslation(forward, sideways), 0L)
        val action = GoToBuilder.with(context)
            .withFrame(targetFrame.frame())
            .withMaxSpeed(speed.toFloat())
            .withFinalOrientationPolicy(OrientationPolicy.ALIGN_X)
            .build()
        startMotion(action.async().run())
        return accepted("MOVE")
    }

    private fun turn(payload: JSONObject): JSONObject {
        var degrees = number(payload, "degrees", -90.0, 90.0)
        val direction = payload.optString("direction", "")
        if (direction.isNotEmpty()) {
            if (direction != "left" && direction != "right") {
                throw IllegalArgumentException("direction must be left or right")
            }
            degrees = abs(degrees) * if (direction == "left") 1.0 else -1.0
        }
        if (abs(degrees) < 15.0) throw IllegalArgumentException("turn must be 15 to 90 degrees")
        val context = requireContext()
        requireIdle()
        requireMovementSafe(context)
        val radians = Math.toRadians(degrees)
        val rotation = Quaternion(0.0, 0.0, sin(radians / 2.0), cos(radians / 2.0))
        val robotFrame = context.actuation.robotFrame()
        val targetFrame = context.mapping.makeFreeFrame()
        targetFrame.update(robotFrame, TransformBuilder.create().fromRotation(rotation), 0L)
        val action = GoToBuilder.with(context)
            .withFrame(targetFrame.frame())
            .withMaxSpeed(0.3f)
            .withFinalOrientationPolicy(OrientationPolicy.ALIGN_X)
            .build()
        startMotion(action.async().run())
        return accepted("TURN")
    }

    private fun look(payload: JSONObject): JSONObject {
        val x = number(payload, "x", -5.0, 5.0)
        val y = number(payload, "y", -5.0, 5.0)
        val z = number(payload, "z", -2.0, 5.0)
        if (x == 0.0 && y == 0.0 && z == 0.0) throw IllegalArgumentException("look target cannot be origin")
        if (payload.optString("movement_policy", "head_only") != "head_only") {
            throw IllegalArgumentException("only head_only is supported")
        }
        val context = requireContext()
        val robotFrame = context.actuation.robotFrame()
        val targetFrame = context.mapping.makeFreeFrame()
        targetFrame.update(
            robotFrame,
            TransformBuilder.create().fromTranslation(Vector3(x, y, z)),
            0L
        )
        val action = LookAtBuilder.with(context).withFrame(targetFrame.frame()).build()
        action.policy = LookAtMovementPolicy.HEAD_ONLY
        val future = action.async().run()
        synchronized(lock) {
            currentLook?.takeIf { !it.isDone }?.requestCancellation()
            currentLook = future
        }
        future.thenConsume { completed ->
            synchronized(lock) { if (currentLook === completed) currentLook = null }
        }
        timeoutExecutor.schedule({
            synchronized(lock) {
                if (currentLook === future && !future.isDone) future.requestCancellation()
            }
        }, LOOK_TIMEOUT_SECONDS, TimeUnit.SECONDS)
        return accepted("LOOK")
    }

    private fun startMotion(future: Future<Void>) {
        synchronized(lock) { currentMotion = future }
        future.thenConsume { completed ->
            synchronized(lock) { if (currentMotion === completed) currentMotion = null }
        }
        timeoutExecutor.schedule({
            synchronized(lock) {
                if (currentMotion === future && !future.isDone) future.requestCancellation()
            }
        }, MOTION_TIMEOUT_SECONDS, TimeUnit.SECONDS)
    }

    private fun stop(): JSONObject {
        stopAll()
        return JSONObject().put("accepted", true).put("action", "STOP")
    }

    private fun stopAll() = synchronized(lock) {
        currentMotion?.takeIf { !it.isDone }?.requestCancellation()
        currentLook?.takeIf { !it.isDone }?.requestCancellation()
        currentMotion = null
        currentLook = null
    }

    private fun accepted(action: String) = JSONObject().put("accepted", true).put("action", action)

    fun shutdown() {
        stopAll()
        timeoutExecutor.shutdownNow()
    }

    companion object {
        private const val MOTION_TIMEOUT_SECONDS = 15L
        private const val LOOK_TIMEOUT_SECONDS = 15L
    }
}
