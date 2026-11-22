package com.robotvlm.pepperbridge

import fi.iki.elonen.NanoHTTPD
import org.json.JSONException
import org.json.JSONObject

class MotionBridgeServer(port: Int, private val controller: PepperMotionController) : NanoHTTPD(port) {
    override fun serve(session: IHTTPSession): Response {
        return try {
            when {
                session.method == Method.GET && session.uri == "/status" ->
                    json(Response.Status.OK, controller.status())
                session.method == Method.POST && session.uri in COMMAND_PATHS -> {
                    val files = HashMap<String, String>()
                    session.parseBody(files)
                    val body = files["postData"] ?: "{}"
                    json(Response.Status.ACCEPTED, controller.execute(session.uri, JSONObject(body)))
                }
                else -> json(Response.Status.NOT_FOUND, JSONObject().put("error", "not found"))
            }
        } catch (e: MotionRejectedException) {
            json(Response.Status.CONFLICT, JSONObject().put("error", e.message ?: "motion rejected"))
        } catch (e: IllegalArgumentException) {
            json(Response.Status.BAD_REQUEST, JSONObject().put("error", e.message ?: "invalid request"))
        } catch (e: JSONException) {
            json(Response.Status.BAD_REQUEST, JSONObject().put("error", "invalid JSON"))
        } catch (e: Exception) {
            json(Response.Status.INTERNAL_ERROR, JSONObject().put("error", e.message ?: "bridge error"))
        }
    }

    private fun json(status: Response.IStatus, value: JSONObject): Response =
        newFixedLengthResponse(status, "application/json; charset=utf-8", value.toString())

    companion object {
        private val COMMAND_PATHS = setOf("/look", "/turn", "/move", "/stop")
    }
}
