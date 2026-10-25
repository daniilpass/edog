package com.edog.video

import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.net.nsd.NsdManager
import android.net.nsd.NsdServiceInfo
import java.io.DataInputStream
import java.io.IOException
import java.io.InputStream
import java.net.HttpURLConnection
import java.net.InetAddress
import java.net.URL
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import org.json.JSONObject

class BusyException : IOException("видео занято")

// Собака по API платы XIAO: /api/config (порт и путь видео), /api/state (camera.viewer),
// POST /api/camera/take, POST /api/patrol, POST /api/gait, POST /api/sleep и MJPEG-поток /stream?id=.
class Dog(context: Context, address: String) {
    private val host: String
    private val port: Int
    private val api: String
    private val streamUrl: String

    @Volatile private var conn: HttpURLConnection? = null

    init {
        val name = address.substringBefore(':')
        port = address.substringAfter(':', "8080").toIntOrNull() ?: 8080
        host = resolve(context, name)
        api = "http://$host:$port"
        val cam = request("GET", "/api/config").optJSONObject("camera")
            ?: throw IOException("у собаки нет камеры")
        if (!cam.optBoolean("ok")) throw IOException("камера не работает: ${cam.optString("error", "сенсор не найден")}")
        streamUrl = "http://$host:${cam.getInt("port")}${cam.getString("path")}"
    }

    val label get() = "$host:$port"

    fun state(): JSONObject = request("GET", "/api/state")

    // Кто смотрит видео: id зрителя, 0 — поток свободен.
    fun viewer(): Long = state().optJSONObject("camera")?.optLong("viewer") ?: 0

    // Спит ли собака (во сне камера выключена).
    fun asleep(state: JSONObject = state()): Boolean = state.optJSONObject("sleep")?.optBoolean("active") == true

    // Сон: спит — разбудить (встанет в стойку), не спит — усыпить. true — уснула.
    fun toggleSleep(): Boolean {
        val sleep = !asleep()
        request("POST", "/api/sleep", JSONObject().put("sleep", sleep))
        return sleep
    }

    fun take(id: Long) {
        request("POST", "/api/camera/take", JSONObject().put("id", id))
    }

    // Патруль: включён — остановить, выключен — запустить. true — запущен.
    fun togglePatrol(): Boolean {
        val off = state().optJSONObject("patrol")?.optString("state", "off") == "off"
        request("POST", "/api/patrol", JSONObject().put("action", if (off) "start" else "stop"))
        return off
    }

    // Стойка — профиль ходьбы: "base" или "base_stopper".
    fun gait(): String = state().optString("gait")

    // Базовая ↔ базовая на стопперах. Возвращает новый профиль.
    fun toggleGait(): String {
        val next = if (gait() == "base_stopper") "base" else "base_stopper"
        return request("POST", "/api/gait", JSONObject().put("name", next)).optString("gait", next)
    }

    // Читает поток, пока его не закроют; true — хотя бы один кадр пришёл. headers — заголовки кадра
    // (ключи в нижнем регистре): X-Detect, X-Free, X-Vision, X-Patrol, X-Map — что увидело зрение собаки.
    fun stream(id: Long, onFrame: (Bitmap, Map<String, String>) -> Unit): Boolean {
        val c = open("$streamUrl?id=$id")
        conn = c
        var got = false
        try {
            c.readTimeout = 5000
            if (c.responseCode == 409) throw BusyException()
            if (c.responseCode != 200) throw IOException("видео: ошибка ${c.responseCode}")
            val input = DataInputStream(c.inputStream.buffered(64 * 1024))
            while (true) {
                var length = -1
                val headers = HashMap<String, String>()
                while (true) {
                    val line = readLine(input) ?: return got
                    if (line.isEmpty()) { if (length >= 0) break else continue }
                    val colon = line.indexOf(':')
                    if (colon > 0) headers[line.substring(0, colon).trim().lowercase()] = line.substring(colon + 1).trim()
                    if (line.startsWith("Content-Length:", ignoreCase = true)) length = line.substringAfter(':').trim().toInt()
                }
                val jpeg = ByteArray(length)
                input.readFully(jpeg)
                BitmapFactory.decodeByteArray(jpeg, 0, length)?.let { onFrame(it, headers); got = true }
            }
        } catch (e: IOException) {
            if (got) return true   // поток оборвался или закрыт: забрали, собака перезагрузилась
            throw e
        } finally {
            conn = null
            c.disconnect()
        }
    }

    // Из другого потока: прервать stream().
    fun close() {
        conn?.disconnect()
    }

    private fun request(method: String, path: String, body: JSONObject? = null): JSONObject {
        val c = open(api + path)
        try {
            c.requestMethod = method
            if (body != null) {
                c.doOutput = true
                c.setRequestProperty("Content-Type", "application/json")
                c.outputStream.use { it.write(body.toString().toByteArray()) }
            }
            val ok = c.responseCode in 200..299
            val text = (if (ok) c.inputStream else c.errorStream)?.use { it.readBytes().decodeToString() } ?: ""
            val json = runCatching { JSONObject(text) }.getOrDefault(JSONObject())
            if (!ok) throw IOException(json.optString("detail", "ошибка ${c.responseCode}"))
            return json
        } finally {
            c.disconnect()
        }
    }

    private fun open(url: String) = (URL(url).openConnection() as HttpURLConnection).apply {
        connectTimeout = 5000
        readTimeout = 5000
        useCaches = false
    }

    companion object {
        // Строка до \r\n; null — поток кончился.
        private fun readLine(input: InputStream): String? {
            val sb = StringBuilder()
            while (true) {
                val b = input.read()
                if (b < 0) return null
                if (b == '\n'.code) return sb.toString().trimEnd('\r')
                sb.append(b.toChar())
            }
        }

        // Имя edog.local Android сам не находит: ищем службу http по mDNS (плата объявляет её под своим именем).
        private fun resolve(context: Context, name: String): String {
            if (!name.endsWith(".local", ignoreCase = true)) return name
            val instance = name.removeSuffix(".local").removeSuffix(".LOCAL")
            val nsd = context.getSystemService(Context.NSD_SERVICE) as NsdManager
            val done = CountDownLatch(1)
            var found: String? = null
            val discovery = object : NsdManager.DiscoveryListener {
                override fun onServiceFound(info: NsdServiceInfo) {
                    if (!info.serviceName.equals(instance, ignoreCase = true)) return
                    @Suppress("DEPRECATION")
                    nsd.resolveService(info, object : NsdManager.ResolveListener {
                        override fun onServiceResolved(info: NsdServiceInfo) {
                            @Suppress("DEPRECATION")
                            found = info.host?.hostAddress
                            done.countDown()
                        }
                        override fun onResolveFailed(info: NsdServiceInfo, error: Int) {}
                    })
                }
                override fun onServiceLost(info: NsdServiceInfo) {}
                override fun onDiscoveryStarted(type: String) {}
                override fun onDiscoveryStopped(type: String) {}
                override fun onStartDiscoveryFailed(type: String, error: Int) { done.countDown() }
                override fun onStopDiscoveryFailed(type: String, error: Int) {}
            }
            nsd.discoverServices("_http._tcp", NsdManager.PROTOCOL_DNS_SD, discovery)
            try {
                done.await(6, TimeUnit.SECONDS)
            } finally {
                runCatching { nsd.stopServiceDiscovery(discovery) }
            }
            found?.let { return it }
            // Вдруг система умеет .local сама
            return runCatching { InetAddress.getByName(name).hostAddress }.getOrNull()
                ?: throw IOException("$name не найден в сети")
        }
    }
}
