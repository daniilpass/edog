package com.edog.video

import kotlin.math.atan
import kotlin.math.tan

// Что увидело зрение собаки — из заголовков кадра MJPEG (vision.cpp, patrol.cpp на плате).
class Vision(h: Map<String, String>) {
    class Person(val x1: Int, val y1: Int, val x2: Int, val y2: Int, val score: Int, val cm: Int)
    class MapPerson(val deg: Int, val cm: Int)

    val people: List<Person>
    val freeCm: List<Int>            // 16 полос свободного пола по ширине кадра, −1 — неизвестно
    val close: Float                 // 0 … 1: насколько кадр закрыт близким предметом
    val ageMs: Int
    val heightMm: Float
    val tiltDeg: Float
    val fovDeg: Float
    val state: String                // состояние патруля: off, walk, scan…
    val left: Int?                   // секунд до старта / до продолжения
    val greetings: Int
    val mapCm: List<Int>             // карта вокруг: 16 секторов по 22.5°, 0 — нос, по часовой; −1 — неизвестно
    val mapAge: List<Int>            // сколько секунд назад видели сектор
    val mapPeople: List<MapPerson>
    val targetDeg: Int?              // куда идёт, относительно носа

    init {
        people = (h["x-detect"] ?: "").split(';').filter { it.isNotBlank() }.mapNotNull { item ->
            val v = item.split(',').mapNotNull { it.trim().toIntOrNull() }
            if (v.size == 6) Person(v[0], v[1], v[2], v[3], v[4], v[5]) else null
        }
        freeCm = (h["x-free"] ?: "").split(',').mapNotNull { it.trim().toIntOrNull() }
        val vis = pairs(h["x-vision"])
        close = vis["close"]?.toFloatOrNull() ?: 0f
        ageMs = vis["age"]?.toIntOrNull() ?: 0
        heightMm = vis["h"]?.toFloatOrNull() ?: 110f
        tiltDeg = vis["tilt"]?.toFloatOrNull() ?: 12f
        fovDeg = vis["fov"]?.toFloatOrNull() ?: 56f
        val patrol = pairs(h["x-patrol"])
        state = patrol["state"] ?: ""
        left = patrol["left"]?.toIntOrNull()
        greetings = patrol["greet"]?.toIntOrNull() ?: 0
        val map = pairs(h["x-map"])
        mapCm = (map["d"] ?: "").split(',').mapNotNull { it.toIntOrNull() }
        mapAge = (map["a"] ?: "").split(',').mapNotNull { it.toIntOrNull() }
        mapPeople = (map["p"] ?: "").split(',').filter { it.contains(':') }.mapNotNull {
            val (d, c) = it.split(':')
            val deg = d.toIntOrNull() ?: return@mapNotNull null
            MapPerson(deg, c.toIntOrNull() ?: -1)
        }
        targetDeg = map["t"]?.toIntOrNull()
    }

    val hasMap get() = mapCm.size == 16

    // Строка кадра 640×480, на которой лежит точка пола на расстоянии mm.
    fun floorRow(mm: Float): Float {
        val f = 320f / tan(Math.toRadians(fovDeg / 2.0)).toFloat()
        val a = atan(heightMm / mm) - Math.toRadians(tiltDeg.toDouble()).toFloat()
        return 240f + f * tan(a)
    }

    companion object {
        val STATE_NAMES = mapOf(
            "off" to "патруль выключен", "countdown" to "старт через", "start" to "кивок — начинаю",
            "paused" to "пауза: управляет человек", "walk" to "иду", "scan" to "осматриваюсь",
            "turn" to "поворачиваю", "approach" to "человек — подхожу", "greet" to "приветствую",
            "leave" to "ухожу дальше", "backup" to "отхожу назад",
        )

        fun metres(cm: Int) = when {
            cm < 0 -> "?"
            cm >= 300 -> "далеко"
            else -> String.format("%.1f м", cm / 100f).replace('.', ',')
        }

        private fun pairs(text: String?): Map<String, String> =
            (text ?: "").split(';').filter { it.contains('=') }.associate {
                val k = it.indexOf('=')
                it.substring(0, k).trim() to it.substring(k + 1).trim()
            }
    }
}
