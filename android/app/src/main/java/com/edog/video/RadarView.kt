package com.edog.video

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.DashPathEffect
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.graphics.Typeface
import android.view.View
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

// Радар: карта вокруг собаки из заголовка X-Map. Вид сверху, нос вверх, по часовой — вправо, радиус 3 м.
// Клин сектора — свободно до препятствия, бледнее — давно видели, штрих и «?» — не знаем, ◆ — человек,
// пунктир — что видит камера, стрелка — куда идёт. Под радаром — то же текстом.
class RadarView(context: Context) : View(context) {
    private var vision: Vision? = null
    private val density = resources.displayMetrics.density
    private val fill = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(91, 146, 240) }
    private val line = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; color = Color.LTGRAY }
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE }
    private val bold = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; typeface = Typeface.DEFAULT_BOLD }
    private val arc = RectF()

    fun show(v: Vision?) {
        vision = v
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        val v = vision ?: return
        if (!v.hasMap) return
        text.textSize = 11 * density
        bold.textSize = 13 * density
        val textH = 2 * 18 * density
        val size = min(width.toFloat(), height - textH)
        if (size <= 0) return
        val cx = width / 2f
        val cy = size / 2f
        val r = size / 2 - 14 * density
        val maxCm = 300f
        fun pt(deg: Float, cm: Float): Pair<Float, Float> {
            val a = Math.toRadians(deg.toDouble())
            val rr = min(cm, maxCm) / maxCm * r
            return Pair(cx + rr * sin(a).toFloat(), cy - rr * cos(a).toFloat())
        }
        // кольца
        line.strokeWidth = density
        line.pathEffect = null
        line.alpha = 100
        for (cm in intArrayOf(50, 100, 200, 300)) {
            canvas.drawCircle(cx, cy, cm / maxCm * r, line)
            canvas.drawText("${cm / 100f} м".replace(".0", "").replace('.', ','), cx + 3 * density, cy - cm / maxCm * r + 12 * density, text)
        }
        line.alpha = 255
        // сектора
        val step = 360f / 16
        v.mapCm.forEachIndexed { i, cm ->
            val start = i * step - step / 2 - 90
            if (cm < 0) {
                val rr = r * 0.35f
                arc.set(cx - rr, cy - rr, cx + rr, cy + rr)
                line.alpha = 90
                val path = Path().apply { moveTo(cx, cy); arcTo(arc, start, step); close() }
                canvas.save()
                canvas.clipPath(path)
                var k = -size
                while (k < 2 * size) { canvas.drawLine(k, 0f, k + size, size, line); k += 7 * density }
                canvas.restore()
                line.alpha = 255
                val (tx, ty) = pt(i * step, maxCm * 0.25f)
                canvas.drawText("?", tx - 3 * density, ty + 4 * density, text)
                return@forEachIndexed
            }
            val rr = min(cm.toFloat(), maxCm) / maxCm * r
            arc.set(cx - rr, cy - rr, cx + rr, cy + rr)
            val age = v.mapAge.getOrElse(i) { 0 }.coerceAtLeast(0)
            fill.alpha = (255 * maxOf(0.2f, 0.75f * (1 - age / 90f))).toInt()
            canvas.drawArc(arc, start, step, true, fill)
        }
        // что видит камера
        line.pathEffect = DashPathEffect(floatArrayOf(4 * density, 4 * density), 0f)
        for (d in floatArrayOf(-v.fovDeg / 2, v.fovDeg / 2)) {
            val (x, y) = pt(d, maxCm)
            canvas.drawLine(cx, cy, x, y, line)
        }
        line.pathEffect = null
        // куда идёт
        v.targetDeg?.let { t ->
            val (tx, ty) = pt(t.toFloat(), maxCm * 0.55f)
            line.strokeWidth = 3 * density
            canvas.drawLine(cx, cy, tx, ty, line)
            val a = atan2(ty - cy, tx - cx)
            val head = Path().apply {
                moveTo(tx, ty)
                lineTo(tx - 10 * density * cos(a - 0.5f), ty - 10 * density * sin(a - 0.5f))
                lineTo(tx - 10 * density * cos(a + 0.5f), ty - 10 * density * sin(a + 0.5f))
                close()
            }
            canvas.drawPath(head, bold)
            line.strokeWidth = density
        }
        // люди
        for (p in v.mapPeople) {
            val (x, y) = pt(p.deg.toFloat(), if (p.cm < 0) 150f else p.cm.toFloat())
            val d = 7 * density
            canvas.drawPath(Path().apply { moveTo(x, y - d); lineTo(x + d, y); lineTo(x, y + d); lineTo(x - d, y); close() }, bold)
            canvas.drawText("человек ${Vision.metres(p.cm)}", x + 10 * density, y + 4 * density, bold)
        }
        // собака
        canvas.drawRect(cx - 5 * density, cy - 10 * density, cx + 5 * density, cy + 10 * density, bold)
        // текстом
        val near = v.mapCm.withIndex().filter { it.value >= 0 }.minByOrNull { it.value }
        val state = (Vision.STATE_NAMES[v.state] ?: "") + (v.left?.let { " $it с" } ?: "")
        canvas.drawText(state, 8 * density, size + 16 * density, bold)
        val deg = near?.let { val a = it.index * step; (if (a > 180) a - 360 else a).toInt() }
        val info = (if (near != null) "ближе всего ${Vision.metres(near.value)} ($deg°)" else "вокруг пока ничего не видно") +
            (if (v.mapPeople.isNotEmpty()) " · людей: ${v.mapPeople.size}" else "") + " · приветствий ${v.greetings}"
        canvas.drawText(info, 8 * density, size + 34 * density, text)
    }
}
