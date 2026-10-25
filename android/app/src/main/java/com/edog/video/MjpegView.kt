package com.edog.video

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.DashPathEffect
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.graphics.Typeface
import android.view.View

// Кадр вписывается в доступное место с сохранением пропорций, по краям — чёрные поля.
// Поверх — что увидела собака: рамки людей с подписью, граница свободного пола пунктиром, состояние патруля.
// Всё подписано текстом — не только цветом.
class MjpegView(context: Context) : View(context) {
    private var frame: Bitmap? = null
    private var vision: Vision? = null
    private val paint = Paint(Paint.FILTER_BITMAP_FLAG)
    private val dst = RectF()
    private val density = resources.displayMetrics.density
    private val box = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; color = Color.WHITE }
    private val floor = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; color = Color.rgb(255, 212, 0) }
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; typeface = Typeface.DEFAULT_BOLD }
    private val backdrop = Paint().apply { color = Color.argb(166, 0, 0, 0) }

    fun show(bitmap: Bitmap?, vision: Vision? = null) {
        frame = bitmap
        this.vision = vision
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        val b = frame ?: return
        val scale = minOf(width.toFloat() / b.width, height.toFloat() / b.height)
        val w = b.width * scale
        val h = b.height * scale
        dst.set((width - w) / 2, (height - h) / 2, (width + w) / 2, (height + h) / 2)
        canvas.drawBitmap(b, null, dst, paint)
        val v = vision ?: return
        // разметка — в координатах кадра 640×480
        val k = dst.width() / 640f
        fun x(px: Float) = dst.left + px * k
        fun y(py: Float) = dst.top + py * k
        text.textSize = 13 * density

        if (v.freeCm.size == 16) {
            floor.strokeWidth = 2.5f * density
            floor.pathEffect = DashPathEffect(floatArrayOf(8 * density, 6 * density), 0f)
            val path = Path()
            var started = false
            v.freeCm.forEachIndexed { i, cm ->
                if (cm < 0) { started = false; return@forEachIndexed }
                val row = minOf(479f, v.floorRow(cm * 10f))
                if (!started) path.moveTo(x(i * 40f), y(row)) else path.lineTo(x(i * 40f), y(row))
                path.lineTo(x(i * 40f + 40), y(row))
                started = true
            }
            canvas.drawPath(path, floor)
            for (i in intArrayOf(1, 5, 10, 14)) {
                val cm = v.freeCm[i]
                if (cm >= 0) label(canvas, Vision.metres(cm), x(i * 40f), y(minOf(470f, v.floorRow(cm * 10f))) - 4 * density)
            }
        }
        box.strokeWidth = 3 * density
        for (p in v.people) {
            canvas.drawRect(x(p.x1.toFloat()), y(p.y1.toFloat()), x(p.x2.toFloat()), y(p.y2.toFloat()), box)
            val dist = if (p.cm < 0) "расстояние ?" else Vision.metres(p.cm)
            label(canvas, "человек ${p.score}% · $dist", x(p.x1.toFloat()), maxOf(y(p.y1.toFloat()), dst.top + 18 * density))
        }
        var line = 0
        fun status(s: String) = label(canvas, s, dst.left + 8 * density, dst.top + (22 + 22 * line++) * density)
        Vision.STATE_NAMES[v.state]?.let { status(it) }
        status(String.format("близко: %.2f", v.close) + if (v.close > 0.7f) " — почти упёрлась" else "")
    }

    private fun label(canvas: Canvas, s: String, x: Float, y: Float) {
        val pad = 4 * density
        canvas.drawRect(x, y - text.textSize - pad, x + text.measureText(s) + 2 * pad, y + pad, backdrop)
        canvas.drawText(s, x + pad, y, text)
    }
}
