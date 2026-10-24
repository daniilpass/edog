package com.edog.video

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.RectF
import android.view.View

// Кадр вписывается в доступное место с сохранением пропорций, по краям — чёрные поля.
class MjpegView(context: Context) : View(context) {
    private var frame: Bitmap? = null
    private val paint = Paint(Paint.FILTER_BITMAP_FLAG)
    private val dst = RectF()

    fun show(bitmap: Bitmap?) {
        frame = bitmap
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        val b = frame ?: return
        val scale = minOf(width.toFloat() / b.width, height.toFloat() / b.height)
        val w = b.width * scale
        val h = b.height * scale
        dst.set((width - w) / 2, (height - h) / 2, (width + w) / 2, (height + h) / 2)
        canvas.drawBitmap(b, null, dst, paint)
    }
}
