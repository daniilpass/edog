package com.edog.video

import android.app.Activity
import android.app.AlertDialog
import android.content.Context
import android.content.res.Configuration
import android.graphics.Color
import android.os.Build
import android.os.Bundle
import android.text.InputType
import android.view.Gravity
import android.view.View
import android.view.WindowInsets
import android.view.WindowInsetsController
import android.view.WindowManager
import android.widget.Button
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.TextView
import android.widget.Toast
import java.io.IOException
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import kotlin.random.Random

// Видео с камеры собаки и радар (карта вокруг неё) — что видит собака в патруле. Поток у собаки один: если его смотрит другое устройство
// (портал, другой телефон), приложение спрашивает, переключить ли видео сюда.
class MainActivity : Activity() {
    private lateinit var video: MjpegView
    private lateinit var radar: RadarView
    private lateinit var message: TextView
    private lateinit var take: Button
    private lateinit var patrol: Button
    private lateinit var stance: Button
    private lateinit var info: TextView

    private val prefs by lazy { getSharedPreferences("edog", Context.MODE_PRIVATE) }
    private val myId by lazy {
        prefs.getLong("id", 0).takeIf { it != 0L }
            ?: (1 + Random.nextLong(0x7fffffff)).also { prefs.edit().putLong("id", it).apply() }
    }
    private var address
        get() = prefs.getString("address", DEFAULT_ADDRESS)!!
        set(v) = prefs.edit().putString("address", v).apply()

    private var worker: Thread? = null
    @Volatile private var dog: Dog? = null
    private val answers = LinkedBlockingQueue<Boolean>()   // ответ на вопрос «переключить сюда?»
    private var dialog: AlertDialog? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        val dp = resources.displayMetrics.density

        video = MjpegView(this)
        message = TextView(this).apply {
            setTextColor(Color.LTGRAY)
            textSize = 16f
            gravity = Gravity.CENTER
        }
        take = Button(this).apply {
            text = "Переключить сюда"
            visibility = View.GONE
            setOnClickListener { answers.offer(true) }
        }
        val center = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.CENTER
            setPadding((24 * dp).toInt(), 0, (24 * dp).toInt(), 0)
            addView(message)
            addView(take, LinearLayout.LayoutParams(-2, -2).apply { gravity = Gravity.CENTER; topMargin = (12 * dp).toInt() })
        }
        info = TextView(this).apply {
            setTextColor(Color.LTGRAY)
            setShadowLayer(3f, 0f, 0f, Color.BLACK)
            textSize = 12f
        }
        val settings = Button(this).apply {
            text = "Адрес"
            alpha = 0.7f
            setOnClickListener { editAddress() }
        }
        patrol = Button(this).apply {
            text = "Патруль"
            alpha = 0.7f
            setOnClickListener { togglePatrol() }
        }
        stance = Button(this).apply {
            text = "Стойка"
            alpha = 0.7f
            setOnClickListener { toggleStance() }
        }
        val buttons = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            addView(patrol)
            addView(stance)
        }
        val overlay = FrameLayout(this).apply {
            addView(center, FrameLayout.LayoutParams(-1, -1))
            addView(buttons, FrameLayout.LayoutParams(-2, -2, Gravity.TOP or Gravity.START))
            addView(info, FrameLayout.LayoutParams(-2, -2, Gravity.BOTTOM or Gravity.START))
            addView(settings, FrameLayout.LayoutParams(-2, -2, Gravity.TOP or Gravity.END))
        }
        // Подписи и кнопки не прячутся под вырез экрана и системные панели
        overlay.setOnApplyWindowInsetsListener { v, insets ->
            if (Build.VERSION.SDK_INT >= 30) {
                val i = insets.getInsets(WindowInsets.Type.systemBars() or WindowInsets.Type.displayCutout())
                val pad = (8 * dp).toInt()
                v.setPadding(i.left + pad, i.top + pad, i.right + pad, i.bottom + pad)
            }
            insets
        }
        // Радар — рядом с видео: в портретной ориентации под ним, в альбомной справа. Пока собака не прислала
        // карту (старая прошивка), его нет.
        radar = RadarView(this).apply { visibility = View.GONE }
        val portrait = resources.configuration.orientation != Configuration.ORIENTATION_LANDSCAPE
        val content = LinearLayout(this).apply {
            orientation = if (portrait) LinearLayout.VERTICAL else LinearLayout.HORIZONTAL
            addView(FrameLayout(this@MainActivity).apply {
                addView(video, FrameLayout.LayoutParams(-1, -1))
                addView(overlay, FrameLayout.LayoutParams(-1, -1))
            }, LinearLayout.LayoutParams(if (portrait) -1 else 0, if (portrait) 0 else -1, 1.6f))
            addView(radar, LinearLayout.LayoutParams(if (portrait) -1 else 0, if (portrait) 0 else -1, 1f).apply {
                val m = (8 * dp).toInt()
                setMargins(m, m, m, m)
            })
        }
        setContentView(FrameLayout(this).apply {
            setBackgroundColor(Color.BLACK)
            addView(content, FrameLayout.LayoutParams(-1, -1))
        })
        hideSystemBars()
    }

    override fun onStart() {
        super.onStart()
        startWorker()
    }

    // В фоне поток не держим: собака освобождает видео для других
    override fun onStop() {
        super.onStop()
        stopWorker()
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) hideSystemBars()
    }

    private fun hideSystemBars() {
        if (Build.VERSION.SDK_INT >= 30) {
            window.insetsController?.apply {
                hide(WindowInsets.Type.systemBars())
                systemBarsBehavior = WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
            }
        } else {
            @Suppress("DEPRECATION")
            window.decorView.systemUiVisibility = View.SYSTEM_UI_FLAG_FULLSCREEN or
                View.SYSTEM_UI_FLAG_HIDE_NAVIGATION or View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
        }
    }

    private fun editAddress() {
        val field = EditText(this).apply {
            setText(address)
            inputType = InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_URI
            setSingleLine()
        }
        AlertDialog.Builder(this)
            .setTitle("Адрес собаки")
            .setMessage("Имя или IP и порт API, например edog.local:8080 или 192.168.50.130:8080")
            .setView(field)
            .setPositiveButton("Сохранить") { _, _ ->
                val v = field.text.toString().trim().removePrefix("http://").trimEnd('/')
                address = v.ifEmpty { DEFAULT_ADDRESS }
                stopWorker()
                startWorker()
            }
            .setNegativeButton("Отмена", null)
            .show()
    }

    // Запустить или остановить патруль. Надпись на кнопке потом поправит состояние из кадров видео.
    private fun togglePatrol() {
        val d = dog ?: return toast("Нет связи с собакой")
        patrol.isEnabled = false
        Thread {
            val result = try {
                if (d.togglePatrol()) "Патруль запущен" else "Патруль остановлен"
            } catch (e: IOException) {
                "Патруль: ${e.message}"
            }
            ui { patrol.isEnabled = true; toast(result) }
        }.start()
    }

    // Стойка: базовая ↔ базовая на стопперах.
    private fun toggleStance() {
        val d = dog ?: return toast("Нет связи с собакой")
        stance.isEnabled = false
        Thread {
            val result = try {
                val g = d.toggleGait()
                ui { showStance(g) }
                "Стойка: ${STANCE_NAMES[g] ?: g}"
            } catch (e: IOException) {
                "Стойка: ${e.message}"
            }
            ui { stance.isEnabled = true; toast(result) }
        }.start()
    }

    private fun showStance(gait: String) {
        stance.text = "Стойка: ${STANCE_NAMES[gait] ?: gait}"
    }

    private fun toast(text: String) = Toast.makeText(this, text, Toast.LENGTH_SHORT).show()

    private fun startWorker() {
        answers.clear()
        worker = Thread(::run, "video").also { it.start() }
    }

    private fun stopWorker() {
        worker?.interrupt()
        dog?.close()
        dog = null
        worker = null
        hideQuestion()
    }

    // ---------- рабочий поток ----------

    private fun run() {
        var watched = false   // до этого видео шло сюда: если теперь занято — его забрали
        while (!Thread.interrupted()) {
            try {
                val d = dog ?: run {
                    ui { status("Поиск собаки: $address…"); info.text = "" }
                    Dog(this, address).also { dog = it }
                }
                val viewer = d.viewer()
                if (viewer != 0L && viewer != myId && !askTake(d, watched)) return
                val gait = d.gait()
                ui { status("Подключение к видео…"); info.text = d.label; showStance(gait) }
                watched = d.stream(myId) { bmp, headers ->
                    val v = Vision(headers)
                    ui {
                        status("")
                        video.show(bmp, v)
                        if (v.state.isNotEmpty()) patrol.text = if (v.state == "off") "Запустить патруль" else "Остановить патруль"
                        radar.show(v)
                        radar.visibility = if (v.hasMap) View.VISIBLE else View.GONE
                    }
                }
            } catch (_: BusyException) {
                // поток только что забрал другой зритель: решит следующий опрос состояния
            } catch (_: InterruptedException) {
                return
            } catch (e: IOException) {
                if (Thread.currentThread().isInterrupted) return
                dog = null
                watched = false
                ui { video.show(null); status("Нет связи с собакой: ${e.message}\nПовтор…") }
                if (!pause(3000)) return
            }
            if (!pause(500)) return
        }
    }

    // Видео смотрит другое устройство: спросить, забрать ли его. false — поток прерван (приложение свёрнуто).
    private fun askTake(d: Dog, kicked: Boolean): Boolean {
        val question = if (kicked) "Видео переключили на другое устройство. Вернуть сюда?"
        else "Видео смотрит другое устройство. Переключить сюда?"
        answers.clear()
        ui {
            video.show(null)
            status("Видео на другом устройстве")
            dialog = AlertDialog.Builder(this)
                .setMessage(question)
                .setPositiveButton("Переключить") { _, _ -> answers.offer(true) }
                .setNegativeButton("Нет") { _, _ -> answers.offer(false) }
                .setOnCancelListener { answers.offer(false) }
                .show()
        }
        try {
            while (true) {
                when (answers.poll(2, TimeUnit.SECONDS)) {
                    true -> { d.take(myId); return true }
                    false -> ui { take.visibility = View.VISIBLE }   // отказались — кнопка остаётся на экране
                    null -> if (d.viewer() == 0L) return true          // другое устройство отключилось
                }
            }
        } catch (_: InterruptedException) {
            return false
        } finally {
            ui { hideQuestion() }
        }
    }

    private fun hideQuestion() {
        dialog?.dismiss()
        dialog = null
        take.visibility = View.GONE
    }

    private fun status(text: String) {
        message.text = text
    }

    private fun ui(block: () -> Unit) = runOnUiThread(block)

    private fun pause(ms: Long) = try { Thread.sleep(ms); true } catch (_: InterruptedException) { false }

    companion object {
        const val DEFAULT_ADDRESS = "edog.local:8080"
        val STANCE_NAMES = mapOf("base" to "базовая", "base_stopper" to "на стопперах")
    }
}
