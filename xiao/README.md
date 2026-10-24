# XIAO ESP32-S3 Sense

Всё, что относится к плате Seeed XIAO ESP32-S3 Sense, которая заменит Raspberry Pi. Код для RPi лежит в `rpi/`.

```
firmware/xiao_test/  — тестовая прошивка: API состояния, видео камеры, звук микрофона, настройка Wi-Fi
ui/                  — страница проверки, запускается на ПК (на плате только API)
start.cmd            — запуск страницы: http://localhost:8001/
```

## Прошивка

Нужен [arduino-cli](https://arduino.github.io/arduino-cli/) с ядром ESP32 (`esp32:esp32`, ставится из `https://espressif.github.io/arduino-esp32/package_esp32_index.json`).

```
arduino-cli compile -b esp32:esp32:XIAO_ESP32S3:PSRAM=opi xiao/firmware/xiao_test
arduino-cli upload  -b esp32:esp32:XIAO_ESP32S3:PSRAM=opi -p COM3 xiao/firmware/xiao_test
```

Без `PSRAM=opi` камера не запустится.

## Wi-Fi

Сеть хранится на плате. Задать её можно двумя способами:

- в Serial (115200): `wifi <ssid> <пароль>`; ещё есть команды `forget` и `status`;
- со страницы, в блоке «Wi-Fi платы».

Если сеть не задана или к ней не подключиться за 15 с, плата поднимает точку доступа `EDOG-XIAO` (пароль `edog1234`, адрес `192.168.4.1`).

## API

| Порт | Путь | Что |
|---|---|---|
| 80 | `GET /api/status` | состояние: Wi-Fi, камера, микрофон, SD, PSRAM, температура |
| 80 | `GET /api/control?framesize=&vflip=&hmirror=` | настройки камеры |
| 80 | `GET /api/scan` | список сетей Wi-Fi |
| 80 | `POST /api/wifi` (`ssid`, `pass`) | сохранить сеть и перезагрузиться |
| 81 | `/stream` | видео MJPEG |
| 82 | `/audio` | сырой PCM 16 кГц, 16 бит, моно |
