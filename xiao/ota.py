"""Прошивка собаки (XIAO ESP32-S3) по воздуху: собрать, залить через API, дождаться новой прошивки.

    python xiao/ota.py                          собрать и залить на edog.local:8080
    python xiao/ota.py --dog 192.168.50.130:8080
    python xiao/ota.py --bin xiao/build/edog.ino.bin   залить готовый файл без сборки
    python xiao/ota.py --build-only             только собрать: xiao/build/edog.ino.bin (для портала)

Включённые серво выключаются перед загрузкой: собака встаёт в монтажное положение (как при деплое на Pi).
Если запись оборвалась или образ битый, плата остаётся на старой прошивке.
Коды выхода: 0 — новая прошивка работает, 1 — нет (причина в выводе).
Зависимостей нет, кроме arduino-cli с ядром esp32 и библиотеками (см. xiao/README.md).
"""

import argparse
import json
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SKETCH = ROOT / "firmware" / "edog"
BUILD = ROOT / "build"
FQBN = "esp32:esp32:XIAO_ESP32S3:PSRAM=opi"
CLI_FALLBACK = Path(r"C:\Program Files\Arduino CLI\arduino-cli.exe")
BOOT_TIMEOUT_S = 60
POWER_DOWN_WAIT_S = 10   # на собаке выключение не дольше 5 с (POWER_DOWN_TIMEOUT_S)


def fail(msg: str) -> None:
    print(f"ОШИБКА: {msg}")
    sys.exit(1)


def arduino_cli() -> str:
    cli = shutil.which("arduino-cli") or (str(CLI_FALLBACK) if CLI_FALLBACK.exists() else None)
    if not cli:
        fail("arduino-cli не найден (ни в PATH, ни в Program Files)")
    return cli


def build() -> Path:
    print(f"Сборка {SKETCH.relative_to(ROOT.parent)} …")
    r = subprocess.run([arduino_cli(), "compile", "-b", FQBN, "--output-dir", str(BUILD), str(SKETCH)],
                       capture_output=True, text=True, encoding="utf-8", errors="replace")
    if r.returncode:
        print(r.stdout[-4000:], r.stderr[-4000:], sep="\n")
        fail("сборка не удалась")
    for line in r.stdout.splitlines():
        if "Скетч" in line or "Sketch uses" in line:
            print(" ", line.strip())
    return BUILD / "edog.ino.bin"


def get(url: str, timeout: float = 5) -> dict:
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        return json.load(resp)


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dog", default="edog.local:8080", help="адрес API собаки (по умолчанию edog.local:8080)")
    ap.add_argument("--bin", type=Path, help="готовый .bin вместо сборки")
    ap.add_argument("--build-only", action="store_true", help="только собрать")
    args = ap.parse_args()

    image = args.bin or build()
    if not image.exists():
        fail(f"нет файла {image}")
    data = image.read_bytes()
    # образ ESP32 с описанием приложения (магия 0xABCD5432 по смещению 32) — не загрузчик и не merged
    if len(data) < 36 or data[0] != 0xE9 or int.from_bytes(data[32:36], "little") != 0xABCD5432:
        fail(f"{image} — не прошивка ESP32 (нужен edog.ino.bin, не merged/bootloader)")
    print(f"Файл: {image} ({len(data) / 1024:.0f} КБ)")
    if args.build_only:
        return

    api = f"http://{args.dog}"
    try:
        root = get(api + "/")
        state = get(api + "/api/state")
    except (urllib.error.URLError, OSError) as e:
        fail(f"собака {args.dog} не отвечает ({e}). Включена? Верный адрес? Можно указать IP: --dog <ip>:8080")
    if "firmware" not in root:
        fail(f"{args.dog} — не XIAO с прошивкой по воздуху ({root.get('board', 'это Raspberry Pi?')})")
    if state.get("enabled"):
        # Как при деплое на Pi: собака встаёт в монтажное положение, потом сигнал снимается.
        # Новая прошивка делает это и сама, прежняя без этого не примет обновление.
        print("Серво включены: собака встаёт в монтажное положение …")
        req = urllib.request.Request(api + "/api/enable", data=json.dumps({"enabled": False}).encode(),
                                     method="POST", headers={"Content-Type": "application/json"})
        urllib.request.urlopen(req, timeout=5).close()
        deadline = time.monotonic() + POWER_DOWN_WAIT_S
        while get(api + "/api/state").get("enabled"):
            if time.monotonic() > deadline:
                fail(f"серво не выключились за {POWER_DOWN_WAIT_S} с")
            time.sleep(0.3)
    old = root["firmware"]
    print(f"Собака {args.dog}: прошивка {old}")

    print("Загрузка …")
    t = time.monotonic()
    req = urllib.request.Request(api + "/api/firmware", data=data, method="POST",
                                 headers={"Content-Type": "application/octet-stream"})
    try:
        with urllib.request.urlopen(req, timeout=180) as resp:
            print(f"  {json.load(resp).get('message', 'записано')} ({time.monotonic() - t:.0f} с)")
    except urllib.error.HTTPError as e:
        try:
            detail = json.load(e).get("detail")
        except ValueError:
            detail = None
        fail(f"собака не приняла прошивку: {detail or e}. Работает старая")
    except (urllib.error.URLError, OSError) as e:
        fail(f"передача оборвалась ({e}). Плата остаётся на старой прошивке")

    print("Жду перезагрузки …")
    deadline, went_down = time.monotonic() + BOOT_TIMEOUT_S, False
    time.sleep(2)
    while time.monotonic() < deadline:
        try:
            fw = get(api + "/", timeout=3).get("firmware")
        except (urllib.error.URLError, OSError):
            went_down = True
            time.sleep(2)
            continue
        if fw != old:
            print(f"Готово: новая прошивка {fw} работает")
            return
        if went_down:
            fail(f"собака перезагрузилась, но прошивка прежняя ({fw}): залит тот же файл?")
        time.sleep(2)
    fail(f"собака не вернулась за {BOOT_TIMEOUT_S} с — смотрите Serial (115200)")


if __name__ == "__main__":
    main()
