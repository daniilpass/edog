"""Модель ESP-DL (.espdl) → заголовок C с массивом для прошивки собаки.

arduino-cli не умеет встраивать двоичные файлы, поэтому модель лежит в прошивке массивом во флеше.
Загрузчик ESP-DL читает формат EDL2 прямо оттуда; массив выровнен по 16 байт.

    python xiao/models/espdl_to_header.py   → xiao/firmware/edog/pedestrian_model.h
"""
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE / "pedestrian_detect_pico_s8_v1.espdl"
DST = HERE.parent / "firmware" / "edog" / "pedestrian_model.h"


def main():
    data = SRC.read_bytes()
    if data[:4] != b"EDL2":
        raise SystemExit(f"{SRC.name}: не формат EDL2")
    lines = [
        f"// Сгенерировано xiao/models/espdl_to_header.py из {SRC.name} (ESP-DL pedestrian_detect, MIT).",
        "// Не править руками.",
        "#pragma once",
        "#include <stdint.h>",
        f"alignas(16) static const uint8_t PEDESTRIAN_MODEL[{len(data)}] = {{",
    ]
    for i in range(0, len(data), 32):
        lines.append(",".join(str(b) for b in data[i:i + 32]) + ",")
    lines.append("};")
    DST.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    print(f"{DST}: {len(data)} байт")


if __name__ == "__main__":
    main()
