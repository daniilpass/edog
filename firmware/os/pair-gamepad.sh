#!/usr/bin/env bash
# Сопряжение Bluetooth-геймпада по SSH (то же умеет веб-портал).
# 1. Переведите геймпад в режим сопряжения.  2. ./pair-gamepad.sh
set -euo pipefail

SCAN_SECONDS="${1:-10}"

bluetoothctl power on >/dev/null
echo "Поиск ${SCAN_SECONDS} с..."
bluetoothctl --timeout "$SCAN_SECONDS" scan on >/dev/null

mapfile -t DEVICES < <(bluetoothctl devices | sed -n 's/^Device //p')
if [[ ${#DEVICES[@]} -eq 0 ]]; then
  echo "Ничего не найдено. Геймпад точно мигает в режиме сопряжения?"
  exit 1
fi

for i in "${!DEVICES[@]}"; do
  echo "  [$i] ${DEVICES[$i]}"
done
read -rp "Номер геймпада: " N
MAC="${DEVICES[$N]%% *}"

bluetoothctl pair "$MAC"
bluetoothctl trust "$MAC"
bluetoothctl connect "$MAC"
echo "Готово. Проверка: evtest (устройство с BTN_SOUTH)"
