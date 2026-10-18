#!/usr/bin/env bash
# Самодиагностика Raspberry Pi под EDOG (можно без PCA9685 и серво).
# Запуск: ~/edog/firmware/os/check.sh
set -uo pipefail
export PATH="$PATH:/usr/sbin:/sbin"  # i2cdetect лежит в /usr/sbin, а в ssh-сессии без логина его нет в PATH

FIRMWARE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${1:-8080}"
fails=0

ok()   { printf '  \e[32mOK\e[0m    %s\n' "$*"; }
warn() { printf '  \e[33mWARN\e[0m  %s\n' "$*"; }
fail() { printf '  \e[31mFAIL\e[0m  %s\n' "$*"; fails=$((fails + 1)); }
# без pipefail: иначе `… | grep -q` при раннем выходе grep даёт SIGPIPE и ложный FAIL
check() { if (set +o pipefail; eval "$2") >/dev/null 2>&1; then ok "$1"; else fail "$1${3:+ — $3}"; fi; }

echo "Система"
[[ "$(uname -m)" == "aarch64" ]] && ok "64-bit ОС" || warn "ОС не 64-bit ($(uname -m)) — зависимости могут собираться из исходников"
ok "$(. /etc/os-release && echo "$PRETTY_NAME"), $(python3 --version)"
throttled="$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)"
[[ -z "$throttled" || "$throttled" == "0x0" ]] && ok "питание без просадок" || warn "get_throttled=$throttled — слабое питание RPi"

echo "I²C"
check "/dev/i2c-1 есть (I²C включён)" "test -e /dev/i2c-1" "sudo raspi-config nonint do_i2c 0 и перезагрузка"
# вывод сначала в переменную: с pipefail `… | grep -q` даёт SIGPIPE у i2cdetect и ложный «не найдена»
i2c_map="$(i2cdetect -y 1 2>/dev/null)"
if grep -q ' 40' <<<"$i2c_map"; then ok "PCA9685 на 0x40"; else warn "PCA9685 не найдена (ожидаемо, пока плата не подключена)"; fi

echo "Права"
for g in i2c input bluetooth; do
  check "сервис в группе $g" "id -nG \"$(systemctl show -p User --value edog)\" | grep -qw $g" "перезапустить setup.sh"
done

echo "Python"
check "venv на месте" "test -x '$FIRMWARE_DIR/.venv/bin/python'" "запустить setup.sh"
check "fastapi, uvicorn, smbus2, evdev импортируются" \
  "'$FIRMWARE_DIR/.venv/bin/python' -c 'import fastapi, uvicorn, smbus2, evdev'"

echo "Сервис"
check "edog запущен" "systemctl is-active --quiet edog" "journalctl -u edog -n 50"
check "edog в автозагрузке" "systemctl is-enabled --quiet edog"
state="$(curl -sf "http://127.0.0.1:$PORT/api/state")"
if [[ -n "$state" ]]; then
  ok "API отвечает на :$PORT"
  python3 - "$state" <<'EOF'
import json, sys
s = json.loads(sys.argv[1])
d, g = s["driver"], s["gamepad"]
print(f"        драйвер: {d['kind']}" + (f" ({d['error']})" if d["error"] else ""))
print(f"        геймпад: {g.get('name') or 'не подключён'}")
print(f"        серво: {'включены' if s['enabled'] else 'выключены'}, режим {s['mode']}")
EOF
else
  fail "API не отвечает на :$PORT"
fi
check "CORS разрешён (портал открывается с ПК)" \
  "curl -sf -o /dev/null -D - -H 'Origin: http://localhost:8000' http://127.0.0.1:$PORT/api/config | grep -qi '^access-control-allow-origin'"

echo "Bluetooth"
check "адаптер включён" "bluetoothctl show | grep -q 'Powered: yes'" "rfkill unblock bluetooth; sudo systemctl restart bluetooth"
paired="$(bluetoothctl devices Paired 2>/dev/null | sed 's/^Device //')"
[[ -n "$paired" ]] && ok "сопряжены: $paired" || warn "сопряжённых устройств нет"
pad="$("$FIRMWARE_DIR/.venv/bin/python" - 2>/dev/null <<'EOF'
import evdev
from evdev import ecodes
for path in evdev.list_devices():
    d = evdev.InputDevice(path)
    if ecodes.BTN_SOUTH in d.capabilities().get(ecodes.EV_KEY, []):
        print(f"{d.name} ({path})")
EOF
)"
[[ -n "$pad" ]] && ok "геймпад в /dev/input: $pad" || warn "геймпад не виден в /dev/input (не подключён или нет доступа)"

echo
[[ $fails -eq 0 ]] && echo "Ошибок нет." || echo "Ошибок: $fails"
exit $fails
