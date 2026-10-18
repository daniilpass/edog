#!/usr/bin/env bash
# Настройка свежей Raspberry Pi OS Lite (64-bit) под EDOG.
# Запуск из клона репозитория на Pi:  sudo ./firmware/os/setup.sh
# Повторный запуск безопасен.
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Запускать через sudo" >&2
  exit 1
fi

RUN_USER="${SUDO_USER:-pi}"
FIRMWARE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$FIRMWARE_DIR/.venv"
BOOT_CONFIG=/boot/firmware/config.txt
[[ -f $BOOT_CONFIG ]] || BOOT_CONFIG=/boot/config.txt

echo "==> Пользователь: $RUN_USER, прошивка: $FIRMWARE_DIR"

echo "==> Пакеты"
apt-get update
apt-get install -y --no-install-recommends \
  python3-venv python3-pip python3-evdev \
  i2c-tools bluez evtest git

echo "==> I²C (400 кГц)"
raspi-config nonint do_i2c 0
if ! grep -q '^dtparam=i2c_arm_baudrate=' "$BOOT_CONFIG"; then
  echo 'dtparam=i2c_arm_baudrate=400000' >> "$BOOT_CONFIG"
fi

echo "==> Bluetooth"
rfkill unblock bluetooth || true
systemctl enable --now bluetooth
MAIN_CONF=/etc/bluetooth/main.conf
set_bt() {  # set_bt <ключ> <значение>: раскомментировать/заменить в секции [General]/[Policy]
  if grep -qE "^#?\s*$1\s*=" "$MAIN_CONF"; then
    sed -i -E "s|^#?\s*$1\s*=.*|$1 = $2|" "$MAIN_CONF"
  else
    echo "$1 = $2" >> "$MAIN_CONF"
  fi
}
set_bt FastConnectable true          # быстрое переподключение геймпада
set_bt JustWorksRepairing always     # повторное сопряжение без вопросов (Xbox, 8BitDo)
set_bt AutoEnable true               # адаптер включается при загрузке
systemctl restart bluetooth

echo "==> Группы: i2c, input, bluetooth"
usermod -aG i2c,input,bluetooth "$RUN_USER"

echo "==> Python venv"
if [[ ! -d $VENV ]]; then
  sudo -u "$RUN_USER" python3 -m venv --system-site-packages "$VENV"
fi
sudo -u "$RUN_USER" "$VENV/bin/pip" install --upgrade pip
sudo -u "$RUN_USER" "$VENV/bin/pip" install -r "$FIRMWARE_DIR/requirements.txt"

echo "==> systemd: edog.service"
cat > /etc/systemd/system/edog.service <<EOF
[Unit]
Description=EDOG robot (servo control, gamepad, web portal)
After=network-online.target bluetooth.target
Wants=network-online.target

[Service]
User=$RUN_USER
Group=$RUN_USER
SupplementaryGroups=i2c input bluetooth
WorkingDirectory=$FIRMWARE_DIR
ExecStart=$VENV/bin/python -m edog
Restart=on-failure
RestartSec=2
# Остановка сервиса снимает сигнал с серво (собака обмякнет)
KillSignal=SIGINT
TimeoutStopSec=5

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable edog.service
systemctl restart edog.service

HOST="$(hostname)"
echo
echo "Готово. Портал: http://$HOST.local:8080/"
echo "I²C-шина: i2cdetect -y 1 (PCA9685 = 0x40)"
echo "Логи: journalctl -u edog -f"
echo "Если I²C включился впервые — перезагрузите: sudo reboot"
