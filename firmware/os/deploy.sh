#!/usr/bin/env bash
# Деплой прошивки с ПК на Raspberry Pi: только firmware/ (без venv, тестов, калибровки) и cad/web/.
# Запуск на ПК (Git Bash):  firmware/os/deploy.sh [ssh-хост]     (по умолчанию алиас edog из ~/.ssh/config)
# Берёт текущие файлы рабочей копии, включая незакоммиченные.
set -euo pipefail

HOST="${1:-edog}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REMOTE_DIR="edog"

cd "$REPO"
echo "==> упаковка"
tar -czf - \
  --exclude='firmware/.venv' --exclude='__pycache__' --exclude='*.pyc' \
  --exclude='firmware/tests' --exclude='firmware/config/calibration.json' \
  firmware cad/web/manifest.json cad/web/*.stl \
  | ssh "$HOST" "set -e
      mkdir -p $REMOTE_DIR
      cd $REMOTE_DIR
      # код и модели меняем целиком, чтобы не оставались удалённые файлы; venv и калибровку не трогаем
      rm -rf firmware/edog cad/web
      tar -xzf -
      echo \"==> на Pi: \$(du -sh --exclude=.venv . | cut -f1) без venv\"
      if systemctl list-unit-files edog.service >/dev/null 2>&1; then
        sudo -n systemctl restart edog
        sleep 2
        systemctl is-active edog >/dev/null && echo '==> edog перезапущен' || { echo '==> edog не поднялся:'; journalctl -u edog -n 20 --no-pager; exit 1; }
      else
        echo '==> сервис ещё не установлен: sudo ~/edog/firmware/os/setup.sh'
      fi"
