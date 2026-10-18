#!/usr/bin/env bash
# Деплой прошивки с ПК на Raspberry Pi: только firmware/ (без venv, тестов, калибровки).
# На Pi работает только API. Портал (ui/) и 3D-модели (cad/web/) остаются на ПК, см. ui/serve.py.
# Запуск на ПК (Git Bash):  firmware/os/deploy.sh [ssh-хост]     (по умолчанию алиас edog из ~/.ssh/config)
# Берёт текущие файлы рабочей копии, включая незакоммиченные.
set -euo pipefail

HOST="${1:-edog}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REMOTE_DIR="edog"

cd "$REPO"
echo "==> упаковка"
tar -czf - \
  --exclude='firmware/.venv' --exclude='__pycache__' --exclude='*.pyc' --exclude='.pytest_cache' \
  --exclude='firmware/tests' --exclude='firmware/config/calibration.json' \
  firmware \
  | ssh "$HOST" "set -e
      mkdir -p $REMOTE_DIR
      cd $REMOTE_DIR
      # код меняем целиком, чтобы не оставались удалённые файлы; venv и калибровку не трогаем.
      # cad/ — остатки старой схемы, когда Pi раздавала портал и модели
      rm -rf firmware/edog firmware/.pytest_cache cad
      tar -xzf -
      # сразу на SD: иначе при выключении питания вскоре после деплоя файлы остаются пустыми (ext4 пишет с задержкой)
      sync
      empty=\$(find firmware -path firmware/.venv -prune -o -type f -size 0 ! -name '__init__.py' -print)
      [ -z \"\$empty\" ] || { echo \"==> ПУСТЫЕ ФАЙЛЫ после распаковки:\"; echo \"\$empty\"; exit 1; }
      echo \"==> на Pi: \$(du -sh --exclude=.venv . | cut -f1) без venv\"
      if systemctl list-unit-files edog.service >/dev/null 2>&1; then
        sudo -n systemctl restart edog
        # ждём не процесс, а ответ API: на Pi 3B+ (особенно со сброшенной частотой) старт занимает несколько секунд
        for i in \$(seq 30); do curl -sf -o /dev/null http://127.0.0.1:8080/api/state && break; sleep 1; done
        curl -sf -o /dev/null http://127.0.0.1:8080/api/state && echo \"==> edog перезапущен, API отвечает (\$i с)\" \\
          || { echo '==> edog не поднялся:'; journalctl -u edog -n 20 --no-pager; exit 1; }
      else
        echo '==> сервис ещё не установлен: sudo ~/edog/firmware/os/setup.sh'
      fi"
