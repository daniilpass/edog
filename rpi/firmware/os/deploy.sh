#!/usr/bin/env bash
# Деплой прошивки с ПК на Raspberry Pi: только rpi/firmware/ (без venv, тестов, калибровки).
# На Pi она лежит в ~/edog/firmware — в архиве пути firmware/…, раскладка на Pi не зависит от репозитория.
# На Pi работает только API. Портал (ui/) и 3D-модели (cad/web/) остаются на ПК, см. ui/serve.py.
# Запуск на ПК (Git Bash):  rpi/firmware/os/deploy.sh [ssh-хост]     (по умолчанию алиас edog из ~/.ssh/config)
# Берёт текущие файлы рабочей копии, включая незакоммиченные.
set -euo pipefail

HOST="${1:-edog}"
RPI="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"   # rpi/
REMOTE_DIR="edog"

cd "$RPI"
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
        # безопасный перезапуск: при выключении серво собака сначала встаёт в монтажное положение,
        # ждём, пока сигнал снимется (иначе она падает под своим весом)
        if curl -sf http://127.0.0.1:8080/api/state | grep -q '\"enabled\":true'; then
          echo '==> серво включены: в монтажное положение и выключение'
          curl -sf -o /dev/null -X POST -H 'Content-Type: application/json' -d '{\"enabled\":false}' http://127.0.0.1:8080/api/enable || true
          for i in \$(seq 20); do curl -sf http://127.0.0.1:8080/api/state | grep -q '\"enabled\":false' && break; sleep 0.5; done
          curl -sf http://127.0.0.1:8080/api/state | grep -q '\"enabled\":false' \\
            && echo '==> сигнал снят' || echo '==> сигнал не снялся за 10 с — перезапуск всё равно'
        fi
        sudo -n systemctl restart edog
        # ждём не процесс, а ответ API: на Pi 3B+ (особенно со сброшенной частотой) старт занимает несколько секунд
        for i in \$(seq 30); do curl -sf -o /dev/null http://127.0.0.1:8080/api/state && break; sleep 1; done
        curl -sf -o /dev/null http://127.0.0.1:8080/api/state && echo \"==> edog перезапущен, API отвечает (\$i с)\" \\
          || { echo '==> edog не поднялся:'; journalctl -u edog -n 20 --no-pager; exit 1; }
      else
        echo '==> сервис ещё не установлен: sudo ~/edog/firmware/os/setup.sh'
      fi"
