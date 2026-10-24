"""Локальный сервер страницы XIAO: отдаёт xiao/ui/. На плате только API, страница — на ПК.

    python xiao/ui/serve.py              → http://localhost:8001/?board=192.168.50.85
    python xiao/ui/serve.py --lan        → доступен и с телефона в той же сети
    python xiao/ui/serve.py --port 9001

Адрес платы задаётся в шапке страницы или ссылкой ?board=…, по умолчанию edog-xiao.local.
Порт 8001, чтобы не мешать порталу RPi (ui/serve.py, 8000). Зависимостей нет.
"""

import argparse
import functools
import http.server
from pathlib import Path

UI = Path(__file__).resolve().parent


class Handler(http.server.SimpleHTTPRequestHandler):
    # На Windows mimetypes берёт типы из реестра, и .js там бывает text/plain.
    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map,
                      ".js": "text/javascript", ".html": "text/html; charset=utf-8", ".css": "text/css"}

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def log_message(self, fmt, *args):
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8001)
    ap.add_argument("--lan", action="store_true", help="слушать все интерфейсы (доступ с телефона)")
    args = ap.parse_args()
    host = "0.0.0.0" if args.lan else "127.0.0.1"
    server = http.server.ThreadingHTTPServer((host, args.port), functools.partial(Handler, directory=str(UI)))
    print(f"Страница XIAO: http://localhost:{args.port}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
