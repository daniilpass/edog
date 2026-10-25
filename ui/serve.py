"""Локальный сервер портала EDOG: отдаёт ui/ и 3D-модели из cad/web/ по адресу /models/.

Плата собаки (Raspberry Pi, XIAO) отдаёт только API, портал запускается на ПК:
    python ui/serve.py              → http://localhost:8000/
    python ui/serve.py --lan        → доступен и с телефона в той же сети
    python ui/serve.py --port 9000

Адрес собаки задаётся в шапке портала или ссылкой: http://localhost:8000/?dog=192.168.50.129:8080
Запись (кнопка «Записывать» у камеры) сохраняется сюда, в recordings/<время>/: кадры NNNNN.jpg и log.jsonl —
заголовки каждого кадра и состояние собаки (патруль, зрение, ходьба). На плате памяти для этого нет.
Зависимостей нет — только стандартная библиотека.
"""

import argparse
import http.server
import re
import socket
import time
import urllib.parse
from pathlib import Path

UI = Path(__file__).resolve().parent
MODELS = UI.parent / "cad" / "web"
RECORDINGS = UI.parent / "recordings"
REC_FILE = re.compile(r"^/rec/(\d{8}-\d{6})/(\d{5}\.jpg|log\.jsonl)$")


class Handler(http.server.SimpleHTTPRequestHandler):
    # На Windows mimetypes берёт типы из реестра, и .js там бывает text/plain — модули тогда не грузятся.
    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map,
                      ".js": "text/javascript", ".json": "application/json", ".stl": "model/stl",
                      ".html": "text/html; charset=utf-8", ".css": "text/css"}

    def translate_path(self, path):
        path = urllib.parse.unquote(path.split("?", 1)[0].split("#", 1)[0])
        base, rel = (MODELS, path[len("/models/"):]) if path.startswith("/models/") else (UI, path.lstrip("/"))
        full = (base / rel).resolve()
        if full != base and base not in full.parents:  # не выпускаем за пределы папки
            return str(base / "__forbidden__")
        return str(full)

    # Запись: POST /rec/start → имя папки; PUT /rec/<имя>/NNNNN.jpg — кадр; POST /rec/<имя>/log.jsonl — дописать строки.
    def do_POST(self):
        if self.path == "/rec/start":
            name = time.strftime("%Y%m%d-%H%M%S")
            (RECORDINGS / name).mkdir(parents=True, exist_ok=True)
            print(f"Запись: {RECORDINGS / name}")
            return self.reply(200, name)
        self.save("ab")

    def do_PUT(self):
        self.save("wb")

    def save(self, mode):
        m = REC_FILE.match(self.path)
        folder = RECORDINGS / m.group(1) if m else None
        if not m or not folder.is_dir():
            return self.reply(404, "no such recording")
        data = self.rfile.read(int(self.headers.get("Content-Length", 0)))
        with open(folder / m.group(2), mode) as f:
            f.write(data)
        self.reply(200, "ok")

    def reply(self, code, text):
        body = text.encode()
        self.send_response(code)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")  # после правок не нужен Ctrl+F5
        super().end_headers()

    def log_message(self, fmt, *args):
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--lan", action="store_true", help="слушать все интерфейсы (доступ с телефона)")
    args = ap.parse_args()
    host = "0.0.0.0" if args.lan else "127.0.0.1"
    server = http.server.ThreadingHTTPServer((host, args.port), Handler)
    print(f"Портал EDOG: http://localhost:{args.port}/")
    if args.lan:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("192.168.0.1", 1))
                print(f"С телефона:  http://{s.getsockname()[0]}:{args.port}/")
        except OSError:
            pass
    if not (MODELS / "manifest.json").exists():
        print("Внимание: нет cad/web/manifest.json — собака будет нарисована примитивами")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
