"""Демо EDOG из консоли — то же, что кнопка «Демо» в портале: ноги по кругу шагают в воздухе.

    python tools/dance.py                         # собака 192.168.50.129:8080, по 3 с, 3 круга
    python tools/dance.py --dog 127.0.0.1:8080    # локальный сервис с --mock
    python tools/dance.py --seconds 5 --laps 1

Демо идёт на собаке (firmware/edog/demo.py), сколько ног сразу — по лимиту тока:
2–3 серво — по одной ноге, 4–7 — диагональными парами, от 8 или без лимита — все ноги.
Ctrl+C — остановить: сигнал с серво снимается.
"""

import argparse
import json
import sys
import time
import urllib.error
import urllib.request


def call(api: str, method: str, path: str, body=None) -> dict:
    req = urllib.request.Request(
        api + path, method=method,
        data=json.dumps(body).encode() if body is not None else None,
        headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        raise SystemExit(f"{method} {path}: {e.code} {e.read().decode(errors='replace')}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--dog", default="192.168.50.129:8080", help="адрес API собаки")
    ap.add_argument("--seconds", type=float, default=3, help="сколько шагает каждая нога, с")
    ap.add_argument("--laps", type=int, default=3, help="кругов по ногам")
    args = ap.parse_args()
    api = f"http://{args.dog}/api"

    st = call(api, "POST", "/demo", {"action": "start", "seconds": args.seconds, "laps": args.laps})
    print(f"Демо: {st['demo']['title']}, по {args.seconds:g} с, кругов {args.laps}, лимит {st['power']['limit_a']:g} А")
    last = None
    try:
        while (d := call(api, "GET", "/state")["demo"]):
            line = f"{d['number']}/{d['numbers']}: шагает {', '.join(d['legs'])}"
            if line != last:
                print(" ", line)
                last = line
            time.sleep(0.3)
    except KeyboardInterrupt:
        call(api, "POST", "/demo", {"action": "stop"})
        print("\nостановлено")
    print("Готово: сигнал с серво снят.")


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
