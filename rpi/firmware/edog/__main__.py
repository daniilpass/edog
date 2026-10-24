"""Запуск: python -m edog [--config путь] [--mock]"""

import argparse
import asyncio
import logging
from pathlib import Path

import uvicorn

from .api import create_app
from .config import load_config
from .gamepad import Gamepad
from .pca9685 import make_driver
from .robot import Robot

ROOT = Path(__file__).resolve().parent.parent          # firmware/
DEFAULT_CONFIG = ROOT / "config" / "config.toml"


async def serve(cfg, robot) -> None:
    app, broadcast = create_app(robot)
    server = uvicorn.Server(uvicorn.Config(
        app, host=cfg.server["host"], port=cfg.server["port"],
        log_level="warning",
    ))
    tasks = [asyncio.create_task(t) for t in
             (robot.run(), broadcast(), Gamepad(robot, cfg.gamepad).run())]
    try:
        await server.serve()
    finally:
        for t in tasks:
            t.cancel()
        robot.shutdown()


def main() -> None:
    ap = argparse.ArgumentParser(prog="edog")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--mock", action="store_true", help="без PCA9685")
    ap.add_argument("--port", type=int)
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    cfg = load_config(args.config)
    if args.mock:
        cfg.driver["type"] = "mock"
    if args.port:
        cfg.server["port"] = args.port
    driver, error = make_driver(cfg.driver)
    robot = Robot(cfg, driver, error)
    logging.getLogger("edog").info("API: http://<хост>:%s/api, документация /docs", cfg.server["port"])
    asyncio.run(serve(cfg, robot))


if __name__ == "__main__":
    main()
