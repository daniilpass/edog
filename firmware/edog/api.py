"""REST + WebSocket API и раздача веб-портала."""

import asyncio
import json
import logging
from pathlib import Path
from typing import Literal

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from . import bluetooth

log = logging.getLogger(__name__)

WEB_DIR = Path(__file__).parent / "web"
STATE_HZ = 20


class EnableReq(BaseModel):
    enabled: bool


class ModeReq(BaseModel):
    mode: Literal["stand", "walk", "manual"]


class BodyReq(BaseModel):
    height: float | None = None
    pitch: float | None = None
    roll: float | None = None


class DriveReq(BaseModel):
    vx: float = Field(0, ge=-1, le=1)
    turn: float = Field(0, ge=-1, le=1)


class ServoReq(BaseModel):
    angle: float | None = Field(None, description="угол серво, °")
    joint: float | None = Field(None, description="угол сустава, °")


class CalibrationReq(BaseModel):
    installed: bool | None = None
    zero: float | None = None
    dir: Literal[-1, 1] | None = None
    min_deg: float | None = None
    max_deg: float | None = None


class TestReq(BaseModel):
    action: Literal["center", "ref", "sweep", "stop", "nudge"]
    delta: float = Field(0, ge=-30, le=30, description="для nudge: шаг, °")


class MacReq(BaseModel):
    mac: str


class CalibrateReq(BaseModel):
    joint: float = Field(..., description="угол сустава, который нога физически показывает сейчас")


def create_app(robot, models_dir: Path | None = None) -> tuple[FastAPI, callable]:
    app = FastAPI(title="EDOG", version="0.1")
    clients: set[WebSocket] = set()

    def servo_call(fn, *args, **kwargs):
        try:
            fn(*args, **kwargs)
        except KeyError as e:
            raise HTTPException(404, f"нет такого: {e}")
        except ValueError as e:
            raise HTTPException(400, str(e))
        return robot.state()

    @app.get("/api/config")
    def get_config():
        return robot.describe()

    @app.get("/api/state")
    def get_state():
        return robot.state()

    @app.post("/api/enable")
    def enable(req: EnableReq):
        return servo_call(robot.set_enabled, req.enabled)

    @app.post("/api/estop")
    def estop():
        return servo_call(robot.estop)

    @app.post("/api/mode")
    def mode(req: ModeReq):
        return servo_call(robot.set_mode, req.mode)

    @app.post("/api/pose/{name}")
    def pose(name: str):
        return servo_call(robot.set_pose, name)

    @app.post("/api/body")
    def body(req: BodyReq):
        return servo_call(robot.set_body, req.height, req.pitch, req.roll)

    @app.post("/api/drive")
    def drive(req: DriveReq):
        return servo_call(robot.set_web_drive, req.vx, req.turn)

    @app.put("/api/servos/{sid}")
    def set_servo(sid: str, req: ServoReq):
        if (req.angle is None) == (req.joint is None):
            raise HTTPException(400, "нужно ровно одно из: angle, joint")
        return servo_call(robot.set_servo, sid, req.angle, req.joint)

    @app.post("/api/servos/{sid}/test")
    def test_servo(sid: str, req: TestReq):
        if req.action == "nudge":
            return servo_call(robot.nudge, sid, req.delta)
        return servo_call(robot.test_servo, sid, req.action)

    @app.patch("/api/servos/{sid}/calibration")
    def patch_calibration(sid: str, req: CalibrationReq):
        return servo_call(robot.update_calibration, sid, **req.model_dump())

    @app.post("/api/servos/{sid}/calibrate")
    def calibrate(sid: str, req: CalibrateReq):
        return servo_call(robot.calibrate, sid, req.joint)

    @app.post("/api/calibration/save")
    def save_calibration():
        return {"saved": robot.save_calibration()}

    @app.get("/api/bluetooth/scan")
    async def bt_scan(seconds: int = 8):
        try:
            return {"devices": await bluetooth.scan(max(2, min(seconds, 30)))}
        except bluetooth.BluetoothError as e:
            raise HTTPException(503, str(e))

    @app.post("/api/bluetooth/pair")
    async def bt_pair(req: MacReq):
        try:
            return {"log": await bluetooth.pair(req.mac)}
        except bluetooth.BluetoothError as e:
            raise HTTPException(502, str(e))

    @app.post("/api/bluetooth/remove")
    async def bt_remove(req: MacReq):
        try:
            return {"log": await bluetooth.remove(req.mac)}
        except bluetooth.BluetoothError as e:
            raise HTTPException(502, str(e))

    @app.websocket("/ws")
    async def ws(socket: WebSocket):
        """Сервер шлёт состояние 20 раз в секунду. Клиент может слать
        {"type": "drive", "vx", "turn"} и {"type": "servo", "id", "angle"|"joint"}."""
        await socket.accept()
        clients.add(socket)
        try:
            while True:
                msg = await socket.receive_json()
                try:
                    if msg.get("type") == "drive":
                        robot.set_web_drive(float(msg.get("vx", 0)), float(msg.get("turn", 0)))
                    elif msg.get("type") == "servo":
                        robot.set_servo(msg["id"], msg.get("angle"), msg.get("joint"))
                except (KeyError, ValueError, TypeError) as e:
                    await socket.send_json({"type": "error", "message": str(e)})
        except (WebSocketDisconnect, json.JSONDecodeError):
            pass
        finally:
            clients.discard(socket)

    async def broadcast() -> None:
        while True:
            await asyncio.sleep(1 / STATE_HZ)
            if not clients:
                continue
            text = json.dumps({"type": "state", **robot.state()})
            for socket in list(clients):
                try:
                    await socket.send_text(text)
                except Exception:
                    clients.discard(socket)

    if models_dir and models_dir.is_dir():
        app.mount("/models", StaticFiles(directory=models_dir), name="models")
    app.mount("/", StaticFiles(directory=WEB_DIR, html=True), name="web")
    return app, broadcast
