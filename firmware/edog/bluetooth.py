"""Сопряжение Bluetooth-геймпада через bluetoothctl (BlueZ)."""

import asyncio
import re
import shutil

MAC_RE = re.compile(r"^[0-9A-F]{2}(:[0-9A-F]{2}){5}$")
DEVICE_RE = re.compile(r"^Device ([0-9A-F:]{17}) (.*)$", re.M)


class BluetoothError(RuntimeError):
    pass


async def _ctl(*args: str, timeout: float = 30) -> str:
    if not shutil.which("bluetoothctl"):
        raise BluetoothError("bluetoothctl не найден (это не Raspberry Pi?)")
    proc = await asyncio.create_subprocess_exec(
        "bluetoothctl", *args,
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
    try:
        out, _ = await asyncio.wait_for(proc.communicate(), timeout)
    except asyncio.TimeoutError:
        proc.kill()
        raise BluetoothError(f"bluetoothctl {' '.join(args)}: таймаут")
    return out.decode(errors="replace")


async def _devices(*filters: str) -> list[dict]:
    out = await _ctl("devices", *filters)
    return [{"mac": mac, "name": name} for mac, name in DEVICE_RE.findall(out)]


async def scan(seconds: int = 8) -> list[dict]:
    await _ctl("power", "on")
    await _ctl("--timeout", str(seconds), "scan", "on", timeout=seconds + 10)
    paired = {d["mac"] for d in await _devices("Paired")}
    connected = {d["mac"] for d in await _devices("Connected")}
    return [{**d, "paired": d["mac"] in paired, "connected": d["mac"] in connected}
            for d in await _devices()]


async def pair(mac: str) -> str:
    mac = mac.upper()
    if not MAC_RE.match(mac):
        raise BluetoothError("неверный MAC")
    log = []
    for cmd in (("pair", mac), ("trust", mac), ("connect", mac)):
        log.append(await _ctl(*cmd))
    text = "\n".join(log)
    if "Connection successful" not in text and "already connected" not in text.lower():
        raise BluetoothError(text.strip().splitlines()[-1] if text.strip() else "не удалось подключить")
    return text


async def remove(mac: str) -> str:
    mac = mac.upper()
    if not MAC_RE.match(mac):
        raise BluetoothError("неверный MAC")
    return await _ctl("remove", mac)
