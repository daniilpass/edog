"""Сопряжение Bluetooth-геймпада через bluetoothctl (BlueZ)."""

import asyncio
import re
import shutil

MAC_RE = re.compile(r"^[0-9A-F]{2}(:[0-9A-F]{2}){5}$")
DEVICE_RE = re.compile(r"^Device ([0-9A-F:]{17}) (.*)$", re.M)
INFO_RE = re.compile(r"^\s+(\w+): (.*)$", re.M)
ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]|\r")


class BluetoothError(RuntimeError):
    pass


def _check_mac(mac: str) -> str:
    mac = mac.upper()
    if not MAC_RE.match(mac):
        raise BluetoothError("неверный MAC")
    return mac


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
    return ANSI_RE.sub("", out.decode(errors="replace"))


async def _session(steps: list[tuple[str, tuple[str, ...], float]]) -> str:
    """Интерактивная сессия bluetoothctl: команда → ждём одну из строк-ответов.

    Нужна для сопряжения: агент (подтверждение без PIN) живёт, пока жива сессия.
    Разовый `bluetoothctl --agent ... pair` завершается раньше, чем геймпад
    (например, Xbox по BLE) присылает запрос подтверждения → AuthenticationFailed.
    """
    if not shutil.which("bluetoothctl"):
        raise BluetoothError("bluetoothctl не найден (это не Raspberry Pi?)")
    proc = await asyncio.create_subprocess_exec(
        "bluetoothctl", stdin=asyncio.subprocess.PIPE,
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
    log: list[str] = []
    try:
        for cmd, expect, timeout in steps:
            proc.stdin.write(f"{cmd}\n".encode())
            await proc.stdin.drain()
            loop = asyncio.get_running_loop()
            deadline = loop.time() + timeout
            while expect:
                left = deadline - loop.time()
                if left <= 0:
                    break
                try:
                    raw = await asyncio.wait_for(proc.stdout.readline(), left)
                except asyncio.TimeoutError:
                    break
                if not raw:
                    break
                line = ANSI_RE.sub("", raw.decode(errors="replace")).strip()
                if line:
                    log.append(line)
                if any(e in line for e in expect):
                    break
    finally:
        if proc.returncode is None:
            proc.stdin.write(b"quit\n")
            try:
                await asyncio.wait_for(proc.wait(), 3)
            except asyncio.TimeoutError:
                proc.kill()
    return "\n".join(log)


async def _devices(*filters: str) -> list[dict]:
    out = await _ctl("devices", *filters)
    return [{"mac": mac, "name": name} for mac, name in DEVICE_RE.findall(out)]


async def _info(mac: str) -> dict[str, str]:
    return dict(INFO_RE.findall(await _ctl("info", mac)))


async def scan(seconds: int = 8) -> list[dict]:
    await _ctl("power", "on")
    await _ctl("--timeout", str(seconds), "scan", "on", timeout=seconds + 10)
    paired = {d["mac"] for d in await _devices("Paired")}
    connected = {d["mac"] for d in await _devices("Connected")}
    return [{**d, "paired": d["mac"] in paired, "connected": d["mac"] in connected}
            for d in await _devices()]


async def pair(mac: str) -> str:
    """Сопряжение + доверие + подключение. Успех проверяется по `bluetoothctl info`."""
    mac = _check_mac(mac)
    if mac not in {d["mac"] for d in await _devices()}:
        await scan(8)  # устройство должно быть найдено поиском, иначе pair его не знает
    log = []
    if (await _info(mac)).get("Paired") != "yes":
        log.append(await _session([
            ("agent NoInputNoOutput", ("Agent registered", "Failed to register"), 5),
            ("default-agent", ("Default agent request successful", "No agent"), 5),
            (f"pair {mac}", ("Pairing successful", "Failed to pair", "not available"), 30),
        ]))
    log.append(await _ctl("trust", mac))
    log.append(await _ctl("connect", mac, timeout=20))
    text = "\n".join(log)

    info = await _info(mac)
    if info.get("Paired") != "yes":
        failed = [l for l in text.splitlines() if "Failed" in l or "Error" in l]
        hint = failed[-1].strip() if failed else "нет ответа"
        raise BluetoothError(f"сопряжение не удалось ({hint}). Переведите геймпад в режим сопряжения и повторите")
    if info.get("Connected") != "yes":
        raise BluetoothError("сопряжено, но не подключилось — нажмите кнопку питания на геймпаде")
    return text


async def remove(mac: str) -> str:
    return await _ctl("remove", _check_mac(mac))
