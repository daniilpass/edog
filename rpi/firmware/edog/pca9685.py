"""Драйвер PCA9685 (I²C) и заглушка для работы без железа."""

import logging
import time

log = logging.getLogger(__name__)

MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06
ALL_LED_ON_L = 0xFA

SLEEP = 0x10
AUTO_INCREMENT = 0x20
RESTART = 0x80
OUTDRV = 0x04
FULL_OFF = 0x10  # бит 4 в LEDn_OFF_H


class MockDriver:
    kind = "mock"

    def __init__(self):
        self.pulses: dict[int, float | None] = {}

    def set_pulse(self, channel: int, us: float) -> None:
        self.pulses[channel] = us

    def off(self, channel: int) -> None:
        self.pulses[channel] = None

    def all_off(self) -> None:
        for ch in self.pulses:
            self.pulses[ch] = None

    def close(self) -> None:
        pass


class PCA9685Driver:
    kind = "pca9685"

    def __init__(self, bus: int, address: int, frequency_hz: float, oscillator_hz: float):
        from smbus2 import SMBus

        self.bus = SMBus(bus)
        self.address = address
        self._ticks: dict[int, int] = {}

        prescale = round(oscillator_hz / (4096 * frequency_hz)) - 1
        self.frequency_hz = oscillator_hz / (4096 * (prescale + 1))
        self._write(MODE1, SLEEP)
        self._write(PRESCALE, prescale)
        self._write(MODE2, OUTDRV)
        self._write(MODE1, AUTO_INCREMENT)
        time.sleep(0.005)
        self._write(MODE1, AUTO_INCREMENT | RESTART)
        self.all_off()
        log.info("PCA9685 0x%02x на шине %d, %.1f Гц", address, bus, self.frequency_hz)

    def _write(self, reg: int, value: int) -> None:
        self.bus.write_byte_data(self.address, reg, value)

    def set_pulse(self, channel: int, us: float) -> None:
        ticks = max(0, min(4095, round(us * self.frequency_hz * 4096 / 1_000_000)))
        if self._ticks.get(channel) == ticks:
            return
        self.bus.write_i2c_block_data(self.address, LED0_ON_L + 4 * channel,
                                      [0, 0, ticks & 0xFF, ticks >> 8])
        self._ticks[channel] = ticks

    def off(self, channel: int) -> None:
        self.bus.write_i2c_block_data(self.address, LED0_ON_L + 4 * channel, [0, 0, 0, FULL_OFF])
        self._ticks.pop(channel, None)

    def all_off(self) -> None:
        self.bus.write_i2c_block_data(self.address, ALL_LED_ON_L, [0, 0, 0, FULL_OFF])
        self._ticks.clear()

    def close(self) -> None:
        try:
            self.all_off()
        finally:
            self.bus.close()


def make_driver(cfg: dict):
    """Возвращает (драйвер, ошибка). При ошибке I²C — заглушка, чтобы портал всё равно работал."""
    if cfg["type"] == "mock":
        return MockDriver(), None
    try:
        return PCA9685Driver(
            bus=cfg["i2c_bus"],
            address=cfg["address"],
            frequency_hz=cfg["frequency_hz"],
            oscillator_hz=cfg["oscillator_hz"],
        ), None
    except Exception as e:  # нет I²C, нет платы, нет smbus2
        log.error("PCA9685 недоступен, работаю на заглушке: %s", e)
        return MockDriver(), f"PCA9685: {e}"
