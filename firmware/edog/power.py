"""Лимит тока серво: подключёнными можно отметить столько приводов,
сколько их пиковый ток в сумме влезает в лимит блока питания."""

from dataclasses import dataclass, fields
from math import floor


@dataclass
class PowerCfg:
    limit_a: float = 3.0   # лимит блока питания серво; 0 — без ограничения
    servo_a: float = 0.9   # пиковый ток одного серво

    @classmethod
    def from_dict(cls, data: dict) -> "PowerCfg":
        names = {f.name for f in fields(cls)}
        p = cls(**{k: float(v) for k, v in data.items() if k in names})
        p.validate()
        return p

    def validate(self) -> None:
        if self.limit_a < 0:
            raise ValueError("лимит тока: ≥ 0 (0 — без ограничения)")
        if self.servo_a <= 0:
            raise ValueError("ток серво: > 0")

    def max_servos(self) -> int | None:
        """Сколько серво можно подключить; None — без ограничения."""
        if self.limit_a <= 0:
            return None
        return floor(self.limit_a / self.servo_a + 1e-9)

    def check(self, installed: int) -> None:
        n = self.max_servos()
        if n is not None and installed > n:
            raise ValueError(f"лимит тока {self.limit_a:g} А: подключить можно не больше {n} серво "
                             f"по {self.servo_a:g} А, а нужно {installed} "
                             f"({installed * self.servo_a:.1f} А)")
