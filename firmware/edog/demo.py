"""Демо: ноги по кругу шагают в воздухе, по несколько секунд каждая группа.
Размер группы подбирается по лимиту тока: сигнал получают только серво шагающих ног.

Шаг — та же траектория лапы, что при ходьбе (robot.step_offset). Каждая группа плавно выходит
из монтажного положения (бедро 0°, колено 90°) и возвращается в него, поэтому на стыке
сигнал можно переключать на следующую группу.
"""

from collections.abc import Callable

RAMP_S = 0.4    # выход из монтажного положения в шаг и обратно, с

ORDER = ("FL", "FR", "RR", "RL")    # по кругу
DIAG_A, DIAG_B = ("FL", "RR"), ("FR", "RL")

PROGRAMS = {
    "solo": {"title": "по одной ноге", "groups": [(leg,) for leg in ORDER]},   # 2–3 серво
    "pairs": {"title": "диагональными парами", "groups": [DIAG_A, DIAG_B]},     # 4–7 серво
    "all": {"title": "все ноги", "groups": [ORDER]},                            # от 8
}


def pick_program(max_servos: int | None) -> str:
    """Программа по лимиту тока: None — без ограничения."""
    if max_servos is None or max_servos >= 8:
        return "all"
    if max_servos >= 4:
        return "pairs"
    if max_servos >= 2:
        return "solo"
    raise ValueError(f"демо: лимит тока позволяет {max_servos} серво, а нужно хотя бы 2 (одна нога)")


def envelope(t: float, length: float) -> float:
    """0 → 1 за RAMP_S в начале, 1 → 0 за RAMP_S в конце (плавно)."""
    x = min(t, length - t, RAMP_S) / RAMP_S
    if x <= 0:
        return 0.0
    return x * x * (3 - 2 * x)


class Demo:
    def __init__(self, program: str, seconds: float, laps: int, ref: dict[str, float],
                 walk: Callable[[str, float], tuple[float, float]]):
        """walk(нога, время шага, с) → углы суставов (hip, knee) шагающей ноги."""
        if program not in PROGRAMS:
            raise ValueError(f"демо: программы {tuple(PROGRAMS)}")
        if seconds < 2 * RAMP_S:
            raise ValueError(f"демо: на ногу не меньше {2 * RAMP_S:g} с")
        groups = PROGRAMS[program]["groups"]
        # Одна группа на всех — шагает без пауз столько же, сколько длился бы круг по ногам.
        self.segments = ([(groups[0], seconds * len(ORDER) * laps)] if len(groups) == 1
                         else [(g, seconds) for _ in range(laps) for g in groups])
        self.program = program
        self.ref = ref
        self.walk = walk
        self.index = 0
        self.t = 0.0
        self.total = sum(length for _, length in self.segments)
        self.done = 0.0

    @property
    def finished(self) -> bool:
        return self.index >= len(self.segments)

    @property
    def legs(self) -> tuple[str, ...]:
        return () if self.finished else self.segments[self.index][0]

    def tick(self, dt: float) -> dict[str, tuple[float, float]]:
        """Сдвинуть время на dt секунд; углы суставов (hip, knee) шагающих ног."""
        if self.finished:
            return {}
        legs, length = self.segments[self.index]
        self.t += dt
        self.done += dt
        if self.t >= length:
            self.index += 1
            self.t = 0.0
            return {leg: (self.ref["hip"], self.ref["knee"]) for leg in legs}
        e = envelope(self.t, length)
        result = {}
        for leg in legs:
            hip, knee = self.walk(leg, self.t)
            result[leg] = (self.ref["hip"] + e * (hip - self.ref["hip"]),
                           self.ref["knee"] + e * (knee - self.ref["knee"]))
        return result

    def state(self) -> dict:
        return {
            "program": self.program,
            "title": PROGRAMS[self.program]["title"],
            "legs": list(self.legs),
            "number": min(self.index + 1, len(self.segments)),
            "numbers": len(self.segments),
            "progress": round(min(self.done / self.total, 1.0), 3),
        }
