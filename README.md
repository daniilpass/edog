# Edog

Робопёс на 3D-печати с управлением с Raspberry Pi.

## Структура проекта

```
cad/
  origin/        — оригинальные STL автора базовой модели (для справки, не редактируем)
    geometry_fix/  — исправленная геометрия оригинала (fixed_motor_bloc.stl)
  freecad/       — рабочие исходники FreeCAD (.FCStd): корпус, платформа под RPi, боковые рамы, доработанные детали
  custom/        — финальные STL для печати (описание — cad/custom/README.md)
    tests/         — тестовые вставки для проверки посадок
  web/           — облегчённые модели и manifest.json для 3D в веб-портале (выгружает export_web_models.py)
firmware/      — сервис для Raspberry Pi: серво, геймпад, API, веб-портал (см. firmware/README.md)
docs/          — обсуждение идей, решения, заметки по железу
```

## О проекте и железе

См. [docs/project.md](docs/project.md) и [docs/hardware.md](docs/hardware.md).

## CAD

Исходники в `cad/freecad/` версионируются буквой в имени (`EDOG_F_…` → `EDOG_T_…`), последняя буква — актуальная сборка. Сейчас это `EDOG_T_Front_Mark.FCStd`: большой корпус со стрелкой «перёд» на полу. Из неё же `cad/web/export_web_models.py` выгружает модели для портала.

`cad/freecad/body_magnets.FCStd` — компактный вариант: корпус и площадка под RPi на магнитах.

STL для печати лежат в `cad/custom/`, их список и параметры печати — в [cad/custom/README.md](cad/custom/README.md).
