# О проекте

Edog — 3D-печатный квадропод. База — открытый проект, который сейчас заброшен автором.

## Источники (оригинал)
- Hackaday: https://hackaday.io/project/177204-edog-mini-3d-printed-quadruped-robot
- Инструкция (Part 1): https://medium.com/@solidmakers/build-your-own-quadruped-robot-with-edog-part-1-e5e2f3a08729

## Чем наша версия отличается от оригинала
- В оригинале Raspberry Pi Zero WH; в нашей сборке — Raspberry Pi 3B+ и адаптированная площадка (`cad/freecad/body_magnets.FCStd`, STL — `cad/custom/chassis_small.stl` и `cad/custom/platform_small.stl`).
- Исправлена геометрия блока привода (motor bloc): `cad/origin/geometry_fix/fixed_motor_bloc.stl`.
- Femur: посадочное место под крыло серво расширено под MG90S: `cad/custom/Femur_hub7.stl`. Подробности — [cad/custom/README.md](../cad/custom/README.md).
- Серво: в оригинале SG90/VOTIK 7455, у нас — MG90S

## Текущий прототип

- Четыре ноги, по два MG90S на ногу: всего 8 приводов.
- Вычислитель — Raspberry Pi 3B+; сервоприводами управляет PCA9685 по I²C.
- На первом этапе рассматривается управление заданными углами сервоприводов без обратной связи по положению суставов.
- План подключения указан в [hardware.md](hardware.md).

## Планы развития
- LiPo 2S вместо проводного питания
- Альтернативный ходовой режим: 4 колеса на лапах, все ведущие
- Камера + автономная навигация

Подробности по железу — [hardware.md](hardware.md).
