// REST + WebSocket API на API_PORT: те же пути, тела и ответы, что у FastAPI на Raspberry Pi
// (rpi/firmware/edog/api.py), поэтому портал ui/ работает без изменений. CORS открыт для всех.
#pragma once

#include "robot.h"

bool api_begin(Robot &robot);   // false — сервер не запустился
