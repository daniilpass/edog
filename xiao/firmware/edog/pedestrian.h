// Поиск людей на кадре: модель ESP-DL pedestrian_detect (Pico, int8, вход 224×224), встроена в прошивку.
// По мотивам espressif/pedestrian_detect 0.3.2 (MIT): модель — массив во флеше, без menuconfig и SD.
// Вход модели заполняется здесь (RGB888 → 224×224 → int8), а не ImagePreprocessor ESP-DL: тот тянет
// преобразования всех форматов пикселей (~650 КБ кода) и не влезает в слот прошивки.
#pragma once

#include <list>

#include "dl_detect_pico_postprocessor.hpp"

class Pedestrian {
 public:
  explicit Pedestrian(float score_thr = 0.5f, float nms_thr = 0.5f);
  // Рамки людей на кадре RGB888 w×h: box — [x1, y1, x2, y2] в пикселях этого кадра.
  std::list<dl::detect::result_t> &run(const uint8_t *rgb, int w, int h);

 private:
  dl::Model *model_;
  dl::image::ImagePreprocessor *pre_;   // только для масштаба рамок в постпроцессоре
  dl::detect::PicoPostprocessor *post_;
  int8_t lut_[256];     // пиксель → вход модели при текущей экспоненте
  int lut_exp_ = 1000;
  bool scale_checked_ = false;
};
