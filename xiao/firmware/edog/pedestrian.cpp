#include "pedestrian.h"

#include <Arduino.h>

#include <algorithm>
#include <cmath>

#include "pedestrian_model.h"

Pedestrian::Pedestrian(float score_thr, float nms_thr) {
  model_ = new dl::Model((const char *)PEDESTRIAN_MODEL, fbs::MODEL_LOCATION_IN_FLASH_RODATA);
  model_->minimize();
  pre_ = new dl::image::ImagePreprocessor(model_, {0, 0, 0}, {1, 1, 1});
  post_ = new dl::detect::PicoPostprocessor(model_, pre_, score_thr, nms_thr, 10,
                                            {{8, 8, 4, 4}, {16, 16, 8, 8}, {32, 32, 16, 16}});
}

std::list<dl::detect::result_t> &Pedestrian::run(const uint8_t *rgb, int w, int h) {
  dl::TensorBase *in = model_->get_input();
  int ih = in->shape[1], iw = in->shape[2];
  int exp = in->exponent;
  // Как ImagePreprocessor с mean 0, std 1: q = пиксель · 2^(−exp), растяжение без полей, ближайший пиксель.
  if (exp != lut_exp_) {
    for (int v = 0; v < 256; v++) lut_[v] = (int8_t)std::min(127, (int)std::lround(std::ldexp((float)v, -exp)));
    lut_exp_ = exp;
  }
  int8_t *dst = (int8_t *)in->data;
  for (int y = 0; y < ih; y++) {
    const uint8_t *row = rgb + (y * h / ih) * w * 3;
    for (int x = 0; x < iw; x++) {
      const uint8_t *p = row + (x * w / iw) * 3;
      *dst++ = lut_[p[0]];
      *dst++ = lut_[p[1]];
      *dst++ = lut_[p[2]];
    }
  }
  // Масштаб рамок постпроцессор берёт у предобработки (get_resize_scale_*), а та считает его в transform(),
  // которую мы не вызываем. Пишем масштаб сами: ImageTransformer — первое поле ImagePreprocessor, масштабы —
  // float по смещениям 24/28 (кадр → вход) и 32/36 (вход → кадр), так читают get_scale_x/y в ESP-DL 3.3.11.
  float *scale = (float *)((uint8_t *)pre_ + 24);
  scale[0] = (float)iw / w;
  scale[1] = (float)ih / h;
  scale[2] = (float)w / iw;
  scale[3] = (float)h / ih;
  if (!scale_checked_) {
    scale_checked_ = true;
    if (pre_->get_resize_scale_x(true) != scale[2] || pre_->get_resize_scale_y(true) != scale[3])
      Serial.println("Зрение: ESP-DL сменил раскладку ImageTransformer — рамки людей будут неверны (pedestrian.cpp)");
  }
  model_->run();
  post_->clear_result();
  post_->postprocess();
  return post_->get_result(w, h);
}
