// hearth-dsp — compressor module family
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A five-model 500-series compressor used by Suite Rack and future plugins.
// The model classes live in hdsp/modules/comp/ and are shared with the
// standalone Comp Suite models.
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/db.hpp>

#include "comp/controls.hpp"
#include "comp/fet.hpp"
#include "comp/optical.hpp"
#include "comp/vca.hpp"

namespace hdsp::modules {

struct CompControls {
  int model = 0;  // 0 76Blue, 1 76Black, 2 2A, 3 3A, 4 VCA, 5 Hearth, 6 Glue
  float threshold = 0.50f;
  float ratio = 0.50f;
  float attack = 0.25f;
  float release = 0.35f;
  float makeup = 0.50f;
  float mix = 1.00f;
  float scHpf = 0.05f;
};

class Comp {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    fetBlue_.prepare(sr_, comp::FetLimiter::Variant::Blue);
    fetBlack_.prepare(sr_, comp::FetLimiter::Variant::Black);
    opto2A_.prepare(sr_, comp::OptoLeveler::Variant::LA2A);
    opto3A_.prepare(sr_, comp::OptoLeveler::Variant::LA3A);
    vca_.prepare(sr_);
    reset();
  }

  void setControls(const CompControls& c) noexcept {
    if (c.model != lastModel_) {
      reset();
      lastModel_ = c.model;
    }
    comp::Controls cc{};
    const float thr = std::clamp(c.threshold, 0.0f, 1.0f);
    const float t = -40.0f + 40.0f * thr;
    const float inputDb = -18.0f + 0.45f * (t + 20.0f);
    cc.input = std::pow(std::clamp((inputDb + 80.0f) / 103.0f, 0.0f, 1.0f), 2.0f);
    cc.threshold = std::clamp((-t - 8.0f) / 32.0f, 0.0f, 1.0f);
    cc.ratio = std::clamp(c.ratio * 8.0f, 0.0f, 8.0f);
    cc.attack = c.attack;
    cc.release = c.release;
    cc.makeup = c.makeup;
    cc.output = 0.5f;
    cc.sidechainHpf = c.scHpf;
    cc.mix = 1.0f;
    cc.link = 1.0f;
    mix_ = std::clamp(c.mix, 0.0f, 1.0f);

    switch (c.model) {
      case 1:
        model_ = 1;
        fetBlack_.setControls(cc);
        break;
      case 2:
        model_ = 2;
        cc.threshold = thr;
        opto2A_.setControls(cc);
        break;
      case 3:
        model_ = 3;
        cc.threshold = thr;
        opto3A_.setControls(cc);
        break;
      case 4:
        model_ = 4;
        cc.threshold = thr;
        vca_.setControls(cc);
        break;
      case 5:
        model_ = 5;
        cc.threshold = thr;
        vca_.setControls(cc);
        break;
      case 6:
        model_ = 6;
        cc.threshold = thr;
        vca_.setControls(cc);
        break;
      default:
        model_ = 0;
        fetBlue_.setControls(cc);
        break;
    }
    trim_ = model_ >= 4 ? 1.0f : hdsp::dbToGain(comp::kModelTrimDb[model_]);
  }

  inline float process(float& l, float& r) noexcept {
    const float dryL = l, dryR = r;
    float gr = 0.0f;
    switch (model_) {
      case 0: gr = fetBlue_.process(l, r); break;
      case 1: gr = fetBlack_.process(l, r); break;
      case 2: gr = opto2A_.process(l, r); break;
      case 3: gr = opto3A_.process(l, r); break;
      default: gr = vca_.process(l, r); break;
    }
    if (model_ == 5) {
      l = softSat(l, 1.15f);
      r = softSat(r, 1.15f);
    } else if (model_ == 6) {
      l = softSat(l, 1.08f);
      r = softSat(r, 1.08f);
    }
    l = dryL + (l * trim_ - dryL) * mix_;
    r = dryR + (r * trim_ - dryR) * mix_;
    return gr;
  }

  void reset() noexcept {
    lastModel_ = -1;
    fetBlue_.reset();
    fetBlack_.reset();
    opto2A_.reset();
    opto3A_.reset();
    vca_.reset();
  }

private:
  static float softSat(float x, float drive) noexcept {
    constexpr float bias = 0.02f;
    const float tb = std::tanh(bias);
    return (std::tanh(x * drive + bias) - tb) / drive;
  }

  float sr_ = 48000.0f;
  int lastModel_ = -1;
  int model_ = 0;
  float trim_ = 1.0f;
  float mix_ = 1.0f;
  comp::FetLimiter fetBlue_;
  comp::FetLimiter fetBlack_;
  comp::OptoLeveler opto2A_;
  comp::OptoLeveler opto3A_;
  comp::BusVca vca_;
};

} // namespace hdsp::modules
