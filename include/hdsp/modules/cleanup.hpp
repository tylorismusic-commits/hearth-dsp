// hearth-dsp — lightweight cleanup module
// SPDX-License-Identifier: GPL-3.0-or-later
//
// De-click and de-hum only. This is the lightweight repair representative for
// the rack; the full Repair Suite remains the deep tool.
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/biquad.hpp>

namespace hdsp::modules {

struct CleanupControls {
  float declick = 0.50f;
  float dehum = 0.50f;
  int mains = 0;  // 0 = 50 Hz, 1 = 60 Hz
  float mix = 1.00f;
};

class Cleanup {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    reset();
  }

  void setControls(const CleanupControls& c) noexcept {
    declick_ = clamp01(c.declick);
    dehum_ = clamp01(c.dehum);
    mix_ = clamp01(c.mix);
    const bool mains = c.mains > 0;
    if (mains != lastMains_ || dehum_ != lastDehum_) {
      lastMains_ = mains;
      lastDehum_ = dehum_;
      const float base = mains ? 60.0f : 50.0f;
      const float gainDb = -40.0f * dehum_;
      for (int i = 0; i < 4; ++i) {
        notchL_[i].peak(base * (i + 1), gainDb, 30.0f, sr_);
        notchR_[i].peak(base * (i + 1), gainDb, 30.0f, sr_);
      }
    }
  }

  inline void process(float& l, float& r) noexcept {
    const float dryL = l, dryR = r;
    if (declick_ > 0.001f) {
      const float maxStep = 0.5f * (1.0f - declick_) + 0.005f;
      const float dl = l - prevL_;
      const float dr = r - prevR_;
      if (std::fabs(dl) > maxStep) l = prevL_ + std::copysign(maxStep, dl);
      if (std::fabs(dr) > maxStep) r = prevR_ + std::copysign(maxStep, dr);
    }
    prevL_ = l;
    prevR_ = r;
    if (dehum_ > 0.001f) {
      for (int i = 0; i < 4; ++i) {
        l = notchL_[i].process(l);
        r = notchR_[i].process(r);
      }
    }
    l = dryL + (l - dryL) * mix_;
    r = dryR + (r - dryR) * mix_;
  }

  void reset() noexcept {
    prevL_ = prevR_ = 0.0f;
    lastMains_ = -1;
    lastDehum_ = -1.0f;
    for (int i = 0; i < 4; ++i) {
      notchL_[i] = Biquad{};
      notchR_[i] = Biquad{};
    }
  }

  int latencySamples() const noexcept { return 0; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  float sr_ = 48000.0f;
  float declick_ = 0.5f;
  float dehum_ = 0.5f;
  float mix_ = 1.0f;
  float prevL_ = 0.0f;
  float prevR_ = 0.0f;
  int lastMains_ = -1;
  float lastDehum_ = -1.0f;
  Biquad notchL_[4];
  Biquad notchR_[4];
};

} // namespace hdsp::modules
