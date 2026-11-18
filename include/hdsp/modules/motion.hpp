// hearth-dsp — tremolo / auto-pan / rotary module
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

namespace hdsp::modules {

struct MotionControls {
  float rate = 0.50f;   // 0..1 -> 0.05..10 Hz
  float depth = 0.50f;  // 0..1
  float width = 1.00f;  // 0..1
  int mode = 0;         // 0 tremolo, 1 auto-pan, 2 rotary
  float mix = 1.00f;
};

class Motion {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    reset();
  }

  void setControls(const MotionControls& c) noexcept {
    rateHz_ = 0.05f * std::pow(200.0f, clamp01(c.rate));
    depth_ = clamp01(c.depth);
    width_ = clamp01(c.width);
    mode_ = std::clamp(c.mode, 0, 2);
    mix_ = clamp01(c.mix);
  }

  inline void process(float& l, float& r) noexcept {
    const float dryL = l, dryR = r;
    phase_ += 2.0f * 3.14159265f * rateHz_ / sr_;
    if (phase_ >= 2.0f * 3.14159265f) phase_ -= 2.0f * 3.14159265f;
    const float s = std::sin(phase_);
    const float c = std::cos(phase_);

    float wetL = l, wetR = r;
    if (mode_ == 0) {
      const float gain = 1.0f - depth_ * 0.5f * (1.0f + s);
      wetL *= gain;
      wetR *= gain;
    } else {
      const float leftGain = 1.0f - width_ * 0.5f * (1.0f + s);
      const float rightGain = 1.0f - width_ * 0.5f * (1.0f - s);
      wetL *= leftGain;
      wetR *= rightGain;
      if (mode_ == 2) {
        const float amp = 1.0f - depth_ * 0.5f * (1.0f + c);
        wetL *= amp;
        wetR *= amp;
      }
    }
    l = dryL + (wetL - dryL) * mix_;
    r = dryR + (wetR - dryR) * mix_;
  }

  void reset() noexcept { phase_ = 0.0f; }

  int latencySamples() const noexcept { return 0; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  float sr_ = 48000.0f;
  float rateHz_ = 0.5f;
  float depth_ = 0.5f;
  float width_ = 1.0f;
  float mix_ = 1.0f;
  int mode_ = 0;
  float phase_ = 0.0f;
};

} // namespace hdsp::modules
