// hearth-dsp — clip module
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct ClipControls {
  float drive = 0.45f;    // 0..1 -> 0..24 dB
  float ceiling = 0.75f;  // 0..1 -> -12..0 dB
  int mode = 0;           // 0 soft, 1 hard, 2 diode
  float mix = 1.00f;
};

class Clip {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    reset();
  }

  void setControls(const ClipControls& c) noexcept {
    drive_ = dbToGain(clamp01(c.drive) * 24.0f);
    ceiling_ = dbToGain(-12.0f + 12.0f * clamp01(c.ceiling));
    mode_ = std::clamp(c.mode, 0, 2);
    mix_ = clamp01(c.mix);
  }

  inline void process(float& l, float& r) noexcept {
    const float dryL = l, dryR = r;
    const float wetL = shape(l * drive_);
    const float wetR = shape(r * drive_);
    l = dryL + (wetL - dryL) * mix_;
    r = dryR + (wetR - dryR) * mix_;
  }

  void reset() noexcept {}

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  float shape(float x) const noexcept {
    const float ceil = std::max(ceiling_, 1.0e-4f);
    if (mode_ == 1) {
      return std::min(std::max(x, -ceil), ceil);
    }
    if (mode_ == 2) {
      const float bias = 0.15f * ceil;
      return std::tanh((x + bias) / ceil) * ceil - std::tanh(0.15f) * ceil;
    }
    return std::tanh(x / ceil) * ceil;
  }

  float sr_ = 48000.0f;
  float drive_ = 1.0f;
  float ceiling_ = 1.0f;
  int mode_ = 0;
  float mix_ = 1.0f;
};

} // namespace hdsp::modules
