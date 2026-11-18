// hearth-dsp — stereo image module
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/biquad.hpp>

namespace hdsp::modules {

struct ImageControls {
  float width = 0.50f;    // 0..1 -> 0..200 %
  float mono = 0.00f;     // 0..1 -> 20..500 Hz mono-maker
  float balance = 0.50f;  // 0..1 -> left..right
  bool ms = false;        // monitor mid on L, side on R
};

class Image {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    reset();
  }

  void setControls(const ImageControls& c) noexcept {
    width_ = std::clamp(c.width, 0.0f, 1.0f) * 2.0f;
    const float monoHz = 20.0f * std::pow(25.0f, std::clamp(c.mono, 0.0f, 1.0f));
    if (monoHz > 25.0f) {
      sideHp_.highPass(monoHz, 0.707f, sr_);
    } else {
      sideHp_ = Biquad{};
    }
    const float bal = std::clamp(c.balance, 0.0f, 1.0f);
    if (bal < 0.5f) {
      leftGain_ = 1.0f;
      rightGain_ = bal * 2.0f;
    } else {
      leftGain_ = (1.0f - bal) * 2.0f;
      rightGain_ = 1.0f;
    }
    ms_ = c.ms;
  }

  inline void process(float& l, float& r) noexcept {
    const float mid = 0.5f * (l + r);
    float side = 0.5f * (l - r);
    side = sideHp_.process(side);
    side *= width_;
    if (ms_) {
      l = mid;
      r = side;
      return;
    }
    l = (mid + side) * leftGain_;
    r = (mid - side) * rightGain_;
  }

  void reset() noexcept {
    sideHp_ = Biquad{};
  }

  int latencySamples() const noexcept { return 0; }

private:
  float sr_ = 48000.0f;
  float width_ = 1.0f;
  float leftGain_ = 1.0f;
  float rightGain_ = 1.0f;
  bool ms_ = false;
  Biquad sideHp_;
};

} // namespace hdsp::modules
