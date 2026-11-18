// hearth-dsp — bitcrusher module
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/biquad.hpp>

namespace hdsp::modules {

struct CrushControls {
  float bits = 1.00f;  // 0..1 -> 1..16 bits
  float rate = 1.00f;  // 0..1 -> full rate..~240 Hz
  float tone = 1.00f;  // 0..1 -> 1..20 kHz
  float mix = 1.00f;
};

class Crush {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    reset();
  }

  void setControls(const CrushControls& c) noexcept {
    bits_ = 1.0f + 15.0f * clamp01(c.bits);
    const float rate = clamp01(c.rate);
    holdSamples_ = 1 + static_cast<int>(std::lround((1.0f - rate) * 200.0f));
    const float toneHz = 1000.0f * std::pow(20.0f, clamp01(c.tone));
    toneL_.lowPass(toneHz, 0.707f, sr_);
    toneR_.lowPass(toneHz, 0.707f, sr_);
    mix_ = clamp01(c.mix);
  }

  inline void process(float& l, float& r) noexcept {
    if (--hold_ <= 0) {
      hold_ = holdSamples_;
      heldL_ = l;
      heldR_ = r;
    }
    const float levels = std::pow(2.0f, bits_ - 1.0f);
    const float ql = std::round(heldL_ * levels) / levels;
    const float qr = std::round(heldR_ * levels) / levels;
    const float wetL = toneL_.process(ql);
    const float wetR = toneR_.process(qr);
    l = l + (wetL - l) * mix_;
    r = r + (wetR - r) * mix_;
  }

  void reset() noexcept {
    hold_ = 0;
    heldL_ = heldR_ = 0.0f;
    toneL_ = Biquad{};
    toneR_ = Biquad{};
  }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  float sr_ = 48000.0f;
  float bits_ = 16.0f;
  int holdSamples_ = 1;
  int hold_ = 0;
  float heldL_ = 0.0f;
  float heldR_ = 0.0f;
  float mix_ = 1.0f;
  Biquad toneL_;
  Biquad toneR_;
};

} // namespace hdsp::modules
