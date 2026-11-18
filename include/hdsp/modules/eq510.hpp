// hearth-dsp — 510-style fixed-band EQ
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/biquad.hpp>
#include <hdsp/modules/eq.hpp>

namespace hdsp::modules {

struct Eq510Controls {
  float low = 0.50f;   // 0..1 -> -12..+12 dB at 100 Hz
  float mid = 0.50f;   // 0..1 -> -12..+12 dB at 1.5 kHz
  float high = 0.50f;  // 0..1 -> -12..+12 dB at 10 kHz
  float hpf = 0.00f;   // 0..1 -> 20..500 Hz
};

class Eq510 {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    reset();
  }

  void setControls(const Eq510Controls& c) noexcept {
    const float low = (std::clamp(c.low, 0.0f, 1.0f) - 0.5f) * 24.0f;
    const float mid = (std::clamp(c.mid, 0.0f, 1.0f) - 0.5f) * 24.0f;
    const float high = (std::clamp(c.high, 0.0f, 1.0f) - 0.5f) * 24.0f;
    lowL_.lowShelf(100.0f, low, 0.7f, sr_);
    lowR_.lowShelf(100.0f, low, 0.7f, sr_);
    midL_.peak(1500.0f, mid, 1.0f, sr_);
    midR_.peak(1500.0f, mid, 1.0f, sr_);
    highL_.highShelf(10000.0f, high, 0.7f, sr_);
    highR_.highShelf(10000.0f, high, 0.7f, sr_);
    if (c.hpf > 0.001f) {
      hpL_.highPass(eqHpfHz(c.hpf), 0.707f, sr_);
      hpR_.highPass(eqHpfHz(c.hpf), 0.707f, sr_);
    } else {
      hpL_ = Biquad{};
      hpR_ = Biquad{};
    }
  }

  inline void process(float& l, float& r) noexcept {
    l = hpL_.process(highL_.process(midL_.process(lowL_.process(l))));
    r = hpR_.process(highR_.process(midR_.process(lowR_.process(r))));
  }

  void reset() noexcept {
    lowL_ = lowR_ = midL_ = midR_ = highL_ = highR_ = hpL_ = hpR_ = Biquad{};
  }

  int latencySamples() const noexcept { return 0; }

private:
  float sr_ = 48000.0f;
  Biquad lowL_, lowR_, midL_, midR_, highL_, highR_, hpL_, hpR_;
};

} // namespace hdsp::modules
