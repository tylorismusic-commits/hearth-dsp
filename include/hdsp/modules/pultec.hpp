// hearth-dsp — Pultec-style EQ
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A compact low/high boost-and-attenuate EQ. The boost and attenuation shelves
// overlap at the same corner, so the classic "boost and cut at once" contour
// is available without a full passive network.
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/biquad.hpp>

namespace hdsp::modules {

struct PultecControls {
  float lowBoost = 0.00f;   // 0..1 -> 0..12 dB at 100 Hz
  float lowAtten = 0.00f;   // 0..1 -> 0..12 dB cut at 100 Hz
  float highBoost = 0.00f;  // 0..1 -> 0..12 dB at 10 kHz
  float highAtten = 0.00f;  // 0..1 -> 0..12 dB cut at 10 kHz
  float bandwidth = 0.50f;  // 0..1 -> shelf Q 0.5..2.0
};

class Pultec {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    reset();
  }

  void setControls(const PultecControls& c) noexcept {
    const float lowBoostDb = 12.0f * std::clamp(c.lowBoost, 0.0f, 1.0f);
    const float lowAttenDb = 12.0f * std::clamp(c.lowAtten, 0.0f, 1.0f);
    const float highBoostDb = 12.0f * std::clamp(c.highBoost, 0.0f, 1.0f);
    const float highAttenDb = 12.0f * std::clamp(c.highAtten, 0.0f, 1.0f);
    const float q = 0.5f + 1.5f * std::clamp(c.bandwidth, 0.0f, 1.0f);
    lowL_.lowShelf(100.0f, lowBoostDb - lowAttenDb, q, sr_);
    lowR_.lowShelf(100.0f, lowBoostDb - lowAttenDb, q, sr_);
    highL_.highShelf(10000.0f, highBoostDb - highAttenDb, q, sr_);
    highR_.highShelf(10000.0f, highBoostDb - highAttenDb, q, sr_);
  }

  inline void process(float& l, float& r) noexcept {
    l = highL_.process(lowL_.process(l));
    r = highR_.process(lowR_.process(r));
  }

  void reset() noexcept {
    lowL_ = lowR_ = highL_ = highR_ = Biquad{};
  }

  int latencySamples() const noexcept { return 0; }

private:
  float sr_ = 48000.0f;
  Biquad lowL_, lowR_, highL_, highR_;
};

} // namespace hdsp::modules
