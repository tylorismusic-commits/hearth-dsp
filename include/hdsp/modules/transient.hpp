// hearth-dsp — transient shaper module
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

namespace hdsp::modules {

struct TransientControls {
  float attack = 0.50f;   // 0..1, 0.5 neutral
  float sustain = 0.50f;  // 0..1, 0.5 neutral
  float mix = 1.00f;
};

class Transient {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    fastCoeff_ = coeff(1.0f);
    slowCoeff_ = coeff(50.0f);
    gainCoeff_ = coeff(5.0f);
    reset();
  }

  void setControls(const TransientControls& c) noexcept {
    attack_ = (clamp01(c.attack) - 0.5f) * 2.0f;
    sustain_ = (clamp01(c.sustain) - 0.5f) * 2.0f;
    mix_ = clamp01(c.mix);
  }

  inline void process(float& l, float& r) noexcept {
    const float detector = std::max(std::fabs(l), std::fabs(r));
    fast_ += (detector - fast_) * fastCoeff_;
    slow_ += (detector - slow_) * slowCoeff_;
    const float diff = fast_ - slow_;
    const float normal = std::max(slow_, 1.0e-4f);
    const float pos = std::min(std::max(diff, 0.0f) / normal, 1.0f);
    const float neg = std::min(std::max(-diff, 0.0f) / normal, 1.0f);
    const float target =
        1.0f + attack_ * pos * 0.75f + sustain_ * neg * 0.75f;
    gain_ += (target - gain_) * gainCoeff_;
    const float dryL = l, dryR = r;
    l = dryL + (dryL * gain_ - dryL) * mix_;
    r = dryR + (dryR * gain_ - dryR) * mix_;
  }

  void reset() noexcept {
    fast_ = 0.0f;
    slow_ = 0.0f;
    gain_ = 1.0f;
  }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }
  float coeff(float ms) const noexcept {
    const float samples = std::max(ms, 0.01f) * 0.001f * sr_;
    return 1.0f - std::exp(-1.0f / samples);
  }

  float sr_ = 48000.0f;
  float fastCoeff_ = 0.02f;
  float slowCoeff_ = 0.001f;
  float gainCoeff_ = 0.004f;
  float fast_ = 0.0f;
  float slow_ = 0.0f;
  float gain_ = 1.0f;
  float attack_ = 0.0f;
  float sustain_ = 0.0f;
  float mix_ = 1.0f;
};

} // namespace hdsp::modules
