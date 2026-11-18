// hearth-dsp — subharmonic synthesizer module
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Zero-crossing flip-flop divider: low-pass the bass, toggle on positive
// crossings, smooth the half-rate square into a sine and blend it under the
// dry signal. This is a musical sub-octave, not a pitch tracker.
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/biquad.hpp>

namespace hdsp::modules {

struct SubControls {
  float frequency = 0.30f;  // 0..1 -> 30..150 Hz tracking low-pass
  float amount = 0.50f;
  float drive = 0.20f;
  float mix = 1.00f;
};

class Sub {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    envAttack_ = coeff(2.0f);
    envRelease_ = coeff(80.0f);
    reset();
  }

  void setControls(const SubControls& c) noexcept {
    const float hp = 30.0f * std::pow(5.0f, clamp01(c.frequency));
    trackLp_.lowPass(hp, 0.707f, sr_);
    subLp_.lowPass(std::min(hp * 0.6f, 120.0f), 0.707f, sr_);
    amount_ = clamp01(c.amount);
    drive_ = 1.0f + 8.0f * clamp01(c.drive);
    mix_ = clamp01(c.mix);
  }

  inline void process(float& l, float& r) noexcept {
    const float dryL = l, dryR = r;
    const float mono = 0.5f * (l + r);
    const float low = trackLp_.process(mono);
    const float envCoeff = std::fabs(low) > env_ ? envAttack_ : envRelease_;
    env_ += (std::fabs(low) - env_) * envCoeff;

    if (low > 0.01f && lastLow_ <= 0.01f) {
      flip_ = -flip_;
    }
    lastLow_ = low;

    float sub = subLp_.process(flip_ * env_ * 2.0f);
    sub = std::tanh(sub * drive_);
    l = dryL + sub * amount_;
    r = dryR + sub * amount_;
    l = dryL + (l - dryL) * mix_;
    r = dryR + (r - dryR) * mix_;
  }

  void reset() noexcept {
    trackLp_ = Biquad{};
    subLp_ = Biquad{};
    env_ = 0.0f;
    lastLow_ = 0.0f;
    flip_ = 1.0f;
  }

  int latencySamples() const noexcept { return 0; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }
  float coeff(float ms) const noexcept {
    const float samples = std::max(ms, 0.01f) * 0.001f * sr_;
    return 1.0f - std::exp(-1.0f / samples);
  }

  float sr_ = 48000.0f;
  float amount_ = 0.5f;
  float drive_ = 2.6f;
  float mix_ = 1.0f;
  float envAttack_ = 0.01f;
  float envRelease_ = 0.0003f;
  float env_ = 0.0f;
  float lastLow_ = 0.0f;
  float flip_ = 1.0f;
  Biquad trackLp_;
  Biquad subLp_;
};

} // namespace hdsp::modules
