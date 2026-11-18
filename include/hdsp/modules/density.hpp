// hearth-dsp — harmonic density module
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Original harmonic-density stage. It leans on the ADAA shaper's asymmetric
// bias to add even and odd harmonics, then blends the density back over the
// dry signal. This is not an OInflator clone and uses no proprietary curve.
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/adaa.hpp>
#include <hdsp/biquad.hpp>
#include <hdsp/db.hpp>
#include <hdsp/smoother.hpp>

namespace hdsp::modules {

struct DensityControls {
  float input = 0.50f;   // 0..1 -> -12..+12 dB
  float effect = 0.50f;  // 0..1
  float output = 0.50f;  // 0..1 -> -12..+12 dB
};

class Density {
public:
  void prepare(float sampleRate) noexcept {
    rate_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    inputSmooth_.setTime(12.0f, rate_);
    effectSmooth_.setTime(20.0f, rate_);
    outputSmooth_.setTime(12.0f, rate_);
    for (auto& h : hp_) {
      h.highPass(15.0f, 0.7071f, rate_);
      h.reset();
    }
    reset();
  }

  void setControls(const DensityControls& c) noexcept {
    inputSmooth_.setTarget(dbToGain(-12.0f + 24.0f * clamp01(c.input)));
    effectSmooth_.setTarget(clamp01(c.effect));
    outputSmooth_.setTarget(dbToGain(-12.0f + 24.0f * clamp01(c.output)));
  }

  inline void process(float& left, float& right) noexcept {
    const float in = inputSmooth_.next();
    const float effect = effectSmooth_.next();
    const float out = outputSmooth_.next();

    const float dryL = left;
    const float dryR = right;
    const float k = 1.0f + 4.0f * effect;
    const float bias = 0.02f + 0.18f * effect;

    float wetL = shaper_[0].process(dryL * in, k, bias);
    float wetR = shaper_[1].process(dryR * in, k, bias);
    wetL = hp_[0].process(wetL) * out;
    wetR = hp_[1].process(wetR) * out;

    left = dryL + effect * (wetL - dryL);
    right = dryR + effect * (wetR - dryR);
  }

  void reset() noexcept {
    for (auto& s : shaper_) s.reset();
    for (auto& h : hp_) h.reset();
  }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  float rate_ = 48000.0f;
  Smoothed inputSmooth_;
  Smoothed effectSmooth_;
  Smoothed outputSmooth_;
  AsymShaper shaper_[2];
  Biquad hp_[2];
};

} // namespace hdsp::modules
