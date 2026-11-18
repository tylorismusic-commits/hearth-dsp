// hearth-dsp — de-esser module
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/biquad.hpp>
#include <hdsp/db.hpp>

namespace hdsp::modules {

struct DeEssControls {
  float frequency = 0.60f;  // 0..1 -> 4..12 kHz
  float threshold = 0.60f;  // 0..1 -> -40..0 dB
  float range = 0.50f;      // 0..1 -> 0..24 dB
  float mix = 1.00f;
};

class DeEss {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    attackCoeff_ = coeff(1.0f);
    releaseCoeff_ = coeff(50.0f);
    gainCoeff_ = coeff(10.0f);
    reset();
  }

  void setControls(const DeEssControls& c) noexcept {
    const float freq = 4000.0f * std::pow(3.0f, clamp01(c.frequency));
    hpL_.highPass(freq, 0.707f, sr_);
    hpR_.highPass(freq, 0.707f, sr_);
    threshold_ = dbToGain(-40.0f + 40.0f * clamp01(c.threshold));
    rangeDb_ = clamp01(c.range) * 24.0f;
    mix_ = clamp01(c.mix);
  }

  inline void process(float& l, float& r) noexcept {
    const float highL = hpL_.process(l);
    const float highR = hpR_.process(r);
    const float lowL = l - highL;
    const float lowR = r - highR;
    const float detector = std::max(std::fabs(highL), std::fabs(highR));
    const float envCoeff = detector > env_ ? attackCoeff_ : releaseCoeff_;
    env_ += (detector - env_) * envCoeff;

    float reduction = 0.0f;
    if (env_ > threshold_) {
      const float overDb = gainToDb(env_ / std::max(threshold_, 1.0e-6f));
      reduction = std::min(overDb, rangeDb_);
    }
    const float target = dbToGain(-reduction);
    gain_ += (target - gain_) * gainCoeff_;
    const float wetL = lowL + highL * gain_;
    const float wetR = lowR + highR * gain_;
    const float dryL = l, dryR = r;
    l = dryL + (wetL - dryL) * mix_;
    r = dryR + (wetR - dryR) * mix_;
    lastGrDb_ = -gainToDb(std::max(gain_, 1.0e-6f));
  }

  void reset() noexcept {
    hpL_ = Biquad{};
    hpR_ = Biquad{};
    env_ = 0.0f;
    gain_ = 1.0f;
    lastGrDb_ = 0.0f;
  }

  float lastGrDb() const noexcept { return lastGrDb_; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }
  float coeff(float ms) const noexcept {
    const float samples = std::max(ms, 0.01f) * 0.001f * sr_;
    return 1.0f - std::exp(-1.0f / samples);
  }

  float sr_ = 48000.0f;
  float threshold_ = 0.1f;
  float rangeDb_ = 12.0f;
  float mix_ = 1.0f;
  float attackCoeff_ = 0.02f;
  float releaseCoeff_ = 0.001f;
  float gainCoeff_ = 0.002f;
  float env_ = 0.0f;
  float gain_ = 1.0f;
  float lastGrDb_ = 0.0f;
  Biquad hpL_, hpR_;
};

} // namespace hdsp::modules
