// hearth-dsp — gate / expander module
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct GateControls {
  float threshold = 0.50f;  // 0..1 -> -60..0 dB
  float range = 0.50f;      // 0..1 -> 0..80 dB
  float attack = 0.15f;     // 0..1 -> 0.1..50 ms
  float release = 0.35f;    // 0..1 -> 10..1000 ms
  float hold = 0.20f;       // 0..1 -> 0..500 ms
};

class Gate {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    envAttack_ = coeff(0.1f);
    envRelease_ = coeff(20.0f);
    reset();
  }

  void setControls(const GateControls& c) noexcept {
    threshold_ = dbToGain(-60.0f + 60.0f * clamp01(c.threshold));
    const float rangeDb = clamp01(c.range) * 80.0f;
    rangeGain_ = rangeDb < 0.05f ? 1.0f : dbToGain(-rangeDb);
    attackCoeff_ = coeff(0.1f * std::pow(500.0f, clamp01(c.attack)));
    releaseCoeff_ = coeff(10.0f * std::pow(100.0f, clamp01(c.release)));
    holdSamples_ = static_cast<int>(clamp01(c.hold) * 0.5f * sr_);
  }

  inline void process(float& l, float& r) noexcept {
    const float detector = std::max(std::fabs(l), std::fabs(r));
    const float envCoeff = detector > env_ ? envAttack_ : envRelease_;
    env_ += (detector - env_) * envCoeff;

    const bool above = env_ > threshold_;
    if (above) {
      holdCounter_ = holdSamples_;
    } else if (holdCounter_ > 0) {
      --holdCounter_;
    }
    const bool open = above || holdCounter_ > 0;
    const float target = open ? 1.0f : rangeGain_;
    const float gainCoeff = target > gain_ ? attackCoeff_ : releaseCoeff_;
    gain_ += (target - gain_) * gainCoeff;
    l *= gain_;
    r *= gain_;
    lastGrDb_ = gain_ > 1.0e-6f ? -gainToDb(gain_) : 96.0f;
  }

  void reset() noexcept {
    env_ = 0.0f;
    gain_ = 1.0f;
    holdCounter_ = 0;
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
  float threshold_ = 0.0f;
  float rangeGain_ = 1.0f;
  float attackCoeff_ = 0.1f;
  float releaseCoeff_ = 0.01f;
  float envAttack_ = 0.1f;
  float envRelease_ = 0.01f;
  int holdSamples_ = 0;
  int holdCounter_ = 0;
  float env_ = 0.0f;
  float gain_ = 1.0f;
  float lastGrDb_ = 0.0f;
};

} // namespace hdsp::modules
