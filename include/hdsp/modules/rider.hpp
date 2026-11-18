// hearth-dsp — auto-level rider module
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Slow gain rider: measure the programme level, move the gain toward the
// target within the selected range, and smooth it so it sounds like a hand on
// a fader rather than a compressor.
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct RiderControls {
  float target = 0.60f;  // 0..1 -> -40..0 dB
  float speed = 0.50f;   // 0..1 -> 20..1000 ms
  float range = 0.50f;   // 0..1 -> 0..24 dB
  float mix = 1.00f;
};

class Rider {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    envAttack_ = coeff(10.0f);
    envRelease_ = coeff(200.0f);
    reset();
  }

  void setControls(const RiderControls& c) noexcept {
    targetDb_ = -40.0f + 40.0f * clamp01(c.target);
    rangeDb_ = 24.0f * clamp01(c.range);
    const float speedMs = 20.0f * std::pow(50.0f, clamp01(c.speed));
    gainCoeff_ = coeff(speedMs);
    mix_ = clamp01(c.mix);
  }

  inline void process(float& l, float& r) noexcept {
    const float dryL = l, dryR = r;
    const float detector = std::max(std::fabs(l), std::fabs(r));
    const float envCoeff = detector > env_ ? envAttack_ : envRelease_;
    env_ += (detector - env_) * envCoeff;

    float desiredDb = 0.0f;
    if (env_ > 1.0e-5f) {
      desiredDb = targetDb_ - gainToDb(env_);
      desiredDb = std::clamp(desiredDb, -rangeDb_, rangeDb_);
    }
    const float targetGain = dbToGain(desiredDb);
    gain_ += (targetGain - gain_) * gainCoeff_;
    l = dryL * gain_;
    r = dryR * gain_;
    l = dryL + (l - dryL) * mix_;
    r = dryR + (r - dryR) * mix_;
    lastGrDb_ = std::fabs(-gainToDb(std::max(gain_, 1.0e-6f)));
  }

  void reset() noexcept {
    env_ = 0.0f;
    gain_ = 1.0f;
    lastGrDb_ = 0.0f;
  }

  float lastGrDb() const noexcept { return lastGrDb_; }
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
  float targetDb_ = -16.0f;
  float rangeDb_ = 12.0f;
  float mix_ = 1.0f;
  float envAttack_ = 0.01f;
  float envRelease_ = 0.0003f;
  float gainCoeff_ = 0.01f;
  float env_ = 0.0f;
  float gain_ = 1.0f;
  float lastGrDb_ = 0.0f;
};

} // namespace hdsp::modules
