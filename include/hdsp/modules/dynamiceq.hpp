// hearth-dsp — single-band dynamic EQ
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A parallel band-pass dynamic EQ: the selected band is compressed and mixed
// back over the dry signal. It is cut-only, which keeps it stable and useful
// for de-essing, resonance control and taming harshness.
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct DynamicEqControls {
  float frequency = 0.50f;  // 0..1 -> 20..20k Hz
  float q = 0.50f;          // 0..1 -> 0.3..6
  float threshold = 0.60f;  // 0..1 -> -40..0 dB
  float range = 0.50f;      // 0..1 -> 0..24 dB
  float ratio = 0.50f;      // 0..1 -> 1..10
  float mix = 1.00f;
};

class DynamicEq {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    envAttack_ = coeff(1.0f);
    envRelease_ = coeff(80.0f);
    gainCoeff_ = coeff(5.0f);
    reset();
  }

  void setControls(const DynamicEqControls& c) noexcept {
    hz_ = 20.0f * std::pow(1000.0f, clamp01(c.frequency));
    q_ = 0.3f * std::pow(20.0f, clamp01(c.q));
    threshold_ = dbToGain(-40.0f + 40.0f * clamp01(c.threshold));
    rangeDb_ = clamp01(c.range) * 24.0f;
    ratio_ = 1.0f + 9.0f * clamp01(c.ratio);
    mix_ = clamp01(c.mix);
  }

  inline void process(float& l, float& r) noexcept {
    const float g = std::tan(3.14159265f * hz_ / sr_);
    const float k = 1.0f / std::max(q_, 0.1f);
    float lpL, bpL, hpL, lpR, bpR, hpR;
    svfL_.process(l, g, k, lpL, bpL, hpL);
    svfR_.process(r, g, k, lpR, bpR, hpR);
    const float bandL = bpL * k;
    const float bandR = bpR * k;

    const float detector = std::max(std::fabs(bandL), std::fabs(bandR));
    const float envCoeff = detector > env_ ? envAttack_ : envRelease_;
    env_ += (detector - env_) * envCoeff;

    float reduction = 0.0f;
    if (env_ > threshold_) {
      const float overDb = gainToDb(env_ / std::max(threshold_, 1.0e-6f));
      reduction = std::min(rangeDb_, overDb * (1.0f - 1.0f / ratio_));
    }
    const float target = dbToGain(-reduction);
    gain_ += (target - gain_) * gainCoeff_;

    const float dryL = l, dryR = r;
    l = dryL + bandL * (gain_ - 1.0f);
    r = dryR + bandR * (gain_ - 1.0f);
    l = dryL + (l - dryL) * mix_;
    r = dryR + (r - dryR) * mix_;
    lastGrDb_ = -gainToDb(std::max(gain_, 1.0e-6f));
  }

  void reset() noexcept {
    svfL_.reset();
    svfR_.reset();
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

  struct Svf {
    float ic1 = 0.0f;
    float ic2 = 0.0f;
    void reset() noexcept {
      ic1 = ic2 = 0.0f;
    }
    inline void process(float x, float g, float k, float& lp, float& bp,
                        float& hp) noexcept {
      const float a1 = 1.0f / (1.0f + g * (g + k));
      const float a2 = g * a1;
      const float a3 = g * a2;
      const float v3 = x - ic2;
      const float v1 = a1 * ic1 + a2 * v3;
      const float v2 = ic2 + a2 * ic1 + a3 * v3;
      ic1 = 2.0f * v1 - ic1;
      ic2 = 2.0f * v2 - ic2;
      lp = v2;
      bp = v1;
      hp = x - k * v1 - v2;
    }
  };

  float sr_ = 48000.0f;
  float hz_ = 632.0f;
  float q_ = 1.0f;
  float threshold_ = 0.1f;
  float rangeDb_ = 12.0f;
  float ratio_ = 5.5f;
  float mix_ = 1.0f;
  float envAttack_ = 0.02f;
  float envRelease_ = 0.0003f;
  float gainCoeff_ = 0.004f;
  float env_ = 0.0f;
  float gain_ = 1.0f;
  float lastGrDb_ = 0.0f;
  Svf svfL_;
  Svf svfR_;
};

} // namespace hdsp::modules
