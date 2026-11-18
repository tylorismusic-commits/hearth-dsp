// hearth-dsp — auto-filter module
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

namespace hdsp::modules {

struct FilterControls {
  float cutoff = 0.50f;     // 0..1 -> 20..20k Hz
  float resonance = 0.20f;  // 0..1 -> Q 0.5..10
  float envelope = 0.50f;   // 0.5 neutral; bipolar env amount
  float rate = 0.50f;       // 0..1 -> 0.05..10 Hz
  int mode = 0;             // 0 LP, 1 BP, 2 HP
};

class Filter {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    envAttack_ = coeff(1.0f);
    envRelease_ = coeff(100.0f);
    reset();
  }

  void setControls(const FilterControls& c) noexcept {
    baseHz_ = 20.0f * std::pow(1000.0f, clamp01(c.cutoff));
    q_ = 0.5f + 9.5f * clamp01(c.resonance);
    envAmount_ = (clamp01(c.envelope) - 0.5f) * 2.0f;
    rateHz_ = 0.05f * std::pow(200.0f, clamp01(c.rate));
    mode_ = std::clamp(c.mode, 0, 2);
  }

  inline void process(float& l, float& r) noexcept {
    const float detector = std::max(std::fabs(l), std::fabs(r));
    const float envCoeff = detector > env_ ? envAttack_ : envRelease_;
    env_ += (detector - env_) * envCoeff;

    phase_ += 2.0f * 3.14159265f * rateHz_ / sr_;
    if (phase_ >= 2.0f * 3.14159265f) phase_ -= 2.0f * 3.14159265f;
    const float lfo = std::sin(phase_);

    const float mod = envAmount_ * env_ * 2.0f + lfo * 0.5f;
    float hz = baseHz_ * std::pow(2.0f, mod);
    hz = std::clamp(hz, 20.0f, 0.45f * sr_);
    const float g = std::tan(3.14159265f * hz / sr_);
    const float k = 1.0f / std::max(q_, 0.1f);

    float lpL, bpL, hpL, lpR, bpR, hpR;
    svfL_.process(l, g, k, lpL, bpL, hpL);
    svfR_.process(r, g, k, lpR, bpR, hpR);
    l = mode_ == 0 ? lpL : (mode_ == 1 ? bpL : hpL);
    r = mode_ == 0 ? lpR : (mode_ == 1 ? bpR : hpR);
  }

  void reset() noexcept {
    svfL_.reset();
    svfR_.reset();
    env_ = 0.0f;
    phase_ = 0.0f;
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

  struct Svf {
    float ic1 = 0.0f;
    float ic2 = 0.0f;
    void reset() noexcept {
      ic1 = 0.0f;
      ic2 = 0.0f;
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
  float baseHz_ = 632.0f;
  float q_ = 0.5f;
  float envAmount_ = 0.0f;
  float rateHz_ = 0.5f;
  int mode_ = 0;
  float envAttack_ = 0.02f;
  float envRelease_ = 0.001f;
  float env_ = 0.0f;
  float phase_ = 0.0f;
  Svf svfL_;
  Svf svfR_;
};

} // namespace hdsp::modules
