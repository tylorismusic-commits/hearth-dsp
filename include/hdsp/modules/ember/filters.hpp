// hearth-dsp shared module — filters.hpp
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "smoother.hpp"
#include "util.hpp"

namespace hdsp::ember {

// Topology-preserving one-pole (Zavalishin).  Stable under modulation.
struct OnePole {
  float G = 0.0f;
  float ic = 0.0f;

  void set(float cutoff, float sampleRate) noexcept {
    cutoff = clamp(cutoff, 1.0f, sampleRate * 0.49f);
    const float g = std::tan(kPi * cutoff / sampleRate);
    G = g / (1.0f + g);
  }

  void reset() noexcept { ic = 0.0f; }

  inline float lp(float x) noexcept {
    const float v = (x - ic) * G;
    const float y = v + ic;
    ic = y + v;
    // Flush denormals so long silences do not fall off the slow path.
    if (std::fabs(ic) < 1.0e-20f) {
      ic = 0.0f;
    }
    return y;
  }

  inline float hp(float x) noexcept { return x - lp(x); }
};

// Topology-preserving state-variable filter.  Coefficients are shared by both
// channels; state is per channel.
struct SVFCoeffs {
  float g = 0.0f;
  float k = 1.0f;
  float a1 = 0.0f;
  float a2 = 0.0f;
  float a3 = 0.0f;

  void set(float cutoff, float q, float sampleRate) noexcept {
    cutoff = clamp(cutoff, 5.0f, sampleRate * 0.49f);
    g = std::tan(kPi * cutoff / sampleRate);
    k = 1.0f / std::max(q, 0.05f);
    a1 = 1.0f / (1.0f + g * (g + k));
    a2 = g * a1;
    a3 = g * a2;
  }
};

struct SVFState {
  float ic1 = 0.0f;
  float ic2 = 0.0f;

  void reset() noexcept { ic1 = ic2 = 0.0f; }

  inline void process(float x, const SVFCoeffs& c,
                      float& lp, float& bp, float& hp) noexcept {
    const float v3 = x - ic2;
    const float v1 = c.a1 * ic1 + c.a2 * v3;
    const float v2 = ic2 + c.a2 * ic1 + c.a3 * v3;
    ic1 = 2.0f * v1 - ic1;
    ic2 = 2.0f * v2 - ic2;
    if (std::fabs(ic1) < 1.0e-20f) {
      ic1 = 0.0f;
    }
    if (std::fabs(ic2) < 1.0e-20f) {
      ic2 = 0.0f;
    }
    lp = v2;
    bp = v1;
    hp = x - c.k * v1 - v2;
  }
};

// Third-order Butterworth high-pass (18 dB/oct), useful for the compression
// side-chain.  A first-order section is cascaded with a second-order section.
class Hpf18 {
public:
  void prepare(float sampleRate) noexcept {
    sampleRate_ = sampleRate;
    for (auto& p : p1_) {
      p.set(20.0f, sampleRate);
    }
    coeffs_.set(20.0f, 1.0f, sampleRate);
    for (auto& s : state_) {
      s.reset();
    }
    freq_.setTime(20.0f, sampleRate);
    mix_.setTime(20.0f, sampleRate);
    freq_.reset(20.0f);
    mix_.reset(0.0f);
  }

  void setTargets(float hz) noexcept {
    if (hz < 1.0f) {
      freq_.setTarget(20.0f);
      mix_.setTarget(0.0f);
    } else {
      freq_.setTarget(hz);
      mix_.setTarget(1.0f);
    }
  }

  void tick() noexcept {
    freq_.next();
    mix_.next();
  }

  void update() noexcept {
    const float f = freq_.current();
    for (auto& p : p1_) {
      p.set(f, sampleRate_);
    }
    coeffs_.set(f, 1.0f, sampleRate_);
  }

  inline float process(int channel, float x) noexcept {
    float lp, bp, hp;
    state_[channel].process(p1_[channel].hp(x), coeffs_, lp, bp, hp);
    const float m = mix_.current();
    return x + m * (hp - x);
  }

  void reset() noexcept {
    for (auto& p : p1_) {
      p.reset();
    }
    for (auto& s : state_) {
      s.reset();
    }
    freq_.snap();
    mix_.snap();
    update();
  }

private:
  float sampleRate_ = 48000.0f;
  OnePole p1_[2];
  SVFCoeffs coeffs_;
  SVFState state_[2];
  Smoothed freq_;
  Smoothed mix_;
};

} // namespace hdsp::ember
