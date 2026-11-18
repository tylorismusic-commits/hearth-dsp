// hearth-dsp shared module — saturator.hpp
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "filters.hpp"
#include "smoother.hpp"
#include "util.hpp"

namespace hdsp::ember {

// First-order antiderivative anti-aliased asymmetric tanh.  The bias creates
// the even-order harmonics of a single-ended Class-A stage while ADAA keeps
// the aliasing under control without adding latency.
class AdaaTanh {
public:
  static float antiderivative(float x, float bias) noexcept {
    return logCosh(x + bias) - std::tanh(bias) * x;
  }

  float process(float x, float bias) noexcept {
    if (!primed_) {
      x1_ = x;
      f1_ = antiderivative(x, bias);
      bias_ = bias;
      primed_ = true;
      return std::tanh(x + bias) - std::tanh(bias);
    }

    if (std::fabs(bias - bias_) > 1.0e-7f) {
      f1_ = antiderivative(x1_, bias);
      bias_ = bias;
    }

    const float fx = antiderivative(x, bias);
    const float dx = x - x1_;
    float y;
    if (std::fabs(dx) < 1.0e-4f) {
      y = std::tanh(0.5f * (x + x1_) + bias) - std::tanh(bias);
    } else {
      y = (fx - f1_) / dx;
    }

    x1_ = x;
    f1_ = fx;
    return y;
  }

  void reset() noexcept {
    x1_ = 0.0f;
    f1_ = 0.0f;
    bias_ = 0.0f;
    primed_ = false;
  }

private:
  float x1_ = 0.0f;
  float f1_ = 0.0f;
  float bias_ = 0.0f;
  bool primed_ = false;
};

// A gentle "finished" harmonic stage: a little asymmetric soft saturation,
// blended in parallel so the low end stays tight and the top keeps its sheen.
class HarmonicStage {
public:
  void prepare(float sampleRate) noexcept {
    sampleRate_ = sampleRate;
    drive_.setTime(32.0f, sampleRate);
    bias_.setTime(32.0f, sampleRate);
    mix_.setTime(32.0f, sampleRate);
    drive_.reset(1.0f);
    bias_.reset(0.0f);
    mix_.reset(0.0f);
    for (auto& s : sat_) {
      s.reset();
    }
    for (auto& hp : dc_) {
      hp.set(14.0f, sampleRate);
      hp.reset();
    }
    for (auto& lp : preLp_) {
      lp.set(9000.0f, sampleRate);
      lp.reset();
    }
    for (auto& lp : diffLp_) {
      lp.set(9000.0f, sampleRate);
      lp.reset();
    }
  }

  void setTargets(float drive, float bias, float mix01) noexcept {
    drive_.setTarget(std::max(drive, 1.0e-3f));
    bias_.setTarget(clamp(bias, 0.0f, 0.25f));
    mix_.setTarget(clamp(mix01, 0.0f, 1.0f));
  }

  void tick() noexcept {
    drive_.next();
    bias_.next();
    mix_.next();
  }

  inline float process(int channel, float x) noexcept {
    const float drive = drive_.current();
    const float bias = bias_.current();
    const float mix = mix_.current();

    // Saturate a band-limited copy and add only the generated difference to
    // the full-band dry signal.  The dry air band is never passed through the
    // nonlinearity's high-frequency averaging, so the top end stays open.
    const float band = preLp_[channel].lp(x);
    float wet = sat_[channel].process(band * drive, bias) * (1.0f / drive);
    wet = dc_[channel].hp(wet);
    float diff = wet - band;
    diff = diffLp_[channel].lp(diff);
    return x + mix * diff;
  }

  void reset() noexcept {
    for (auto& s : sat_) {
      s.reset();
    }
    for (auto& hp : dc_) {
      hp.reset();
    }
    for (auto& lp : diffLp_) {
      lp.reset();
    }
    for (auto& lp : preLp_) {
      lp.reset();
    }
    drive_.snap();
    bias_.snap();
    mix_.snap();
  }

private:
  float sampleRate_ = 48000.0f;
  AdaaTanh sat_[2];
  OnePole dc_[2];
  OnePole diffLp_[2];
  OnePole preLp_[2];
  Smoothed drive_;
  Smoothed bias_;
  Smoothed mix_;
};

} // namespace hdsp::ember
