// hearth-dsp shared module — preamp.hpp
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "filters.hpp"
#include "smoother.hpp"
#include "util.hpp"

namespace hdsp::reel {

// First-order antiderivative anti-aliased asymmetric tanh.  The asymmetry
// (bias) generates the even-order harmonics associated with a single-ended
// Class-A stage; ADAA keeps the aliasing under control without adding latency.
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
      // The antiderivative changed; rebase the previous sample so the
      // difference quotient remains consistent while the knob moves.
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

// Console input stage: input transformer high-pass, Class-A soft saturation,
// and partial drive compensation so the knob changes character more than level.
class Preamp {
public:
  void prepare(float sampleRate) noexcept {
    sampleRate_ = sampleRate;
    hp_.set(15.0f, sampleRate);
    drive_.setTime(20.0f, sampleRate);
    character_.setTime(20.0f, sampleRate);
    drive_.reset(1.0f);
    character_.reset(0.25f);
    sat_.reset();
    hp_.reset();
  }

  void setTargets(float driveDb, float character01) noexcept {
    drive_.setTarget(dbToGain(driveDb));
    character_.setTarget(clamp(character01, 0.0f, 1.0f));
  }

  inline float process(float x) noexcept {
    const float drive = drive_.next();
    const float c = character_.next();

    // Character drives the stage and adds asymmetry.  The extra drive is
    // compensated so the knob changes harmonic content more than level; the
    // quadratic term from the bias is what produces the audible 2nd harmonic.
    const float charDrive = 1.0f + 2.5f * c;
    const float bias = 0.15f * c;
    const float d = drive * charDrive;

    float y = sat_.process(x * d, bias);

    // Compensate the user Drive partially (so it can still add level) and the
    // Character drive fully (so it is primarily a tone control).
    y *= 1.0f / std::sqrt(std::sqrt(std::max(drive, 1.0e-6f)));
    y *= 1.0f / charDrive;

    // Transformer core high-pass.  Also removes any DC produced by the
    // asymmetric stage.
    return hp_.hp(y);
  }

  void reset() noexcept {
    sat_.reset();
    hp_.reset();
    drive_.snap();
    character_.snap();
  }

private:
  float sampleRate_ = 48000.0f;
  AdaaTanh sat_;
  OnePole hp_;
  Smoothed drive_;
  Smoothed character_;
};

} // namespace hdsp::reel
