// hearth-dsp shared module — eq.hpp
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "filters.hpp"
#include "smoother.hpp"
#include "util.hpp"

namespace hdsp::ember {

enum class EqShape {
  LowShelf,
  Bell,
  HighShelf,
};

// A single mastering EQ band built from a TPT state-variable filter.  At 0 dB
// the band is mathematically transparent (A == 1), so a flat band does not add
// phase shift.
class EqBand {
public:
  void prepare(float sampleRate, EqShape shape, float freq, float gainDb,
               float q) noexcept {
    sampleRate_ = sampleRate;
    shape_ = shape;
    freq_.setTime(28.0f, sampleRate);
    gain_.setTime(28.0f, sampleRate);
    q_.setTime(28.0f, sampleRate);
    freq_.reset(freq);
    gain_.reset(dbToGain(gainDb));
    q_.reset(q);
    for (auto& s : state_) {
      s.reset();
    }
    for (auto& s : shelf_) {
      s.reset();
    }
    update();
  }

  void setTargets(float freq, float gainDb, float q) noexcept {
    freq_.setTarget(freq);
    gain_.setTarget(dbToGain(gainDb));
    q_.setTarget(q);
  }

  void tick() noexcept {
    freq_.next();
    gain_.next();
    q_.next();
  }

  void update() noexcept {
    const float f = freq_.current();
    if (shape_ == EqShape::Bell) {
      coeffs_.set(f, q_.current(), sampleRate_);
    } else {
      // Broad, first-order shelves: no dip above/below the shelf.
      for (auto& s : shelf_) {
        s.set(f, sampleRate_);
      }
    }
    a_ = gain_.current();
  }

  inline float process(int channel, float x) noexcept {
    if (shape_ == EqShape::LowShelf) {
      return x + (a_ - 1.0f) * shelf_[channel].lp(x);
    }
    if (shape_ == EqShape::HighShelf) {
      return x + (a_ - 1.0f) * shelf_[channel].hp(x);
    }
    float lp, bp, hp;
    state_[channel].process(x, coeffs_, lp, bp, hp);
    // bp has a peak gain of 1/k, so k*bp is unity at the centre and the
    // band reaches exactly `a` there.
    return x + (a_ - 1.0f) * coeffs_.k * bp;
  }

  void reset() noexcept {
    for (auto& s : state_) {
      s.reset();
    }
    for (auto& s : shelf_) {
      s.reset();
    }
    freq_.snap();
    gain_.snap();
    q_.snap();
    update();
  }

private:
  float sampleRate_ = 48000.0f;
  EqShape shape_ = EqShape::Bell;
  SVFCoeffs coeffs_;
  SVFState state_[2];
  OnePole shelf_[2];
  Smoothed freq_;
  Smoothed gain_;
  Smoothed q_;
  float a_ = 1.0f;
};

// Low shelf -> presence bell -> air shelf.  This is the gentle "finished"
// curve behind the one knob: weight at the bottom, a broad upper-mid lift,
// and a little open top.
class EqSection {
public:
  void prepare(float sampleRate) noexcept {
    low_.prepare(sampleRate, EqShape::LowShelf, 105.0f, 0.0f, 0.70710678f);
    presence_.prepare(sampleRate, EqShape::Bell, 3000.0f, 0.0f, 0.65f);
    air_.prepare(sampleRate, EqShape::HighShelf, 11000.0f, 0.0f, 0.70710678f);
  }

  void setTargets(float lowFreq, float lowGain, float presenceFreq,
                  float presenceGain, float presenceQ, float airFreq,
                  float airGain) noexcept {
    low_.setTargets(lowFreq, lowGain, 0.70710678f);
    presence_.setTargets(presenceFreq, presenceGain, presenceQ);
    air_.setTargets(airFreq, airGain, 0.70710678f);
  }

  void tick() noexcept {
    low_.tick();
    presence_.tick();
    air_.tick();
  }

  void update() noexcept {
    low_.update();
    presence_.update();
    air_.update();
  }

  inline float process(int channel, float x) noexcept {
    x = low_.process(channel, x);
    x = presence_.process(channel, x);
    x = air_.process(channel, x);
    return x;
  }

  void reset() noexcept {
    low_.reset();
    presence_.reset();
    air_.reset();
  }

private:
  EqBand low_;
  EqBand presence_;
  EqBand air_;
};

} // namespace hdsp::ember
