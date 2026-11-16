// hearth-dsp — crossover.hpp
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "biquad.hpp"

namespace hdsp {

// 4th-order Linkwitz-Riley split: two cascaded 2nd-order Butterworth
// sections.  The low and high outputs are in phase at the crossover and sum
// to an allpass response, so a three-band serial split reconstructs the input
// magnitude when the bands are clean.
struct LinkwitzRiley4 {
  void prepare(float sampleRate) noexcept {
    sampleRate_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    reset();
  }

  void setCrossover(float freq) noexcept {
    const float q = 0.70710678f;
    lowA_.lowPass(freq, q, sampleRate_);
    lowB_.lowPass(freq, q, sampleRate_);
    highA_.highPass(freq, q, sampleRate_);
    highB_.highPass(freq, q, sampleRate_);
  }

  void reset() noexcept {
    lowA_.reset();
    lowB_.reset();
    highA_.reset();
    highB_.reset();
  }

  inline void process(float x, float& low, float& high) noexcept {
    low = lowB_.process(lowA_.process(x));
    high = highB_.process(highA_.process(x));
  }

private:
  float sampleRate_ = 48000.0f;
  Biquad lowA_;
  Biquad lowB_;
  Biquad highA_;
  Biquad highB_;
};

// Three bands from two serial LR4 splits.  `low` comes from the first split;
// `mid` and `high` come from splitting the first high output, so
// low + mid + high reconstructs the input when the bands are not coloured.
struct ThreeBandCrossover {
  void prepare(float sampleRate) noexcept {
    lowSplit_.prepare(sampleRate);
    highSplit_.prepare(sampleRate);
  }

  void setCrossovers(float lowFreq, float highFreq) noexcept {
    lowSplit_.setCrossover(lowFreq);
    highSplit_.setCrossover(highFreq);
  }

  void reset() noexcept {
    lowSplit_.reset();
    highSplit_.reset();
  }

  inline void process(float x, float& low, float& mid, float& high) noexcept {
    float rest = 0.0f;
    lowSplit_.process(x, low, rest);
    highSplit_.process(rest, mid, high);
  }

private:
  LinkwitzRiley4 lowSplit_;
  LinkwitzRiley4 highSplit_;
};

} // namespace hdsp
