// hearth-dsp — hybrid console EQ
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Five musical moves in one 500-series-sized EQ, inspired by the hybrid
// Helios/Pultec/API/Neve layout popularised by Analog Obsession's Frank:
// a Helios-style low cut, Pultec low boost, API midrange, Neve high boost and
// Pultec high attenuation. This is an original implementation, not a port.
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/biquad.hpp>

namespace hdsp::modules {

struct HybridEqControls {
  float lowCut = 0.0f;      // 20..800 Hz
  float lowFreq = 0.465f;   // 30..400 Hz
  float lowBoost = 0.0f;    // 0..+15 dB
  float midFreq = 0.5f;     // 200..5000 Hz
  float midGain = 0.5f;     // -15..+15 dB
  float highFreq = 0.747f;  // 2.5..16 kHz
  float highBoost = 0.0f;   // 0..+18 dB
  float highAtten = 0.0f;   // 0..-18 dB
};

class HybridEq {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    reset();
    dirty_ = true;
  }

  void setControls(const HybridEqControls& c) noexcept {
    controls_ = c;
    dirty_ = true;
  }

  inline void process(float& l, float& r) noexcept {
    updateFilters();
    l = lineAmp(highCutL_.process(highShelfL_.process(
        midL_.process(lowShelfL_.process(lowCutL_.process(l))))));
    r = lineAmp(highCutR_.process(highShelfR_.process(
        midR_.process(lowShelfR_.process(lowCutR_.process(r))))));
  }

  void reset() noexcept {
    for (auto* f : {&lowCutL_, &lowCutR_, &lowShelfL_, &lowShelfR_, &midL_,
                    &midR_, &highShelfL_, &highShelfR_, &highCutL_,
                    &highCutR_}) {
      f->reset();
    }
  }

  int latencySamples() const noexcept { return 0; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  static inline float lineAmp(float x) noexcept {
    // Gentle line-stage colour: unity slope at low level, soft peak rounding.
    return 2.0f * std::tanh(x * 0.5f);
  }

  void updateFilters() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    const float lowCut = 20.0f * std::pow(40.0f, clamp01(controls_.lowCut));
    const float lowFreq = 30.0f * std::pow(13.3333f, clamp01(controls_.lowFreq));
    const float lowBoost = 15.0f * clamp01(controls_.lowBoost);
    const float midFreq = 200.0f * std::pow(25.0f, clamp01(controls_.midFreq));
    const float midGain = -15.0f + 30.0f * clamp01(controls_.midGain);
    const float highFreq =
        2500.0f * std::pow(6.4f, clamp01(controls_.highFreq));
    const float highBoost = 18.0f * clamp01(controls_.highBoost);
    const float highAtten = -18.0f * clamp01(controls_.highAtten);

    lowCutL_.highPass(lowCut, 0.707f, sr_);
    lowCutR_.highPass(lowCut, 0.707f, sr_);
    lowShelfL_.lowShelf(lowFreq, lowBoost, 0.72f, sr_);
    lowShelfR_.lowShelf(lowFreq, lowBoost, 0.72f, sr_);
    midL_.peak(midFreq, midGain, 0.9f, sr_);
    midR_.peak(midFreq, midGain, 0.9f, sr_);
    highShelfL_.highShelf(highFreq, highBoost, 0.68f, sr_);
    highShelfR_.highShelf(highFreq, highBoost, 0.68f, sr_);
    highCutL_.highShelf(10000.0f, highAtten, 0.7f, sr_);
    highCutR_.highShelf(10000.0f, highAtten, 0.7f, sr_);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  HybridEqControls controls_;
  Biquad lowCutL_, lowCutR_, lowShelfL_, lowShelfR_, midL_, midR_,
      highShelfL_, highShelfR_, highCutL_, highCutR_;
};

} // namespace hdsp::modules
