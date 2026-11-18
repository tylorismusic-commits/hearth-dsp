// hearth-dsp — Custom EQ
// SPDX-License-Identifier: GPL-3.0-or-later
//
// An original broad-stroke EQ inspired by the Slate Custom Series approach:
// four musical bands, three fixed frequency choices per band, and different
// filter shapes for boosts and cuts. Not a port of the Slate plugin.
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/biquad.hpp>

namespace hdsp::modules {

struct CustomEqControls {
  float lowFreq = 0.5f;
  float lowGain = 0.5f;
  float lowMidFreq = 0.5f;
  float lowMidGain = 0.5f;
  float highMidFreq = 0.5f;
  float highMidGain = 0.5f;
  float highFreq = 0.5f;
  float highGain = 0.5f;
};

class CustomEq {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    reset();
    dirty_ = true;
  }

  void setControls(const CustomEqControls& c) noexcept {
    controls_ = c;
    dirty_ = true;
  }

  inline void process(float& l, float& r) noexcept {
    updateFilters();
    l = lineAmp(highB_L_.process(highA_L_.process(
        highMidB_L_.process(highMidA_L_.process(lowMidB_L_.process(
            lowMidA_L_.process(lowB_L_.process(lowA_L_.process(l)))))))));
    r = lineAmp(highB_R_.process(highA_R_.process(
        highMidB_R_.process(highMidA_R_.process(lowMidB_R_.process(
            lowMidA_R_.process(lowB_R_.process(lowA_R_.process(r)))))))));
  }

  void reset() noexcept {
    for (auto* f : {&lowA_L_, &lowA_R_, &lowB_L_, &lowB_R_, &lowMidA_L_,
                    &lowMidA_R_, &lowMidB_L_, &lowMidB_R_, &highMidA_L_,
                    &highMidA_R_, &highMidB_L_, &highMidB_R_, &highA_L_,
                    &highA_R_, &highB_L_, &highB_R_}) {
      f->reset();
    }
  }

  int latencySamples() const noexcept { return 0; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }
  static float gainDb(float v) noexcept {
    return -12.0f + 24.0f * clamp01(v);
  }
  static float pickFreq(const float (&freqs)[3], float v) noexcept {
    const int index =
        std::clamp(static_cast<int>(std::lround(clamp01(v) * 2.0f)), 0, 2);
    return freqs[index];
  }
  static inline float lineAmp(float x) noexcept {
    return 2.0f * std::tanh(x * 0.5f);
  }

  void updateFilters() noexcept {
    if (!dirty_) return;
    dirty_ = false;

    static constexpr float kLow[3] = {60.0f, 120.0f, 240.0f};
    static constexpr float kLowMid[3] = {250.0f, 500.0f, 1000.0f};
    static constexpr float kHighMid[3] = {1500.0f, 3000.0f, 6000.0f};
    static constexpr float kHigh[3] = {8000.0f, 12000.0f, 16000.0f};

    const float lowF = pickFreq(kLow, controls_.lowFreq);
    const float lowMidF = pickFreq(kLowMid, controls_.lowMidFreq);
    const float highMidF = pickFreq(kHighMid, controls_.highMidFreq);
    const float highF = pickFreq(kHigh, controls_.highFreq);
    const float lowG = gainDb(controls_.lowGain);
    const float lowMidG = gainDb(controls_.lowMidGain);
    const float highMidG = gainDb(controls_.highMidGain);
    const float highG = gainDb(controls_.highGain);

    // Boosts are broader; cuts use a slightly tighter, more focused shape.
    const float lowWide = lowG >= 0.0f ? 0.62f : 0.74f;
    lowA_L_.lowShelf(lowF, lowG * 0.82f, lowWide, sr_);
    lowA_R_.lowShelf(lowF, lowG * 0.82f, lowWide, sr_);
    lowB_L_.lowShelf(lowF * (lowG >= 0.0f ? 1.55f : 2.10f), lowG * 0.24f,
                     0.82f, sr_);
    lowB_R_.lowShelf(lowF * (lowG >= 0.0f ? 1.55f : 2.10f), lowG * 0.24f,
                     0.82f, sr_);

    const float lowMidQ = lowMidG >= 0.0f ? 0.72f : 0.88f;
    lowMidA_L_.peak(lowMidF, lowMidG * 0.84f, lowMidQ, sr_);
    lowMidA_R_.peak(lowMidF, lowMidG * 0.84f, lowMidQ, sr_);
    lowMidB_L_.peak(lowMidF * (lowMidG >= 0.0f ? 0.55f : 1.55f),
                    lowMidG * 0.18f, 0.9f, sr_);
    lowMidB_R_.peak(lowMidF * (lowMidG >= 0.0f ? 0.55f : 1.55f),
                    lowMidG * 0.18f, 0.9f, sr_);

    const float highMidQ = highMidG >= 0.0f ? 0.84f : 0.76f;
    highMidA_L_.peak(highMidF, highMidG * 0.82f, highMidQ, sr_);
    highMidA_R_.peak(highMidF, highMidG * 0.82f, highMidQ, sr_);
    highMidB_L_.peak(highMidF * (highMidG >= 0.0f ? 1.85f : 0.58f),
                     highMidG * 0.20f, 0.95f, sr_);
    highMidB_R_.peak(highMidF * (highMidG >= 0.0f ? 1.85f : 0.58f),
                     highMidG * 0.20f, 0.95f, sr_);

    const float highWide = highG >= 0.0f ? 0.58f : 0.74f;
    highA_L_.highShelf(highF, highG * 0.78f, highWide, sr_);
    highA_R_.highShelf(highF, highG * 0.78f, highWide, sr_);
    highB_L_.highShelf(highF * (highG >= 0.0f ? 0.62f : 1.35f),
                       highG * 0.28f, 0.86f, sr_);
    highB_R_.highShelf(highF * (highG >= 0.0f ? 0.62f : 1.35f),
                       highG * 0.28f, 0.86f, sr_);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  CustomEqControls controls_;
  Biquad lowA_L_, lowA_R_, lowB_L_, lowB_R_;
  Biquad lowMidA_L_, lowMidA_R_, lowMidB_L_, lowMidB_R_;
  Biquad highMidA_L_, highMidA_R_, highMidB_L_, highMidB_R_;
  Biquad highA_L_, highA_R_, highB_L_, highB_R_;
};

} // namespace hdsp::modules
