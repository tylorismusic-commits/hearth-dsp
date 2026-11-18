// hearth-dsp — JClones LMC-1 console compressor
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct LMC1Controls {
  float input = 0.50f;      // 0..1 -> -20..+20 dB
  float threshold = 0.50f;  // 0..1 -> -48..0 dB
  float output = 0.50f;     // 0..1 -> -20..+20 dB
  bool boxTone = true;
};

class LMC1 {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    const float e1 = std::exp(-340.0f / sr_ * 2.0f * 3.14159265358979323846f);
    filter1K1_ = e1;
    filter1K2_ = (e1 + 1.0f) * 0.5f;
    const float e2 = std::exp(-4000.0f / sr_ * 2.0f * 3.14159265358979323846f);
    filter2K1_ = e2;
    filter2K2_ = 1.0f - e2;
    attackK_ = 1.0f / (0.0025f * sr_);
    releaseK_ = 1.0f / (0.025f * sr_);
    reset();
  }

  void setControls(const LMC1Controls& c) noexcept {
    inputGain_ = dbToGain(-20.0f + 40.0f * clamp01(c.input));
    thresholdDb_ = -48.0f + 48.0f * clamp01(c.threshold);
    outputGain_ = dbToGain(-20.0f + 40.0f * clamp01(c.output));
    boxTone_ = c.boxTone;
  }

  inline void process(float& l, float& r) noexcept {
    l = ch_[0].process(l * inputGain_, thresholdDb_, boxTone_, filter1K1_,
                       filter1K2_, filter2K1_, filter2K2_, attackK_,
                       releaseK_) *
        outputGain_;
    r = ch_[1].process(r * inputGain_, thresholdDb_, boxTone_, filter1K1_,
                       filter1K2_, filter2K1_, filter2K2_, attackK_,
                       releaseK_) *
        outputGain_;
    lastGrDb_ = std::max(-gainToDb(std::max(ch_[0].feedbackGr, 1.0e-6f)),
                         -gainToDb(std::max(ch_[1].feedbackGr, 1.0e-6f)));
  }

  void reset() noexcept {
    for (auto& ch : ch_) ch.reset();
    lastGrDb_ = 0.0f;
  }

  int latencySamples() const noexcept { return 0; }
  float lastGrDb() const noexcept { return lastGrDb_; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  struct Channel {
    float filter1S = 0.0f;
    float filter2S = 0.0f;
    float arState = 0.0f;
    float feedbackGr = 1.0f;
    void reset() noexcept {
      filter1S = filter2S = arState = 0.0f;
      feedbackGr = 1.0f;
    }
    inline float process(float x, float thresholdDb, bool boxTone,
                         float f1k1, float f1k2, float f2k1, float f2k2,
                         float attackK, float releaseK) noexcept {
      float nextS = f1k1 * filter1S + x;
      float y = (nextS - filter1S) * f1k2;
      filter1S = nextS;
      nextS = f2k1 * filter2S + y;
      y = nextS * f2k2;
      filter2S = nextS;
      y *= feedbackGr;
      const float output = boxTone ? y : x * feedbackGr;
      const float y2 = y * y;
      const float arK = y2 > arState ? attackK : releaseK;
      arState += (y2 - arState) * arK;
      float levelLin = std::sqrt(std::max(arState, 0.0f));
      if (levelLin < 1.0e-8f) levelLin = 1.0e-8f;
      const float levelDb = gainToDb(levelLin);
      float grDb = 0.0f;
      if (levelDb < thresholdDb + 13.0f) {
        if (levelDb >= thresholdDb) grDb = (levelDb - thresholdDb) * -3.0769231f;
      } else {
        grDb = -40.0f;
      }
      feedbackGr = dbToGain(grDb);
      return output;
    }
  };

  float sr_ = 48000.0f;
  float inputGain_ = 1.0f;
  float outputGain_ = 1.0f;
  float thresholdDb_ = -24.0f;
  float filter1K1_ = 0.99f, filter1K2_ = 0.995f;
  float filter2K1_ = 0.8f, filter2K2_ = 0.2f;
  float attackK_ = 0.008f, releaseK_ = 0.0008f;
  bool boxTone_ = true;
  float lastGrDb_ = 0.0f;
  Channel ch_[2];
};

} // namespace hdsp::modules
