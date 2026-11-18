// hearth-dsp — three-band compressor
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A compact three-band compressor: per-band thresholds, shared ratio, attack,
// release and makeup. The Linkwitz-Riley split keeps the bands phase-aligned
// when summed.
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/crossover.hpp>
#include <hdsp/db.hpp>

namespace hdsp::modules {

struct MultibandControls {
  float lowThreshold = 0.50f;
  float midThreshold = 0.50f;
  float highThreshold = 0.50f;
  float ratio = 0.30f;
  float attack = 0.20f;
  float release = 0.40f;
  float makeup = 0.00f;
};

class Multiband {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    crossL_.prepare(sr_);
    crossR_.prepare(sr_);
    crossL_.setCrossovers(200.0f, 2000.0f);
    crossR_.setCrossovers(200.0f, 2000.0f);
    reset();
  }

  void setControls(const MultibandControls& c) noexcept {
    threshold_[0] = -60.0f + 60.0f * clamp01(c.lowThreshold);
    threshold_[1] = -60.0f + 60.0f * clamp01(c.midThreshold);
    threshold_[2] = -60.0f + 60.0f * clamp01(c.highThreshold);
    ratio_ = 1.0f + 9.0f * clamp01(c.ratio);
    attackCoeff_ = coeff(0.1f * std::pow(500.0f, clamp01(c.attack)));
    releaseCoeff_ = coeff(10.0f * std::pow(100.0f, clamp01(c.release)));
    makeup_ = dbToGain(24.0f * clamp01(c.makeup));
  }

  inline void process(float& l, float& r) noexcept {
    float lowL, midL, highL, lowR, midR, highR;
    crossL_.process(l, lowL, midL, highL);
    crossR_.process(r, lowR, midR, highR);

    float bandsL[3] = {lowL, midL, highL};
    float bandsR[3] = {lowR, midR, highR};
    float maxGr = 0.0f;
    for (int b = 0; b < 3; ++b) {
      const float detector = std::max(std::fabs(bandsL[b]), std::fabs(bandsR[b]));
      const float coeff = detector > env_[b] ? attackCoeff_ : releaseCoeff_;
      env_[b] += (detector - env_[b]) * coeff;

      const float envDb = gainToDb(std::max(env_[b], 1.0e-6f));
      const float overDb = envDb - threshold_[b];
      float reduction = 0.0f;
      if (overDb > 0.0f) {
        reduction = overDb * (1.0f - 1.0f / ratio_);
      }
      const float gain = dbToGain(-reduction);
      bandsL[b] *= gain;
      bandsR[b] *= gain;
      maxGr = std::max(maxGr, reduction);
    }

    l = (bandsL[0] + bandsL[1] + bandsL[2]) * makeup_;
    r = (bandsR[0] + bandsR[1] + bandsR[2]) * makeup_;
    lastGrDb_ = maxGr;
  }

  void reset() noexcept {
    crossL_.reset();
    crossR_.reset();
    for (int b = 0; b < 3; ++b) env_[b] = 0.0f;
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

  float sr_ = 48000.0f;
  hdsp::ThreeBandCrossover crossL_;
  hdsp::ThreeBandCrossover crossR_;
  float threshold_[3] = {-30.0f, -30.0f, -30.0f};
  float env_[3] = {0.0f, 0.0f, 0.0f};
  float ratio_ = 3.7f;
  float attackCoeff_ = 0.02f;
  float releaseCoeff_ = 0.001f;
  float makeup_ = 1.0f;
  float lastGrDb_ = 0.0f;
};

} // namespace hdsp::modules
