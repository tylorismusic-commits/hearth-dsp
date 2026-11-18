// hearth-dsp — Classic Limiter (JClones-derived)
// SPDX-License-Identifier: GPL-3.0-or-later
//
// C++ translation of the MIT-licensed JClones Classic Master Limiter JSFX.
// The original is a multi-pass lookahead limiter; this keeps the same
// four-pass gain-reduction structure and 3 ms delay line.
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct ClassicLimiterControls {
  float threshold = 0.75f;  // 0..1 -> -20..0 dB
  float output = 0.50f;     // 0..1 -> -12..+12 dB
  float mix = 1.00f;
};

class ClassicLimiter {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    kA_ = std::pow(0.368, 1.0 / (sr_ * 0.0005));
    kX_ = 1.0 - std::pow(0.368, 1.0 / (sr_ * 0.5));
    kC_ = std::pow(0.368, 1.0 / (sr_ * 0.001));
    kE_ = 1.0 - std::pow(0.368, 1.0 / (sr_ * 0.0005));
    delaySize_ = std::max(1, static_cast<int>(std::floor(sr_ * 0.003 + 0.5)));
    for (auto& ch : ch_) ch.prepare(sr_, delaySize_);
    reset();
  }

  void setControls(const ClassicLimiterControls& c) noexcept {
    const float thresholdDb = -20.0f + 20.0f * std::clamp(c.threshold, 0.0f, 1.0f);
    threshold_ = dbToGain(thresholdDb);
    masterGain_ = 0.977f / std::max(threshold_, 1.0e-6f);
    outputGain_ = dbToGain(-12.0f + 24.0f * std::clamp(c.output, 0.0f, 1.0f));
    mix_ = std::clamp(c.mix, 0.0f, 1.0f);
    for (auto& ch : ch_) ch.setThreshold(threshold_, masterGain_);
  }

  inline void process(float& l, float& r) noexcept {
    const float dryL = l, dryR = r;
    float grL = 0.0f, grR = 0.0f;
    l = ch_[0].process(l, kA_, kX_, kC_, kE_, threshold_, masterGain_, grL);
    r = ch_[1].process(r, kA_, kX_, kC_, kE_, threshold_, masterGain_, grR);
    l *= outputGain_;
    r *= outputGain_;
    l = dryL + (l - dryL) * mix_;
    r = dryR + (r - dryR) * mix_;
    lastGrDb_ = std::max(grL, grR);
  }

  void reset() noexcept {
    for (auto& ch : ch_) ch.reset();
    lastGrDb_ = 0.0f;
  }

  int latencySamples() const noexcept { return delaySize_; }
  float lastGrDb() const noexcept { return lastGrDb_; }

private:
  struct Channel {
    struct Filter {
      float a0 = 1.0f, a1 = 0.0f, b1 = 0.0f;
      float x1 = 0.0f, y1 = 0.0f;
      void init(float freqHz, float sr) noexcept {
        const float w0 = freqHz * 2.0f * 3.14159265358979323846f / sr;
        const float kk = 1.0f / std::tan(w0 / 2.0f);
        const float b0 = 1.0f + kk;
        const float b1 = 1.0f - kk;
        a0 = 1.0f / b0;
        a1 = 1.0f / b0;
        this->b1 = -b1 / b0;
      }
      void reset() noexcept { x1 = y1 = 0.0f; }
      inline float process(float x) noexcept {
        const float y = a0 * x + a1 * x1 + b1 * y1;
        x1 = x;
        y1 = y;
        return y;
      }
    };

    void prepare(float sr, int delaySize) noexcept {
      sr_ = sr;
      delay_.assign(static_cast<size_t>(delaySize), 0.0f);
      attack_.init(100.0f, sr);
      reset();
    }
    void setThreshold(float threshold, float masterGain) noexcept {
      threshold_ = threshold;
      masterGain_ = masterGain;
    }
    void reset() noexcept {
      iGR_A_ = iGR_B_ = iGR_X_ = iGR_C_ = iGR_D_ = iGR_E_ = 0.0f;
      attack_.reset();
      std::fill(delay_.begin(), delay_.end(), 0.0f);
      head_ = 0;
      kB_ = 1.0f;
      kD_ = 1.0f;
      gr_ = 1.0f;
    }
    inline float process(float x, float kA, float kX, float kC, float kE,
                         float threshold, float masterGain,
                         float& grDbOut) noexcept {
      constexpr float oneE20 = 1.0e-20f;
      float L = std::fabs(x) + oneE20;
      float iGR_1 = std::max(1.0f - threshold / L, oneE20);
      iGR_A_ *= kA;
      iGR_A_ = std::max(iGR_A_, iGR_1);
      iGR_B_ *= kB_;
      iGR_B_ = std::max(iGR_B_, iGR_1);
      if (iGR_A_ >= iGR_B_) {
        kB_ = 1.0f;
      } else {
        kB_ += (kA - kB_) * (iGR_B_ - iGR_A_) / sr_;
      }
      iGR_X_ += (iGR_B_ - iGR_X_) * kX;
      iGR_X_ = std::min(iGR_X_, iGR_B_);
      const float gr1 = 1.0f - attack_.process(iGR_X_);
      const float y1 = x * gr1;

      L = std::fabs(y1);
      float y2 = y1;
      float gr2 = 1.0f;
      if (L >= threshold) {
        const float iGR_2 = 1.0f - threshold / L;
        gr2 = 1.0f - iGR_2 * iGR_2 * iGR_2;
        y2 = y1 * gr2;
      }

      const float y3 = delay_[head_];
      delay_[head_] = y2;
      head_ = (head_ + 1) % static_cast<int>(delay_.size());

      L = std::fabs(y2) + oneE20;
      float iGR_3 = std::max(1.0f - threshold / L, oneE20);
      iGR_C_ *= kC;
      iGR_C_ = std::max(iGR_C_, iGR_3);
      iGR_D_ *= kD_;
      iGR_D_ = std::max(iGR_D_, iGR_3);
      if (iGR_C_ >= iGR_D_) {
        kD_ += (1.0f - kD_) * 8000.0f / sr_;
      } else {
        kD_ += (kC - kD_) * (iGR_D_ - iGR_C_) / sr_;
      }
      iGR_E_ += (iGR_D_ - iGR_E_) * kE;
      iGR_E_ = std::min(iGR_E_, iGR_D_);
      const float gr3 = 1.0f - iGR_E_;
      const float y4 = y3 * gr3;

      L = std::fabs(y4) + oneE20;
      float iGR_4 = std::max(1.0f - threshold / L, oneE20);
      const float gr4 = 1.0f - iGR_4;
      gr_ = gr1 * gr2 * gr3 * gr4;
      grDbOut = -gainToDb(std::max(gr_, 1.0e-6f));
      return y4 * gr4 * masterGain;
    }

    Filter attack_;
    float sr_ = 48000.0f;
    std::vector<float> delay_;
    int head_ = 0;
    float threshold_ = 0.5f;
    float masterGain_ = 1.0f;
    float iGR_A_ = 0.0f, iGR_B_ = 0.0f, iGR_X_ = 0.0f;
    float iGR_C_ = 0.0f, iGR_D_ = 0.0f, iGR_E_ = 0.0f;
    float kB_ = 1.0f, kD_ = 1.0f;
    float gr_ = 1.0f;
  };

  float sr_ = 48000.0f;
  int delaySize_ = 144;
  float kA_ = 0.999f, kX_ = 0.1f, kC_ = 0.99f, kE_ = 0.1f;
  float threshold_ = 0.5f;
  float masterGain_ = 1.0f;
  float outputGain_ = 1.0f;
  float mix_ = 1.0f;
  float lastGrDb_ = 0.0f;
  Channel ch_[2];
};

} // namespace hdsp::modules
