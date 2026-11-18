// hearth-dsp — JClones O3 maximizer
// SPDX-License-Identifier: GPL-3.0-or-later
//
// C++ translation of the MIT-licensed JClones O3 Maximizer JSFX. It uses 5x
// oversampled inter-sample peak detection, max-hold, a windowed attack FIR and
// adaptive release. The limiter threshold is fixed at 0 dBFS; the front-panel
// threshold control is input drive and margin is the output ceiling.
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct O3Controls {
  float threshold = 1.0f;  // 0..1 -> -20..0 dB input drive
  float margin = 1.0f;     // 0..1 -> -6..0 dB output ceiling
  float character = 0.4f;  // 0..1 -> 0..10
  bool preventIsp = false;
};

class O3 {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    const int maxLookahead =
        sr_ <= 50000.0f ? 32 : (sr_ <= 130000.0f ? 64 : 128);
    maxHoldLen_ = maxLookahead * 2 + 1;
    attackLen_ = maxHoldLen_;
    autoReleaseLen_ = std::max(1, static_cast<int>(std::floor(sr_ * 0.014f)));

    upBufL_.assign(static_cast<size_t>(2 * kUpsampleLen), 0.0f);
    upBufR_.assign(static_cast<size_t>(2 * kUpsampleLen), 0.0f);
    maxHoldBuf_.assign(static_cast<size_t>(2 * maxHoldLen_), 0.0f);
    attackBuf_.assign(static_cast<size_t>(2 * attackLen_), 0.0f);
    attackIr_.assign(static_cast<size_t>(attackLen_), 0.0f);
    autoReleaseBuf_.assign(static_cast<size_t>(autoReleaseLen_), 0.0f);
    delayGr1_.init(autoReleaseLen_);
    delayGr2_.init(autoReleaseLen_);
    const int delaySize = kUpsampleLen / 2 + maxHoldLen_ - 1 + autoReleaseLen_;
    delayL_.init(delaySize);
    delayR_.init(delaySize);
    maxHoldDelay_.init(1);

    reset();
    dirty_ = true;
  }

  void setControls(const O3Controls& c) noexcept {
    threshold_ = clamp01(c.threshold);
    margin_ = clamp01(c.margin);
    character_ = clamp01(c.character);
    preventIsp_ = c.preventIsp;
    dirty_ = true;
  }

  inline void process(float& l, float& r) noexcept {
    updateParameters();
    l *= inputGain_;
    r *= inputGain_;

    const float level = detectLevel(l, r);
    const float levelDb =
        level < kMinLevelLin ? kMinLevelDb : gainToDb(std::max(level, 1.0e-12f));
    const float rawGrDb = std::max(levelDb - 0.0f, 0.0f);

    const float maxHoldGr = applyMaxHold(rawGrDb);
    const float grWithRelease = applyRelease(rawGrDb);
    float grWithAuto;
    if (attackCoeff_ > 0.04f) {
      grWithAuto = applyAutoRelease(rawGrDb, maxHoldGr, grWithRelease);
    } else {
      grWithAuto = delayGr1_.push(maxHoldGr);
    }
    grWithAuto = maxHoldDelay_.push(grWithAuto);
    const float finalGrDb = applyAttack(grWithAuto);
    const float outGr = dbToGain(-finalGrDb);
    lastGrDb_ = -gainToDb(std::max(outGr, 1.0e-6f));

    l = delayL_.push(l) * outGr * outputGain_;
    r = delayR_.push(r) * outGr * outputGain_;
  }

  void reset() noexcept {
    upHead_ = 0;
    std::fill(upBufL_.begin(), upBufL_.end(), 0.0f);
    std::fill(upBufR_.begin(), upBufR_.end(), 0.0f);
    maxHoldHead_ = 0;
    std::fill(maxHoldBuf_.begin(), maxHoldBuf_.end(), 0.0f);
    maxHoldDelay_.reset();
    attackHead_ = 0;
    std::fill(attackBuf_.begin(), attackBuf_.end(), 0.0f);
    autoReleaseHead_ = 0;
    std::fill(autoReleaseBuf_.begin(), autoReleaseBuf_.end(), 0.0f);
    autoReleaseSum_ = 0.0f;
    autoReleaseState1_ = 0.0f;
    autoReleaseState2_ = 0.0f;
    delayGr1_.reset();
    delayGr2_.reset();
    delayL_.reset();
    delayR_.reset();
    releaseState_ = 0.0f;
    lastRawGr_ = 0.0f;
    lastGrDb_ = 0.0f;
  }

  int latencySamples() const noexcept { return delayL_.size(); }
  float lastGrDb() const noexcept { return lastGrDb_; }

private:
  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr float kMinLevelDb = -200.0f;
  static constexpr float kMinLevelLin = 1.0e-10f;
  static constexpr int kUpsampleLen = 14;

  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  struct DelayLine {
    std::vector<float> line;
    int head = 0;
    int size() const noexcept { return static_cast<int>(line.size()); }
    void init(int n) noexcept {
      line.assign(static_cast<size_t>(std::max(0, n)), 0.0f);
      head = 0;
    }
    void reset() noexcept {
      std::fill(line.begin(), line.end(), 0.0f);
      head = 0;
    }
    float push(float x) noexcept {
      if (line.empty()) return x;
      const float y = line[static_cast<size_t>(head)];
      line[static_cast<size_t>(head)] = x;
      head = (head + 1) % static_cast<int>(line.size());
      return y;
    }
  };

  void updateParameters() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    inputGain_ = dbToGain(20.0f - 20.0f * threshold_);
    outputGain_ = dbToGain(-6.0f + 6.0f * margin_);
    setAttack(character_ * 1.5f);
  }

  float detectLevel(float inL, float inR) noexcept {
    if (upHead_ == 0) {
      std::copy(upBufL_.begin(), upBufL_.begin() + kUpsampleLen,
                upBufL_.begin() + kUpsampleLen);
      std::copy(upBufR_.begin(), upBufR_.begin() + kUpsampleLen,
                upBufR_.begin() + kUpsampleLen);
      upHead_ = kUpsampleLen;
    }
    --upHead_;
    upBufL_[static_cast<size_t>(upHead_)] = inL;
    upBufR_[static_cast<size_t>(upHead_)] = inR;

    const float y0L = upBufL_[static_cast<size_t>(upHead_ + kUpsampleLen / 2)];
    const float y0R = upBufR_[static_cast<size_t>(upHead_ + kUpsampleLen / 2)];
    float maxValue = std::max(std::fabs(y0L), std::fabs(y0R));

    if (preventIsp_) {
      float sumL[5] = {0, 0, 0, 0, 0};
      float sumR[5] = {0, 0, 0, 0, 0};
      int j = 0;
      for (int i = 0; i < kUpsampleLen; ++i) {
        const float xl = upBufL_[static_cast<size_t>(upHead_ + i)];
        const float xr = upBufR_[static_cast<size_t>(upHead_ + i)];
        for (int p = 0; p < 5; ++p) {
          const float ir = kUpsampleIr[j + p];
          sumL[p] += xl * ir;
          sumR[p] += xr * ir;
        }
        j += 5;
      }
      for (int p = 0; p < 5; ++p) {
        maxValue = std::max(maxValue,
                            std::max(std::fabs(sumL[p]), std::fabs(sumR[p])));
      }
    }
    return maxValue;
  }

  float applyMaxHold(float input) noexcept {
    if (maxHoldHead_ == 0) {
      std::copy(maxHoldBuf_.begin(), maxHoldBuf_.begin() + maxHoldLen_,
                maxHoldBuf_.begin() + maxHoldLen_);
      maxHoldHead_ = maxHoldLen_;
    }
    --maxHoldHead_;
    maxHoldBuf_[static_cast<size_t>(maxHoldHead_)] = input;
    float maxValue = input;
    for (int i = 1; i < maxHoldSamples_; ++i) {
      maxValue = std::max(
          maxValue, maxHoldBuf_[static_cast<size_t>(maxHoldHead_ + i)]);
    }
    return maxValue;
  }

  float applyRelease(float input) noexcept {
    releaseState_ *= releaseFilterK_;
    if (input > releaseState_) releaseState_ = input;
    return releaseState_;
  }

  float applyAutoRelease(float rawGrDb, float maxHoldGrDb,
                         float grWithRelease) noexcept {
    const float rawDelta = std::fabs(rawGrDb - lastRawGr_);
    lastRawGr_ = rawGrDb;

    autoReleaseSum_ -= autoReleaseBuf_[static_cast<size_t>(autoReleaseHead_)];
    autoReleaseBuf_[static_cast<size_t>(autoReleaseHead_)] = rawDelta;
    autoReleaseSum_ += rawDelta;
    autoReleaseHead_ = (autoReleaseHead_ + 1) % autoReleaseLen_;

    float s1 = autoReleaseK1_ * autoReleaseSum_ + autoReleaseK2_;
    s1 = std::min(s1, 1.0f);
    const float s2 = s1 * autoReleaseK3_ * autoReleaseK4_ +
                     (1.0f - autoReleaseK4_) * autoReleaseState1_;
    if (s2 < autoReleaseState1_) {
      autoReleaseState1_ += (s2 - autoReleaseState1_) * 2.0f;
    } else {
      autoReleaseState1_ += (s2 - autoReleaseState1_) * 0.3f;
    }
    autoReleaseState1_ = std::min(std::max(autoReleaseState1_, 0.0f), 1.0f);

    maxHoldGrDb = delayGr1_.push(maxHoldGrDb);
    grWithRelease = delayGr2_.push(grWithRelease);

    autoReleaseState2_ = grWithRelease * grWithRelease * autoReleaseK5_ +
                         (1.0f - autoReleaseK5_) * autoReleaseState2_;
    const float grOutput =
        std::sqrt(std::max(autoReleaseState2_, 0.0f)) * autoReleaseState1_;
    return std::max(maxHoldGrDb, grOutput);
  }

  float applyAttack(float input) noexcept {
    if (attackHead_ == 0) {
      std::copy(attackBuf_.begin(), attackBuf_.begin() + attackLen_,
                attackBuf_.begin() + attackLen_);
      attackHead_ = attackLen_;
    }
    --attackHead_;
    attackBuf_[static_cast<size_t>(attackHead_)] = input;
    float y = 0.0f;
    for (int i = 0; i < attackLen_; ++i) {
      y += attackBuf_[static_cast<size_t>(attackHead_ + i)] *
           attackIr_[static_cast<size_t>(i)];
    }
    return y;
  }

  void setAttack(float coeff) noexcept {
    attackCoeff_ = coeff;
    int real = attackLen_;
    if (coeff < 0.3f) {
      real = static_cast<int>(
          std::floor(std::sqrt(coeff) * coeff / (std::sqrt(0.3f) * 0.3f) *
                         attackLen_ +
                     0.5f));
      if ((real & 1) == 0) ++real;
      real = std::max(real, 1);
    }

    float sum = 0.0f;
    for (int i = 0; i < attackLen_; ++i) {
      const float x = static_cast<float>(i) -
                      static_cast<float>(attackLen_ - 1) * 0.5f;
      float v = 1.0f - std::cos(
                           (x / static_cast<float>(real) * 2.0f + 1.0f) * kPi);
      if (i < (attackLen_ - real) / 2 || i >= (attackLen_ + real) / 2) v = 0.0f;
      attackIr_[static_cast<size_t>(i)] = v;
      sum += v;
    }
    if (sum != 0.0f) {
      const float norm = 1.0f / sum;
      for (auto& v : attackIr_) v *= norm;
    }
    maxHoldSamples_ = real;

    const int delaySize = (attackLen_ - 1) / 2 - (real - 1) / 2;
    if (maxHoldDelay_.size() != delaySize) maxHoldDelay_.init(delaySize);

    float releaseX1 = coeff;
    if (releaseX1 > 1.0f) releaseX1 = 2.0f * releaseX1 - 1.0f;
    const float releaseX2 = releaseX1 * 2.0f;
    releaseFilterK_ =
        1.0f - sr_ / (std::exp(releaseX2) * 35.0f) * (0.1f / 44100.0f);

    autoReleaseK1_ = std::exp(releaseX2) * 0.018181818f;
    autoReleaseK2_ = 0.0f;
    if (releaseX1 > 0.5f) {
      autoReleaseK2_ = (releaseX1 - 0.5f) * 0.7f;
      if (autoReleaseK2_ > 0.9f) autoReleaseK2_ = 0.9f;
    }
    autoReleaseK3_ = 1.0f;
    if (coeff < 0.3f) autoReleaseK3_ = coeff * coeff * 11.111111f;
    autoReleaseK4_ = 44.1f / sr_;
    autoReleaseK5_ = autoReleaseK4_;
    if (releaseX1 > 0.7f) {
      autoReleaseK5_ += (releaseX1 - 0.7f) * (releaseX1 - 0.7f);
    }
    autoReleaseK5_ = std::min(autoReleaseK5_, 1.0f);
  }

  float sr_ = 48000.0f;
  int maxHoldLen_ = 65, attackLen_ = 65, autoReleaseLen_ = 672;
  int maxHoldSamples_ = 65;
  float threshold_ = 1.0f, margin_ = 1.0f, character_ = 0.4f;
  bool preventIsp_ = false;
  bool dirty_ = true;
  float inputGain_ = 1.0f, outputGain_ = 1.0f;

  float attackCoeff_ = 0.0f;
  float releaseFilterK_ = 0.99f, releaseState_ = 0.0f;
  float autoReleaseSum_ = 0.0f, autoReleaseState1_ = 0.0f,
        autoReleaseState2_ = 0.0f, lastRawGr_ = 0.0f;
  float autoReleaseK1_ = 0.0f, autoReleaseK2_ = 0.0f, autoReleaseK3_ = 1.0f,
        autoReleaseK4_ = 1.0f, autoReleaseK5_ = 1.0f;
  float lastGrDb_ = 0.0f;

  int upHead_ = 0, maxHoldHead_ = 0, attackHead_ = 0, autoReleaseHead_ = 0;
  std::vector<float> upBufL_, upBufR_;
  std::vector<float> maxHoldBuf_;
  std::vector<float> attackBuf_;
  std::vector<float> attackIr_;
  std::vector<float> autoReleaseBuf_;
  DelayLine delayGr1_, delayGr2_, maxHoldDelay_, delayL_, delayR_;

  static constexpr float kUpsampleIr[70] = {
      0.0f,        0.0f,        0.0f,        0.0f,
      0.001108887f, 0.0f,        -0.002307397f, -0.005021744f,
      -0.006557025f, -0.005171423f, 0.0f,      0.007996489f,
      0.015768220f, 0.019012450f, 0.014039180f, 0.0f,
      -0.019598060f, -0.037128130f, -0.043264330f, -0.031036880f,
      0.0f,        0.041477940f, 0.077415300f, 0.089296040f,
      0.063735860f, 0.0f,        -0.085880880f, -0.162850400f,
      -0.192896500f, -0.143438300f, 0.0f,      0.225520200f,
      0.494558500f, 0.750534300f, 0.934209700f, 1.000956000f,
      0.934209700f, 0.750534300f, 0.494558500f, 0.225520200f,
      0.0f,        -0.143438300f, -0.192896500f, -0.162850400f,
      -0.085880880f, 0.0f,       0.063735860f, 0.089296040f,
      0.077415300f, 0.041477940f, 0.0f,       -0.031036880f,
      -0.043264330f, -0.037128130f, -0.019598060f, 0.0f,
      0.014039180f, 0.019012450f, 0.015768220f, 0.007996489f,
      0.0f,       -0.005171423f, -0.006557025f, -0.005021744f,
      -0.002307397f, 0.0f,      0.001108887f, 0.0f,
      0.0f,        0.0f};
};

} // namespace hdsp::modules
