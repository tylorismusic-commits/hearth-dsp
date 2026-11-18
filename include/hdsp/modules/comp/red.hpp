// hearth-dsp shared module — red.hpp
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include "biquad.hpp"
#include "common.hpp"
#include "controls.hpp"

namespace hdsp::modules::comp {

// British Class-A VCA compressor in the Focusrite Red family.
//
// The defining behaviour is the gain structure: compression followed by
// limiting, rather than a compressor that gradually turns into a limiter.  The
// hardware derives two control voltages from a three-VCA side-chain; here the
// first stage is the soft-knee compressor and the second is a fast peak
// limiter acting on the already-compressed signal.
//
// Detection is peak, feed-forward, as on the hardware.  The transformer-coupled
// I/O is modelled with a small asymmetric saturation and a low-frequency
// high-pass, kept subtle: this is a transparent compressor, not a colour box.
class RedCompressor {
public:
  // The Red's ratio ladder, snapped to the nearest value when the shared
  // ladder arrives from another model.
  // The Red has six ratios.  The control is six positions; a value arriving
  // from another model's nine-step ladder clamps to the nearest end.
  static float ratioForStep(int step) noexcept {
    static const float kRed[6] = {1.5f, 2.0f, 3.0f, 5.0f, 7.0f, 10.0f};
    return kRed[std::min(std::max(step, 0), 5)];
  }

  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    inputSmooth_.setTime(15.0f, sr_);
    outputSmooth_.setTime(15.0f, sr_);
    makeupSmooth_.setTime(15.0f, sr_);
    for (auto& f : transformerHp_) {
      f.highPass(15.0f, 0.707f, sr_);
    }
    for (auto& s : shaper_) {
      s.reset();
    }
    reset();
    setControls(controls_);
    inputSmooth_.snap();
    outputSmooth_.snap();
    makeupSmooth_.snap();
  }

  void setControls(const Controls& c) noexcept {
    controls_ = c;
    inputSmooth_.setTarget(
        dbToGain(-18.0f + 36.0f * clampf(c.input, 0.0f, 1.0f)));
    thresholdDb_ = -40.0f + 40.0f * clampf(c.threshold, 0.0f, 1.0f);
    ratio_ = ratioForStep(ratioStepFrom(c.ratio));
    kneeDb_ = 12.0f * clampf(c.knee, 0.0f, 1.0f);
    attackCoeff_ =
        timeCoeff(0.3f * std::pow(300.0f, clampf(c.attack, 0.0f, 1.0f)), sr_);
    releaseMs_ = 100.0f * std::pow(40.0f, clampf(c.release, 0.0f, 1.0f));
    releaseCoeff_ = timeCoeff(releaseMs_, sr_);
    limiterAttackCoeff_ = timeCoeff(0.05f, sr_);
    // Bipolar: 0.5 is unity, matching the other models.
    makeupSmooth_.setTarget(
        dbToGain(20.0f * (clampf(c.makeup, 0.0f, 1.0f) - 0.5f)));
    outputSmooth_.setTarget(
        dbToGain(-12.0f + 24.0f * clampf(c.output, 0.0f, 1.0f)));
    limiterOn_ = c.limiterOn;
    limitDb_ = -24.0f + 24.0f * clampf(c.limitThreshold, 0.0f, 1.0f);
    autoRelease_ = c.vcaAutoRelease;
    sidechainOn_ = clampf(c.sidechainHpf, 0.0f, 1.0f) > 0.01f;
    if (sidechainOn_) {
      const float cutoff = 20.0f * std::pow(20.0f, clampf(c.sidechainHpf, 0.0f, 1.0f));
      for (auto& f : sidechain_) {
        f.set(cutoff, sr_);
      }
    }
  }

  // Returns total gain reduction in dB (>= 0).
  float process(float& left, float& right,
                const SidechainFrame& sc = {}) noexcept {
    const float inputGain = inputSmooth_.next();
    const float inL = left * inputGain;
    const float inR = right * inputGain;

    float detL = sc.active ? sc.left : inL;
    float detR = sc.active ? sc.right : inR;
    if (sidechainOn_) {
      detL = sidechain_[0].process(detL);
      detR = sidechain_[1].process(detR);
    }

    // Peak detection, stereo linked.
    const float level = std::max(std::fabs(detL), std::fabs(detR));
    if (level > env_) {
      env_ += (level - env_) * (1.0f - attackCoeff_);
    } else {
      env_ += (level - env_) * (1.0f - releaseCoeffFor(env_, thresholdDb_));
    }
    env_ = std::max(env_, 1e-9f);

    const float grDb =
        gainReductionDb(gainToDb(env_), thresholdDb_, ratio_, kneeDb_);
    const float compGain = dbToGain(-grDb);

    float yL = inL * compGain;
    float yR = inR * compGain;

    // Second stage: fast peak limiting on the compressed signal.
    float limDb = 0.0f;
    if (limiterOn_) {
      const float compressedPeak = std::max(std::fabs(yL), std::fabs(yR));
      if (compressedPeak > limiterEnv_) {
        limiterEnv_ += (compressedPeak - limiterEnv_) *
                       (1.0f - limiterAttackCoeff_);
      } else {
        limiterEnv_ += (compressedPeak - limiterEnv_) *
                       (1.0f - releaseCoeffFor(limiterEnv_, limitDb_));
      }
      limiterEnv_ = std::max(limiterEnv_, 1e-9f);
      const float overDb = gainToDb(limiterEnv_) - limitDb_;
      limDb = std::max(overDb, 0.0f);
      const float limGain = dbToGain(-limDb);
      yL *= limGain;
      yR *= limGain;
    }

    const float makeup = makeupSmooth_.next();
    const float output = outputSmooth_.next();
    const float trim = makeup * output;

    // Transformer-coupled I/O: a touch of asymmetry and no DC.
    yL = transformerHp_[0].process(shaper_[0].process(yL * trim, 1.05f, 0.03f));
    yR = transformerHp_[1].process(shaper_[1].process(yR * trim, 1.05f, 0.03f));

    left = yL;
    right = yR;
    return grDb + limDb;
  }

  void reset() noexcept {
    env_ = 0.0f;
    limiterEnv_ = 0.0f;
    for (auto& f : sidechain_) {
      f.reset();
    }
    for (auto& f : transformerHp_) {
      f.reset();
    }
    for (auto& s : shaper_) {
      s.reset();
    }
  }

private:
  // Program-dependent release: the further the detector sits over the stage's
  // threshold, the slower it lets go.  Using the absolute level here made a
  // quiet signal release slowest of all.
  float releaseCoeffFor(float level, float thresholdDb) const noexcept {
    if (!autoRelease_) {
      return releaseCoeff_;
    }
    const float over =
        clampf((gainToDb(level) - thresholdDb) / 20.0f, 0.0f, 1.0f);
    return timeCoeff(releaseMs_ * (1.0f + 3.0f * over), sr_);
  }

  float sr_ = 48000.0f;
  Controls controls_;
  float thresholdDb_ = -20.0f;
  float ratio_ = 4.0f;
  float kneeDb_ = 6.0f;
  float limitDb_ = 0.0f;
  float releaseMs_ = 300.0f;
  float attackCoeff_ = 0.0f;
  float releaseCoeff_ = 0.0f;
  float limiterAttackCoeff_ = 0.0f;
  bool limiterOn_ = false;
  bool autoRelease_ = false;
  bool sidechainOn_ = false;
  float env_ = 0.0f;
  float limiterEnv_ = 0.0f;
  Smoothed inputSmooth_;
  Smoothed outputSmooth_;
  Smoothed makeupSmooth_;
  HighPass2 sidechain_[2];
  Biquad transformerHp_[2];
  AsymShaper shaper_[2];
};

} // namespace hdsp::modules::comp
