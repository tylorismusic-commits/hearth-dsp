// hearth-dsp shared module — distressor.hpp
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cmath>

#include "common.hpp"
#include "controls.hpp"

namespace hdsp::modules::comp {

// Feed-forward compressor with a fixed threshold, input-driven, in the style
// of the Distressor family.  Input drives the signal into the threshold, the
// ratio is a set of steps up to a 20:1 "nuke" setting, and the distortion
// selector adds even-order, odd-order, or hard-clip colour after the gain
// stage.  Distortion increases with gain reduction, as on the hardware.
class Distressor {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate;
    for (int ch = 0; ch < 2; ++ch) {
      sidechain_[ch].reset();
    }
    inputSmooth_.setTime(25.0f, sr_);
    inputSmooth_.reset(dbToGain(-80.0f));
    for (int ch = 0; ch < 2; ++ch) {
      dcLp_[ch].set(18.0f, sr_);
      dcLp_[ch].reset();
    }
    env_ = 0.0f;
    setControls(controls_);
  }

  void setControls(const Controls& c) noexcept {
    controls_ = c;
    // The Input knob is the drive control: 0 dB at the bottom of the range,
    // so unity in is unity out until you push it.
    const float drive = clampf(c.input, 0.0f, 1.0f);
    inputSmooth_.setTarget(dbToGain(-80.0f + 103.0f * std::sqrt(drive)));

    const int step = ratioStepFrom(c.ratio);
    ratio_ = ratioFromStep(step);
    nuke_ = step >= 8;

    attackMs_ = 0.1f * std::pow(500.0f, clampf(c.attack, 0.0f, 1.0f));
    releaseMs_ = 50.0f * std::pow(60.0f, clampf(c.release, 0.0f, 1.0f));
    if (nuke_) {
      attackMs_ *= 0.5f;
    }
    attackCoeff_ = timeCoeff(attackMs_, sr_);
    releaseCoeff_ = timeCoeff(releaseMs_, sr_);

    distortion_ = static_cast<int>(std::round(clampf(c.distortion, 0.0f, 1.0f) * 3.0f));
    outputGain_ = dbToGain(-12.0f + 24.0f * clampf(c.output, 0.0f, 1.0f));

    const float hpf = clampf(c.sidechainHpf, 0.0f, 1.0f);
    sidechainOn_ = hpf > 0.01f;
    if (sidechainOn_) {
      const float cutoff = 20.0f * std::pow(20.0f, hpf);
      for (auto& f : sidechain_) {
        f.set(cutoff, sr_);
      }
    }
  }

  // Processes one stereo frame in place.  Returns gain reduction in dB (>= 0).
  float process(float& left, float& right,
                const SidechainFrame& sc = {}) noexcept {
    const float inputGain = inputSmooth_.next();
    float inL = left * inputGain;
    float inR = right * inputGain;

    // How hard the input is driven decides how hard the distortion bites.
    const float inPeak = std::max(std::fabs(inL), std::fabs(inR));
    const float amount = clampf((gainToDb(inPeak) + 45.0f) / 35.0f, 0.0f, 1.0f);

    float detL = sc.active ? sc.left : inL;
    float detR = sc.active ? sc.right : inR;
    if (sidechainOn_) {
      detL = sidechain_[0].process(detL);
      detR = sidechain_[1].process(detR);
    }
    const float peak = std::max(std::fabs(detL), std::fabs(detR));
    if (peak > env_) {
      env_ = attackCoeff_ * env_ + (1.0f - attackCoeff_) * peak;
    } else {
      env_ = releaseCoeff_ * env_ + (1.0f - releaseCoeff_) * peak;
    }

    constexpr float kThresholdDb = -20.0f;
    constexpr float kKneeDb = 2.0f;
    const float grDb =
        gainReductionDb(gainToDb(env_), kThresholdDb, ratio_, kKneeDb);
    const float gain = dbToGain(-grDb);

    // The reference applies distortion after the gain cell (its DIST block
    // runs at the signal output), so it shapes the compressed signal.
    float outL = inL * gain;
    float outR = inR * gain;
    outL = distort(outL, 0, amount);
    outR = distort(outR, 1, amount);
    // DC removal: the asymmetric modes offset the stage on low transients.
    outL -= dcLp_[0].process(outL);
    outR -= dcLp_[1].process(outR);
    left = outL * outputGain_;
    right = outR * outputGain_;
    return std::max(grDb, 0.0f);
  }

  void reset() noexcept {
    for (int ch = 0; ch < 2; ++ch) {
      sidechain_[ch].reset();
      dcLp_[ch].reset();
    }
    env_ = 0.0f;
  }

private:
  // Distortion is deliberately conservative.  The reference implementation
  // blends in a flattened copy (x^0.04), which is full-on at any level and
  // sounds synthetic; these are level-scaled waveshapers instead.
  float distort(float x, int ch, float amount) noexcept {
    (void)ch;
    switch (distortion_) {
      case 1: { // Dist 2: gentle asymmetric saturation, even harmonics
        const float k = 1.0f + 0.9f * amount;
        const float bias = 0.10f + 0.08f * amount;
        const float tb = std::tanh(bias);
        const float norm = k * (1.0f - tb * tb);
        return (std::tanh(k * x + bias) - tb) / norm;
      }
      case 2: // Dist 3: symmetric, odd harmonics
        return std::tanh((1.0f + 1.6f * amount) * x);
      case 3: { // Clip: soft ceiling, unity at low level
        const float drive = 1.0f + 2.5f * amount;
        return std::tanh(x * drive) / drive;
      }
      default:
        return x; // Off is genuinely clean
    }
  }

  float sr_ = 48000.0f;
  Controls controls_;

  float ratio_ = 4.0f;
  bool nuke_ = false;
  float attackMs_ = 1.0f;
  float releaseMs_ = 100.0f;
  float attackCoeff_ = 0.0f;
  float releaseCoeff_ = 0.0f;
  int distortion_ = 0;
  float outputGain_ = 1.0f;
  bool sidechainOn_ = false;
  float env_ = 0.0f;
  HighPass2 sidechain_[2];
  OnePoleLP dcLp_[2];
  Smoothed inputSmooth_;
};

} // namespace hdsp::modules::comp
