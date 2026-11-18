// hearth-dsp shared module — clean.hpp
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include "common.hpp"
#include "controls.hpp"

namespace hdsp::modules::comp {

// Clean digital compressor, in the spirit of a workstation's "Platinum
// Digital" mode: no transformer, no valve, no program dependence.  A
// stereo-linked RMS detector, a soft-knee gain computer and a smooth
// attack/release on the gain.  The optional distortion stage is the only
// deliberate colour, and it is off by default.
class CleanCompressor {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    rmsCoeff_ = timeCoeff(5.0f, sr_);
    inputSmooth_.setTime(15.0f, sr_);
    outputSmooth_.setTime(15.0f, sr_);
    makeupSmooth_.setTime(15.0f, sr_);
    for (auto& f : sidechain_) {
      f.reset();
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
    inputSmooth_.setTarget(dbToGain(-30.0f + 60.0f * clampf(c.input, 0.0f, 1.0f)));
    // Same -40..0 dB range as the VCA and Molot, so one knob position
    // means one threshold across the models that have a threshold.
    thresholdDb_ = -40.0f + 40.0f * clampf(c.threshold, 0.0f, 1.0f);
    ratio_ = ratioFromStep(ratioStepFrom(c.ratio));
    // Knee runs 0..18 dB, as a fraction of the control's travel.
    kneeDb_ = 18.0f * clampf(c.knee, 0.0f, 1.0f);
    attackCoeff_ =
        timeCoeff(0.1f * std::pow(2000.0f, clampf(c.attack, 0.0f, 1.0f)), sr_);
    releaseCoeff_ =
        timeCoeff(5.0f * std::pow(1000.0f, clampf(c.release, 0.0f, 1.0f)), sr_);
    // Bipolar: 0.5 is unity, matching the other models.
    makeupSmooth_.setTarget(
        dbToGain(20.0f * (clampf(c.makeup, 0.0f, 1.0f) - 0.5f)));
    outputSmooth_.setTarget(dbToGain(
        -30.0f + 60.0f * clampf(c.output, 0.0f, 1.0f)));
    distortion_ = c.distortion;

    const float hpf = clampf(c.sidechainHpf, 0.0f, 1.0f);
    sidechainOn_ = hpf > 0.01f;
    if (sidechainOn_) {
      const float cutoff = 20.0f * std::pow(20.0f, hpf);
      for (auto& f : sidechain_) {
        f.set(cutoff, sr_);
      }
    }
  }

  // Returns gain reduction in dB (>= 0).
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

    // Stereo-linked RMS.  Mean power of the two channels, so a hard-panned
    // signal reads 3 dB low rather than full scale on one side only.
    const float power = 0.5f * (detL * detL + detR * detR);
    // timeCoeff() returns the retention factor: 1 - coeff is the update weight.
    rms_ += (power - rms_) * (1.0f - rmsCoeff_);
    const float levelDb = gainToDb(std::sqrt(std::max(rms_, 1e-20f)));

    const float target =
        gainReductionDb(levelDb, thresholdDb_, ratio_, kneeDb_);
    if (target > gr_) {
      gr_ += (target - gr_) * (1.0f - attackCoeff_);
    } else {
      gr_ += (target - gr_) * (1.0f - releaseCoeff_);
    }
    gr_ = std::max(gr_, 0.0f);

    const float makeup = makeupSmooth_.next();
    const float output = outputSmooth_.next();
    const float gain = dbToGain(-gr_) * makeup;

    // The colour stage is deliberately last in the whole chain, after the
    // output limiter, so it is applied by postColour() rather than here.
    left = inL * gain * output;
    right = inR * gain * output;
    return gr_;
  }

  // Applied by the engine after the gain cell, before the model calibration
  // trim and the output limiter.  No-op when the colour mode is Off.
  void postColour(float& left, float& right) noexcept {
    left = colour(left, 0);
    right = colour(right, 1);
  }

  void reset() noexcept {
    for (auto& f : sidechain_) {
      f.reset();
    }
    for (auto& s : shaper_) {
      s.reset();
    }
    rms_ = 0.0f;
    gr_ = 0.0f;
  }

private:
  // The optional distortion is the model's only colour.  Off is bit-clean,
  // and the two shaped modes are deliberately gentle: the shaper is
  // normalised to unity slope, so at these drive values a full-scale peak
  // loses well under a decibel and what is left is a little harmonic shading.
  float colour(float x, int ch) noexcept {
    switch (static_cast<int>(distortion_ * 3.0f + 0.5f)) {
      case 1:  // Soft
        return shaper_[ch].process(x, 0.35f, 0.0f);
      case 2:  // Hard
        return shaper_[ch].process(x, 0.75f, 0.0f);
      case 3: {  // Clip: a soft ceiling, inaudible below it
        constexpr float kCeiling = 0.85f;
        const float a = std::fabs(x);
        if (a <= kCeiling) {
          return x;
        }
        const float y =
            kCeiling + (1.0f - kCeiling) *
                           std::tanh((a - kCeiling) / (1.0f - kCeiling));
        return x < 0.0f ? -y : y;
      }
      default:
        return x;
    }
  }

  float sr_ = 48000.0f;
  Controls controls_;
  float rmsCoeff_ = 0.0f;
  float thresholdDb_ = -30.0f;
  float ratio_ = 4.0f;
  float kneeDb_ = 9.0f;
  float attackCoeff_ = 0.0f;
  float releaseCoeff_ = 0.0f;
  float distortion_ = 0.0f;
  bool sidechainOn_ = false;
  float rms_ = 0.0f;
  float gr_ = 0.0f;
  Smoothed inputSmooth_;
  Smoothed outputSmooth_;
  Smoothed makeupSmooth_;
  HighPass2 sidechain_[2];
  AsymShaper shaper_[2];
};

} // namespace hdsp::modules::comp
