// hearth-dsp shared module — ember_strip.hpp
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cstdint>

#include "compressor.hpp"
#include "eq.hpp"
#include "macro.hpp"
#include "params.hpp"
#include "saturator.hpp"
#include "smoother.hpp"
#include "util.hpp"

namespace hdsp::ember {

// Ember's signal flow:
//
//   input -> low shelf -> presence bell -> air shelf
//         -> mastering glue -> harmonic stage -> automatic trim
//         -> hidden soft clipper -> master bypass crossfade
//
// Every stage is neutral at Amount = 0, so the plugin is bit-for-bit dry when
// the knob is all the way down (apart from denormal flushing).
class EmberStrip {
public:
  void prepare(float sampleRate) noexcept {
    sampleRate_ = sampleRate;

    eq_.prepare(sampleRate);
    compressor_.prepare(sampleRate);
    harmonics_.prepare(sampleRate);

    outputGain_.setTime(24.0f, sampleRate);
    enableAmt_.setTime(10.0f, sampleRate);
    outputGain_.reset(1.0f);
    enableAmt_.reset(1.0f);

    counter_ = 0;
  }

  void setTargets(const Params& p) noexcept {
    params_ = p;
    const MacroSettings m = macroFor(p.amount);

    eq_.setTargets(m.lowFreq, m.lowGainDb, m.presenceFreq, m.presenceGainDb,
                   m.presenceQ, m.airFreq, m.airGainDb);
    compressor_.setTargets(m.compThresholdDb, m.compRatio, m.compAttackMs,
                           m.compReleaseMs, m.compKneeDb, m.compMakeupDb,
                           m.compMix, m.compOn);
    harmonics_.setTargets(m.satDrive, m.satBias, m.satMix);
    outputGain_.setTarget(dbToGain(m.outputGainDb));
    enableAmt_.setTarget(p.enabled ? 1.0f : 0.0f);
  }

  void reset() noexcept {
    eq_.reset();
    compressor_.reset();
    harmonics_.reset();
    outputGain_.snap();
    enableAmt_.snap();
    counter_ = 0;
  }

  // Processes `frames` samples from the input buffers into the output buffers
  // (which may alias the inputs).  Returns the compressor gain reduction
  // (<= 0 dB) and the peak output level in dBFS (clamped for display).
  void process(const float* inLeft, const float* inRight, float* outLeft,
               float* outRight, uint32_t frames, float& grDb,
               float& outDb) noexcept {
    float lastGr = 0.0f;
    float peak = 0.0f;

    for (uint32_t i = 0; i < frames; ++i) {
      const float outputGain = outputGain_.next();
      const float enable = enableAmt_.next();

      eq_.tick();
      compressor_.tick();
      harmonics_.tick();

      if ((counter_ & 15u) == 0u) {
        eq_.update();
        compressor_.updateCoeffs();
      }

      const float dryL = inLeft[i];
      const float dryR = inRight[i];

      float wetL = eq_.process(0, dryL);
      float wetR = eq_.process(1, dryR);

      lastGr = compressor_.process(wetL, wetR);

      wetL = harmonics_.process(0, wetL);
      wetR = harmonics_.process(1, wetR);

      const float faderL = wetL * outputGain;
      const float faderR = wetR * outputGain;

      // Hidden output protection: linear below the knee, a smooth tanh
      // shoulder above it, and a hard ceiling at +0.2 dBFS.
      const float clippedL = softClip(faderL);
      const float clippedR = softClip(faderR);

      const float finalL = dryL + enable * (clippedL - dryL);
      const float finalR = dryR + enable * (clippedR - dryR);

      outLeft[i] = finalL;
      outRight[i] = finalR;

      peak = std::max(peak, std::max(std::fabs(finalL), std::fabs(finalR)));
      ++counter_;
    }

    grDb = lastGr;
    outDb = clamp(gainToDb(peak), -60.0f, 6.0f);
  }

private:
  float sampleRate_ = 48000.0f;
  Params params_;
  EqSection eq_;
  MasterCompressor compressor_;
  HarmonicStage harmonics_;
  Smoothed outputGain_;
  Smoothed enableAmt_;
  uint32_t counter_ = 0;

  // +0.2 dBFS: 10^(0.2/20)
  static constexpr float kOutputClipLevel = 1.02329299f;
  static constexpr float kOutputClipKnee = 0.90f * kOutputClipLevel;

  static inline float softClip(float x) noexcept {
    const float ax = std::fabs(x);
    if (ax <= kOutputClipKnee) {
      return x;
    }
    const float over = (ax - kOutputClipKnee) /
                       (kOutputClipLevel - kOutputClipKnee);
    const float shaped =
        kOutputClipKnee + (kOutputClipLevel - kOutputClipKnee) *
                              std::tanh(over);
    return x < 0.0f ? -shaped : shaped;
  }
};

} // namespace hdsp::ember
