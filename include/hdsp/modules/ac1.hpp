// hearth-dsp — JClones AC-1 console channel
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct AC1Controls {
  float input = 0.50f;     // 0..1 -> -24..+24 dB
  float output = 0.50f;    // 0..1 -> -24..+24 dB
  bool autoGain = false;
  float drive = 0.50f;     // 0..1 -> -12..+12 dB
  float comp = 0.11f;      // 0..1 -> 1..10
  float attack = 0.04f;    // 0..1 -> 0.1..10 ms
  float release = 0.57f;   // 0..1 -> 10..1000 ms
  bool stereoLink = true;
  int mode = 0;            // 0 AC1, 1 AC101
};

class AC1 {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    reset();
  }

  void setControls(const AC1Controls& c) noexcept {
    inputGain_ = dbToGain(-24.0f + 48.0f * clamp01(c.input));
    outputGain_ = dbToGain(-24.0f + 48.0f * clamp01(c.output));
    drive_ = dbToGain(-12.0f + 24.0f * clamp01(c.drive));
    const float curve = 1.0f + 9.0f * clamp01(c.comp);
    a_ = 0.75f - (curve - 1.0f) * 0.075f;
    if (c.autoGain) {
      outputGain_ *= std::min(
          (1.0f / (1.0f - (curve - 1.0f) * 0.075f)) / inputGain_, 16.0f);
    }
    const float attackSec = std::max(0.0001f, (0.1f + 9.9f * clamp01(c.attack)) * 0.001f);
    const float releaseSec = std::max(0.001f, (10.0f + 990.0f * clamp01(c.release)) * 0.001f);
    attackK_ = 1.0f / (sr_ * attackSec);
    releaseK_ = 1.0f / (sr_ * releaseSec);
    stereoLink_ = c.stereoLink;
    ac101_ = c.mode > 0;
  }

  inline void process(float& l, float& r) noexcept {
    const float levelL = std::fabs(l);
    const float levelR = std::fabs(r);
    const float level = stereoLink_ ? std::max(levelL, levelR) : 0.0f;
    float grL = 1.0f, grR = 1.0f;
    l = ch_[0].process(l, stereoLink_ ? level : levelL, inputGain_, drive_,
                       outputGain_, a_, ac101_, attackK_, releaseK_, grL);
    r = ch_[1].process(r, stereoLink_ ? level : levelR, inputGain_, drive_,
                       outputGain_, a_, ac101_, attackK_, releaseK_, grR);
    lastGrDb_ = -gainToDb(std::max(1.0e-6f, std::min(grL, grR)));
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
    float levelS = 0.0f;
    void reset() noexcept { levelS = 0.0f; }
    inline float process(float x, float xLevel, float inputGain, float drive,
                         float outputGain, float a, bool ac101,
                         float attackK, float releaseK, float& grOut) noexcept {
      const float filterK = xLevel > levelS ? attackK : releaseK;
      levelS += (xLevel - levelS) * filterK;
      float lvl = levelS * inputGain * drive;
      x *= inputGain;
      float gr = (lvl * lvl * 0.5f - 1.0f) * (lvl * lvl * 0.5f - 1.0f) + lvl * a;
      if (gr > 1.0f) gr = 1.0f;
      if (ac101 && lvl > 1.0f) {
        gr = 1.0f / (lvl + 0.00001f) * (a + 0.25f);
      }
      grOut = gr;
      return x * gr * outputGain;
    }
  };

  float sr_ = 48000.0f;
  float inputGain_ = 1.0f;
  float outputGain_ = 1.0f;
  float drive_ = 1.0f;
  float a_ = 0.75f;
  float attackK_ = 0.01f;
  float releaseK_ = 0.0001f;
  bool stereoLink_ = true;
  bool ac101_ = false;
  float lastGrDb_ = 0.0f;
  Channel ch_[2];
};

} // namespace hdsp::modules
