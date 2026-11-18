// hearth-dsp — JClones DaTube saturation
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct DaTubeControls {
  float drive = 0.30f;
  float balance = 1.00f;
  float output = 0.75f;  // 0..1 -> -24..0 dB
  bool bitReduction = false;
  bool foldBack = false;
};

class DaTube {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    dcK_ = 3.0f / sr_ * 2.0f * 3.14159265358979323846f;
    p_ = std::log(10.0f) + 1.0f;
    reset();
  }

  void setControls(const DaTubeControls& c) noexcept {
    drive_ = clamp01(c.drive);
    drive05_ = drive_ * 0.5f;
    wet_ = clamp01(c.balance);
    outputGain_ = dbToGain(-24.0f + 24.0f * clamp01(c.output)) * 2.0f;
    bit_ = c.bitReduction;
    fold_ = c.foldBack;
  }

  inline void process(float& l, float& r) noexcept {
    l = ch_[0].process(l, drive_, drive05_, wet_, outputGain_, bit_, fold_, p_,
                       dcK_);
    r = ch_[1].process(r, drive_, drive05_, wet_, outputGain_, bit_, fold_, p_,
                       dcK_);
  }

  void reset() noexcept {
    for (auto& ch : ch_) ch.reset();
  }

  int latencySamples() const noexcept { return 0; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  struct Channel {
    float dcS = 0.0f;
    void reset() noexcept { dcS = 0.0f; }
    inline float process(float x, float drive, float drive05, float wet,
                         float outputGain, bool bit, bool fold,
                         float p, float dcK) noexcept {
      const float dry = x;
      x *= drive + 0.5f;
      if (bit) {
        x *= 2048.0f;
        x = x >= 0.0f ? std::floor(x) : std::ceil(x);
        x /= 2048.0f;
      }
      if (fold && std::fabs(x) > 1.0f) {
        x = x >= 0.0f ? 2.0f - x : -2.0f - x;
      }
      float y;
      if (x > 1.0f) {
        y = (1.0f - drive05) * x + drive05;
      } else if (x > 0.0f) {
        y = ((1.0f - x) - std::pow(1.0f - x, p)) * drive05 + x;
      } else if (x > -1.0f) {
        y = (std::pow(1.0f + x, p) - (x + 1.0f)) * drive + x;
      } else {
        y = (1.0f - drive) * x - drive;
      }
      if (!bit) {
        y -= (dcS += dcK * (y - dcS));
      }
      y = y * wet + dry * (1.0f - wet);
      return y * outputGain;
    }
  };

  float sr_ = 48000.0f;
  float drive_ = 0.3f;
  float drive05_ = 0.15f;
  float wet_ = 1.0f;
  float outputGain_ = 1.0f;
  float dcK_ = 0.0004f;
  float p_ = 3.302585f;
  bool bit_ = false;
  bool fold_ = false;
  Channel ch_[2];
};

} // namespace hdsp::modules
