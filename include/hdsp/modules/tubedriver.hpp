// hearth-dsp — JClones Tube Driver saturation
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct TubeDriverControls {
  float input = 0.50f;   // 0..1 -> -12..+12 dB
  float drive = 0.20f;   // 0..1
  float bias = 0.50f;    // 0..1
  bool boost = false;
  float output = 0.50f;  // 0..1 -> -12..+12 dB
};

class TubeDriver {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    dcK_ = 5.0f / sr_ * 2.0f * 3.14159265358979323846f;
    reset();
  }

  void setControls(const TubeDriverControls& c) noexcept {
    inputGain_ = dbToGain(-12.0f + 24.0f * clamp01(c.input));
    outputGain_ = dbToGain(-12.0f + 24.0f * clamp01(c.output));
    const float drive = clamp01(c.drive);
    const float bias = clamp01(c.bias);
    drive_ = drive * 5.0f * (c.boost ? 5.0f : 1.0f) + 0.5f;
    bias_ = bias * bias * 30.0f + 2.0f;
    const float xx = 0.1f;
    const float td = std::tanh(xx * drive_);
    compensation_ = xx * (td + bias_) / std::max(td, 1.0e-6f);
  }

  inline void process(float& l, float& r) noexcept {
    l = ch_[0].process(l * inputGain_, drive_, bias_, compensation_, dcK_) *
        outputGain_;
    r = ch_[1].process(r * inputGain_, drive_, bias_, compensation_, dcK_) *
        outputGain_;
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
    inline float process(float x, float drive, float bias, float compensation,
                         float dcK) noexcept {
      const float tx = std::tanh((x - dcS) * drive);
      const float y = tx / (tx + bias) * compensation;
      dcS += dcK * y;
      return y;
    }
  };

  float sr_ = 48000.0f;
  float inputGain_ = 1.0f;
  float outputGain_ = 1.0f;
  float drive_ = 1.5f;
  float bias_ = 10.0f;
  float compensation_ = 1.0f;
  float dcK_ = 0.001f;
  Channel ch_[2];
};

} // namespace hdsp::modules
