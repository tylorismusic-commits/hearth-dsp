// hearth-dsp — JClones Tape Head saturation
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct TapeHeadControls {
  float drive = 0.28f;  // 0..1 -> 1..10
  float trim = 0.86f;   // 0..1 -> -21..0 dB
  int color = 0;        // 0 normal, 1 medium, 2 bright
  bool clip = true;
};

class TapeHead {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    reset();
  }

  void setControls(const TapeHeadControls& c) noexcept {
    drive_ = std::pow(10.0f, (1.0f + 9.0f * clamp01(c.drive) - 1.0f) / 9.0f) * 0.8f;
    trim_ = std::pow(10.0f, (-21.0f + 21.0f * clamp01(c.trim)) / 20.0f) * 0.7f;
    const int color = std::clamp(c.color, 0, 2);
    const float freq = color == 0 ? 2100.0f : (color == 1 ? 3680.0f : 5000.0f);
    k1_ = 5.0f / 7.0f;
    k2_ = 2.0f * std::sin(freq * 3.14159265358979323846f / sr_);
    k3_ = 1.4f * -k2_;
    const float gain = sr_ < 88200.0f ? 1.4f : 2.0f;
    g3_ = std::pow(10.0f, gain * 3.0f / 20.0f) * -1.0f * k1_;
    clip_ = c.clip;
  }

  inline void process(float& l, float& r) noexcept {
    l = ch_[0].process(l, drive_, trim_, k1_, k2_, k3_, g3_, clip_);
    r = ch_[1].process(r, drive_, trim_, k1_, k2_, k3_, g3_, clip_);
  }

  void reset() noexcept {
    for (auto& ch : ch_) ch.reset();
  }

  int latencySamples() const noexcept { return 0; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }
  static float clip(float x) noexcept {
    return std::min(std::max(x, -1.0f), 1.0f);
  }
  static float smoothstep(float x) noexcept {
    if (x < -1.0f) return -1.0f;
    if (x > 1.0f) return 1.0f;
    return x * 1.5f - x * x * x * 0.5f;
  }

  struct Channel {
    float y1 = 0.0f, y2 = 0.0f, y3 = 0.0f;
    void reset() noexcept { y1 = y2 = y3 = 0.0f; }
    inline float process(float x, float drive, float trim, float k1, float k2,
                         float k3, float g3, bool hardClip) noexcept {
      if (hardClip) x = clip(x);
      y1 += y2 * k2;
      y3 = y1 * k1 + y2 - x;
      y2 += y3 * k3;
      const float y1s = smoothstep(y1 * drive);
      const float y2s = smoothstep(y2 * drive);
      const float y3c = hardClip ? clip(y3) : y3;
      float y = (y3c * g3 + y1s + y2s) * trim;
      if (hardClip) y = clip(y);
      return y;
    }
  };

  float sr_ = 48000.0f;
  float drive_ = 0.8f;
  float trim_ = 0.7f;
  float k1_ = 5.0f / 7.0f;
  float k2_ = 0.0f;
  float k3_ = 0.0f;
  float g3_ = 0.0f;
  bool clip_ = true;
  Channel ch_[2];
};

} // namespace hdsp::modules
