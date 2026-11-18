// hearth-dsp — JClones Fattener
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct FattenerControls {
  float fattness = 0.0f;
  float color = 0.0f;
  float gain = 0.5f;  // 0..1 -> -24..+24 dB
};

class Fattener {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    for (auto& ch : ch_) {
      ch.hp1.setHighPass(20.0f, kSqrt1_2, sr_);
      ch.hp2.setHighPass(30.0f, kSqrt1_2, sr_);
    }
    attackK_ = std::exp(-1.0f / (sr_ * 0.00269f));
    releaseK_ = std::exp(-1.0f / (sr_ * 0.0329f));
    reset();
  }

  void setControls(const FattenerControls& c) noexcept {
    const float fattness = clamp01(c.fattness);
    const float colour = clamp01(c.color);
    threshold_ = dbToGain(-(fattness * 34.0f + 1.0f));
    outputGain_ = dbToGain(fattness * 39.0f + 1.0f);
    finalGain_ = dbToGain(-0.1f);
    inputGain_ = dbToGain(-24.0f + 48.0f * clamp01(c.gain));
    const float gainDb = colour * 8.0f;
    float freq = std::pow(colour, 3.219f) * 6680.0f + 40.0f;
    freq = std::min(freq, sr_ * 0.5f - 100.0f);
    const float q = 0.71f * 1.5f * (std::cos(3.14159265f * freq / sr_) + 1.0f) *
                    std::cos(3.14159265f * freq / sr_) /
                    (2.0f * std::cos(3.14159265f * freq / sr_) + 1.0f);
    for (auto& ch : ch_) {
      ch.peak.setPeak(freq, std::max(q, 0.1f), dbToGain(gainDb), sr_);
    }
    satThreshold_ = 0.1f * colour + 0.9f;
  }

  inline void process(float& l, float& r) noexcept {
    const float ppL = ch_[0].preprocess(l * inputGain_);
    const float ppR = ch_[1].preprocess(r * inputGain_);
    const float level = std::max(std::fabs(ppL), std::fabs(ppR));
    const float k = level > level_ ? attackK_ : releaseK_;
    level_ = (1.0f - k) * level + k * level_;
    float gr = 1.0f;
    if (level_ > threshold_) {
      gr = std::pow(level_ / threshold_, -0.9f);
    }
    l = softClip(ppL * gr * outputGain_) * finalGain_;
    r = softClip(ppR * gr * outputGain_) * finalGain_;
  }

  void reset() noexcept {
    for (auto& ch : ch_) ch.reset();
    level_ = 0.0f;
  }

  int latencySamples() const noexcept { return 0; }

private:
  static constexpr float kSqrt1_2 = 0.70710678f;
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  struct Biquad2 {
    float a0 = 1, a1 = 0, a2 = 0, b1 = 0, b2 = 0;
    float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    void setHighPass(float hz, float q, float sr) noexcept {
      const float w0 = hz * 2.0f * 3.14159265f / sr;
      const float alpha = std::sin(w0) / (2.0f * q);
      a0 = ((1.0f + std::cos(w0)) / 2.0f) / (1.0f + alpha);
      a1 = -(1.0f + std::cos(w0)) / (1.0f + alpha);
      a2 = ((1.0f + std::cos(w0)) / 2.0f) / (1.0f + alpha);
      b1 = (2.0f * std::cos(w0)) / (1.0f + alpha);
      b2 = -(1.0f - alpha) / (1.0f + alpha);
    }
    void setPeak(float hz, float q, float gain, float sr) noexcept {
      const float A = std::sqrt(gain);
      const float w0 = hz * 2.0f * 3.14159265f / sr;
      const float alpha = std::sin(w0) / (2.0f * q);
      a0 = (1.0f + alpha * A) / (1.0f + alpha / A);
      a1 = -2.0f * std::cos(w0) / (1.0f + alpha / A);
      a2 = (1.0f - alpha * A) / (1.0f + alpha / A);
      b1 = (2.0f * std::cos(w0)) / (1.0f + alpha / A);
      b2 = -(1.0f - alpha / A) / (1.0f + alpha / A);
    }
    void reset() noexcept { x1 = x2 = y1 = y2 = 0.0f; }
    inline float process(float x) noexcept {
      const float y = x * a0 + x1 * a1 + x2 * a2 + y1 * b1 + y2 * b2;
      x2 = x1;
      x1 = x;
      y2 = y1;
      y1 = y;
      return y;
    }
  };

  struct Channel {
    Biquad2 hp1, hp2, peak;
    void reset() noexcept {
      hp1.reset();
      hp2.reset();
      peak.reset();
    }
    inline float preprocess(float x) noexcept {
      return peak.process(hp2.process(hp1.process(x)));
    }
  };

  float softClip(float x) const noexcept {
    if (std::fabs(x) <= satThreshold_) return x;
    if (satThreshold_ < 1.0f) {
      const float t = (std::fabs(x) - satThreshold_) / (1.0f - satThreshold_);
      const float y = std::tanh(t) * (1.0f - satThreshold_) + satThreshold_;
      return x > 0.0f ? y : -y;
    }
    return x > 0.0f ? satThreshold_ : -satThreshold_;
  }

  float sr_ = 48000.0f;
  float attackK_ = 0.99f;
  float releaseK_ = 0.999f;
  float threshold_ = 0.9f;
  float outputGain_ = 1.0f;
  float finalGain_ = 0.99f;
  float inputGain_ = 1.0f;
  float satThreshold_ = 0.9f;
  float level_ = 0.0f;
  Channel ch_[2];
};

} // namespace hdsp::modules
