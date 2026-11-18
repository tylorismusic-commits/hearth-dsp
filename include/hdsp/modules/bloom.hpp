// hearth-dsp — JClones Bloom (Inflator-style harmonic enhancer)
// SPDX-License-Identifier: GPL-3.0-or-later
//
// C++ translation of the MIT-licensed JClones OInflator JSFX. The original is
// a loudness/warmth processor: a symmetric curve raises low-level harmonics
// without hard-clipping, optionally split into three bands.
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct BloomControls {
  float input = 0.333f;   // 0..1 -> -6..+12 dB
  float effect = 0.0f;    // 0..1 -> 0..100 %
  float curve = 0.5f;     // 0..1 -> -50..+50
  float output = 1.0f;    // 0..1 -> -12..0 dB
  bool bandSplit = false;
  bool clip = true;
};

class Bloom {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    setSampleRate();
    reset();
  }

  void setControls(const BloomControls& c) noexcept {
    inputGain_ = dbToGain(-6.0f + 18.0f * clamp01(c.input)) * 0.5f;
    outputGain_ = dbToGain(-12.0f + 12.0f * clamp01(c.output)) * 2.0f;
    effect_ = clamp01(c.effect);
    const float curveNormalised = clamp01(c.curve);
    curve_ = 0.5f - curveNormalised * 0.5f;
    clipThreshold_ = c.clip ? 0.5f : 0.9999999f;
    bandSplit_ = c.bandSplit;
  }

  inline void process(float& l, float& r) noexcept {
    l = ch_[0].process(l, inputGain_, outputGain_, effect_, curve_,
                       clipThreshold_, bandSplit_, midGain_);
    r = ch_[1].process(r, inputGain_, outputGain_, effect_, curve_,
                       clipThreshold_, bandSplit_, midGain_);
  }

  void reset() noexcept {
    for (auto& ch : ch_) ch.reset();
  }

  int latencySamples() const noexcept { return 0; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  struct FilterParam {
    float k0 = 0, k1 = 0, k2 = 0;
    float state = 0.0f;
    void lpf(float freqNorm) noexcept {
      const float t = std::tan(freqNorm * 3.14159265358979323846f);
      const float a = 0.5f / (t + 1.0f);
      k0 = a * t;
      k1 = a * t;
      k2 = (1.0f - t) * a;
    }
    void hpf(float freqNorm) noexcept {
      const float t = std::tan(freqNorm * 3.14159265358979323846f);
      const float a = 0.125f / (t + 1.0f);
      k0 = a;
      k1 = -a;
      k2 = (1.0f - t) * 0.5f / (t + 1.0f);
    }
  };

  struct Channel {
    FilterParam lowLpf, midLpf, midHpf, highHpf;

    void reset() noexcept {
      lowLpf.state = 0.0f;
      midLpf.state = 0.0f;
      midHpf.state = 0.0f;
      highHpf.state = 0.0f;
    }

    void configure(float sr) noexcept {
      highHpf.hpf(2400.0f / sr);
      midLpf.lpf(2400.0f / sr);
      midHpf.hpf(240.0f / sr);
      lowLpf.lpf(240.0f / sr);
    }

    inline float enhance(float x, float effect, float curve) noexcept {
      float gr = std::fabs(x) * 2.0f * curve + (1.0f - curve);
      gr = std::min(1.0f, std::max(-1.0f, gr));
      const float shaped = gr * x;
      return (1.0f - std::fabs(shaped)) * shaped * (effect * 2.0f) +
             (1.0f - effect) * x;
    }

    inline float process(float x, float inputGain, float outputGain,
                         float effect, float curve, float clipThreshold,
                         bool bandSplit, float midGain) noexcept {
      x *= inputGain;
      x = std::min(clipThreshold, std::max(-clipThreshold, x));

      float y;
      if (!bandSplit) {
        y = enhance(x, effect, curve);
      } else {
        float high = (highHpf.k0 * x + highHpf.state) * 2.0f;
        highHpf.state = highHpf.k1 * x + highHpf.k2 * high;
        high *= 4.0f;

        float halfMid = (midLpf.k0 * x + midLpf.state) * 2.0f;
        midLpf.state = midLpf.k1 * x + midLpf.k2 * halfMid;
        halfMid *= 2.0f;
        float mid = (midHpf.k0 * halfMid + midHpf.state) * 2.0f;
        midHpf.state = halfMid * midHpf.k1 + midHpf.k2 * mid;
        mid *= 2.0f;

        const float lowX = x * 0.5f;
        float low = (lowLpf.k0 * lowX + lowLpf.state) * 2.0f;
        lowLpf.state = lowX * lowLpf.k1 + lowLpf.k2 * low;

        high = enhance(high, effect, curve);
        mid = enhance(mid, effect, curve) * midGain;
        low = enhance(low, effect, curve);

        y = low + mid + high;
      }

      return y * outputGain;
    }
  };

  void setSampleRate() noexcept {
    for (auto& ch : ch_) ch.configure(sr_);
    midGain_ = 1.0f -
               std::tan(240.0f * 3.14159265358979323846f / sr_) /
                   std::tan(2400.0f * 3.14159265358979323846f / sr_);
  }

  float sr_ = 48000.0f;
  float inputGain_ = 0.5f;
  float outputGain_ = 2.0f;
  float effect_ = 0.0f;
  float curve_ = 0.25f;
  float clipThreshold_ = 0.5f;
  float midGain_ = 0.9f;
  bool bandSplit_ = false;
  Channel ch_[2];
};

} // namespace hdsp::modules
