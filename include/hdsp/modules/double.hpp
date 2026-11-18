// hearth-dsp — doubler / microshift module
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A short stereo delay pair with opposite low-rate modulation. The result is
// a wider, slightly detuned double without a full pitch shifter.
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace hdsp::modules {

struct DoubleControls {
  float delay = 0.30f;   // 0..1 -> 10..40 ms
  float detune = 0.40f;  // 0..1 -> 0..3 ms modulation
  float width = 0.50f;   // 0..1 -> stereo offset
  float mix = 0.35f;
};

class Double {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    maxDelay_ = static_cast<int>(0.060f * sr_) + 8;
    bufferL_.assign(static_cast<size_t>(maxDelay_), 0.0f);
    bufferR_.assign(static_cast<size_t>(maxDelay_), 0.0f);
    reset();
  }

  void setControls(const DoubleControls& c) noexcept {
    baseDelayMs_ = 10.0f + 30.0f * clamp01(c.delay);
    modDepthMs_ = 3.0f * clamp01(c.detune);
    widthMs_ = 2.0f * clamp01(c.width);
    mix_ = clamp01(c.mix);
  }

  inline void process(float& l, float& r) noexcept {
    phase_ += 2.0f * 3.14159265f * 0.30f / sr_;
    if (phase_ >= 2.0f * 3.14159265f) phase_ -= 2.0f * 3.14159265f;
    const float mod = std::sin(phase_) * modDepthMs_;

    const float delayL = msToSamples(baseDelayMs_ + widthMs_ + mod);
    const float delayR = msToSamples(baseDelayMs_ - widthMs_ - mod);
    const float wetL = read(bufferL_, delayL);
    const float wetR = read(bufferR_, delayR);

    bufferL_[write_] = l;
    bufferR_[write_] = r;
    write_ = (write_ + 1) % maxDelay_;

    l += wetL * mix_ * 0.7f;
    r += wetR * mix_ * 0.7f;
  }

  void reset() noexcept {
    std::fill(bufferL_.begin(), bufferL_.end(), 0.0f);
    std::fill(bufferR_.begin(), bufferR_.end(), 0.0f);
    write_ = 0;
    phase_ = 0.0f;
  }

  int latencySamples() const noexcept { return 0; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  float msToSamples(float ms) const noexcept {
    return std::clamp(ms * 0.001f * sr_, 1.0f,
                      static_cast<float>(maxDelay_ - 4));
  }

  float read(const std::vector<float>& buffer, float delaySamples) const noexcept {
    float pos = static_cast<float>(write_) - delaySamples;
    while (pos < 0.0f) pos += static_cast<float>(maxDelay_);
    const int i0 = static_cast<int>(pos) % maxDelay_;
    const int i1 = (i0 + 1) % maxDelay_;
    const float frac = pos - std::floor(pos);
    return buffer[i0] * (1.0f - frac) + buffer[i1] * frac;
  }

  float sr_ = 48000.0f;
  int maxDelay_ = 2896;
  int write_ = 0;
  float baseDelayMs_ = 19.0f;
  float modDepthMs_ = 1.2f;
  float widthMs_ = 1.0f;
  float mix_ = 0.35f;
  float phase_ = 0.0f;
  std::vector<float> bufferL_;
  std::vector<float> bufferR_;
};

} // namespace hdsp::modules
