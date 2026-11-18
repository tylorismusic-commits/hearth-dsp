// hearth-dsp — JClones Modern Limiter
// SPDX-License-Identifier: GPL-3.0-or-later
//
// C++ translation of the MIT-licensed JClones FabModern Limiter JSFX. It uses
// a short 0.18 ms lookahead, a minimum-hold plus moving-average gain smoother
// and a two-stage slow release network.
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct ModernLimiterControls {
  float input = 0.50f;      // 0..1 -> -18..+18 dB
  float threshold = 0.95f;  // 0..1 -> -20..0 dB
  float output = 0.50f;     // 0..1 -> -12..+12 dB
  float mix = 1.00f;
};

class ModernLimiter {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    lookahead_ = std::max(2, static_cast<int>(std::floor(0.00018f * sr_ + 0.5f)));
    for (auto& ch : ch_) ch.prepare(lookahead_);
    slow_.prepare(sr_);
    reset();
  }

  void setControls(const ModernLimiterControls& c) noexcept {
    inputGain_ = dbToGain(-18.0f + 36.0f * clamp01(c.input));
    threshold_ = dbToGain(-20.0f + 20.0f * clamp01(c.threshold));
    outputGain_ = dbToGain(-12.0f + 24.0f * clamp01(c.output));
    mix_ = clamp01(c.mix);
  }

  inline void process(float& l, float& r) noexcept {
    const float dryL = l, dryR = r;
    l *= inputGain_;
    r *= inputGain_;
    const float grL = l <= threshold_ ? 1.0f : threshold_ / std::max(std::fabs(l), 1.0e-9f);
    const float grR = r <= threshold_ ? 1.0f : threshold_ / std::max(std::fabs(r), 1.0e-9f);
    const float linked = std::min(grL, grR);
    const float slow = slow_.process(linked);
    float outL = ch_[0].process(l, grL, slow);
    float outR = ch_[1].process(r, grR, slow);
    outL *= outputGain_;
    outR *= outputGain_;
    l = dryL + (outL - dryL) * mix_;
    r = dryR + (outR - dryR) * mix_;
    lastGrDb_ = -gainToDb(std::max(1.0e-6f, std::min(ch_[0].gr, ch_[1].gr)));
  }

  void reset() noexcept {
    for (auto& ch : ch_) ch.reset();
    slow_.reset();
    lastGrDb_ = 0.0f;
  }

  int latencySamples() const noexcept { return lookahead_ - 1; }
  float lastGrDb() const noexcept { return lastGrDb_; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  struct Filter1 {
    float a0 = 1, a1 = 0, b1 = 0;
    float x1 = 0, y1 = 0;
    void butLp(float freqNorm) noexcept {
      const float w0 = freqNorm * 2.0f * 3.14159265358979323846f;
      const float kk = 1.0f / std::tan(w0 / 2.0f);
      const float b0 = 1.0f + kk;
      const float b1 = 1.0f - kk;
      a0 = 1.0f / b0;
      a1 = 1.0f / b0;
      this->b1 = -b1 / b0;
    }
    void step99(float raise99Sec, float sr) noexcept {
      butLp((0.732936f / raise99Sec) / sr);
    }
    void reset(float v) noexcept { x1 = y1 = v; }
    float process(float x) noexcept {
      const float y = a0 * x + a1 * x1 + b1 * y1;
      x1 = x;
      y1 = y;
      return y;
    }
    float pass(float x) noexcept { x1 = y1 = x; return x; }
    float back() const noexcept { return y1; }
  };

  struct DelayLine {
    std::vector<float> line;
    int head = 0;
    void init(int size) noexcept { line.assign(static_cast<size_t>(std::max(1, size)), 0.0f); head = 0; }
    void reset(float v) noexcept { std::fill(line.begin(), line.end(), v); head = 0; }
    float push(float x) noexcept {
      const float y = line[static_cast<size_t>(head)];
      line[static_cast<size_t>(head)] = x;
      head = (head + 1) % static_cast<int>(line.size());
      return y;
    }
    float minimum() const noexcept {
      float m = line[0];
      for (float v : line) m = std::min(m, v);
      return m;
    }
  };

  struct MovingAverage {
    std::vector<float> line;
    int head = 0;
    float sum = 0.0f;
    void init(int size) noexcept { line.assign(static_cast<size_t>(std::max(1, size)), 1.0f); head = 0; }
    void reset(float v) noexcept {
      std::fill(line.begin(), line.end(), v);
      head = 0;
      sum = v * static_cast<float>(line.size());
    }
    float push(float x) noexcept {
      sum -= line[static_cast<size_t>(head)];
      line[static_cast<size_t>(head)] = x;
      head = (head + 1) % static_cast<int>(line.size());
      sum += x;
      return sum / static_cast<float>(line.size());
    }
  };

  struct SlowGR {
    Filter1 attack1, attack2, release1, release2, slowAttack, slowRelease;
    void prepare(float sr) noexcept {
      attack1.step99(0.025f, sr);
      attack2 = attack1;
      release1.step99(0.200f, sr);
      release2 = release1;
      slowAttack.step99(0.150f, sr);
      slowRelease.step99(1.5f, sr);
    }
    void reset() noexcept {
      attack1.reset(1.0f);
      attack2.reset(1.0f);
      release1.reset(1.0f);
      release2.reset(1.0f);
      slowAttack.reset(1.0f);
      slowRelease.reset(1.0f);
    }
    float process(float gr) noexcept {
      float gr2 = gr * gr;
      gr2 = attack1.process(gr2);
      gr2 = attack2.process(gr2);
      gr2 = gr2 > release1.back() ? release1.process(gr2) : release1.pass(gr2);
      gr2 = gr2 > release2.back() ? release2.process(gr2) : release2.pass(gr2);
      const float slow = slowAttack.process(gr);
      const float rel = slow > slowRelease.back() ? slowRelease.process(slow)
                                                  : slowRelease.pass(slow);
      return std::min(rel, std::sqrt(std::max(gr2, 0.0f)));
    }
  };

  struct Channel {
    DelayLine delayX, delayMin;
    MovingAverage flt;
    float gr = 1.0f;
    void prepare(int lookahead) noexcept {
      delayX.init(lookahead - 1);
      delayMin.init(lookahead);
      flt.init(lookahead);
    }
    void reset() noexcept {
      delayX.reset(0.0f);
      delayMin.reset(1.0f);
      flt.reset(1.0f);
      gr = 1.0f;
    }
    float process(float x, float fastGr, float slowGr) noexcept {
      float g = std::min(fastGr, slowGr);
      delayMin.push(g);
      g = delayMin.minimum();
      g = flt.push(g);
      gr = g;
      x = delayX.push(x);
      return x * g;
    }
  };

  float sr_ = 48000.0f;
  int lookahead_ = 8;
  float inputGain_ = 1.0f;
  float threshold_ = 0.9f;
  float outputGain_ = 1.0f;
  float mix_ = 1.0f;
  float lastGrDb_ = 0.0f;
  SlowGR slow_;
  Channel ch_[2];
};

} // namespace hdsp::modules
