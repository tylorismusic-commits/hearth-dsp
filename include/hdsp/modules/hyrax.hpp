// hearth-dsp — JClones Hyrax limiter
// SPDX-License-Identifier: GPL-3.0-or-later
//
// C++ translation of the MIT-licensed JClones Hyrax limiter. It uses a
// sliding-max attack window, a block-based linear-phase attack filter, a hold
// stage and a slow release filter.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct HyraxControls {
  float input = 0.50f;      // 0..1 -> -18..+18 dB
  float threshold = 0.94f;  // 0..1 -> -18..0 dB
  float output = 0.50f;     // 0..1 -> -12..+12 dB
  float mix = 1.00f;
};

class Hyrax {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    limiter_.prepare(sr_);
    reset();
  }

  void setControls(const HyraxControls& c) noexcept {
    inputGain_ = dbToGain(-18.0f + 36.0f * clamp01(c.input));
    threshold_ = dbToGain(-18.0f + 18.0f * clamp01(c.threshold));
    outputGain_ = dbToGain(-12.0f + 24.0f * clamp01(c.output));
    mix_ = clamp01(c.mix);
    limiter_.setThreshold(threshold_);
  }

  inline void process(float& l, float& r) noexcept {
    const float dryL = l, dryR = r;
    float yL, yR;
    limiter_.process(l * inputGain_, r * inputGain_, yL, yR);
    yL *= outputGain_;
    yR *= outputGain_;
    l = dryL + (yL - dryL) * mix_;
    r = dryR + (yR - dryR) * mix_;
    lastGrDb_ = limiter_.lastGrDb();
  }

  void reset() noexcept {
    limiter_.reset();
    lastGrDb_ = 0.0f;
  }

  int latencySamples() const noexcept { return limiter_.latencySamples(); }
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
    void reset() noexcept { x1 = y1 = 0.0f; }
    float process(float x) noexcept {
      const float y = a0 * x + a1 * x1 + b1 * y1;
      x1 = x;
      y1 = y;
      return y;
    }
  };

  struct SlidingMax {
    std::deque<std::pair<uint64_t, float>> dq;
    uint64_t counter = 0;
    int window = 1;
    void prepare(int w) noexcept {
      window = std::max(1, w);
      reset();
    }
    void reset() noexcept {
      dq.clear();
      counter = 0;
    }
    void push(float v) noexcept {
      while (!dq.empty() && dq.back().second <= v) dq.pop_back();
      dq.emplace_back(counter, v);
      while (!dq.empty() && dq.front().first + static_cast<uint64_t>(window) <= counter) {
        dq.pop_front();
      }
      ++counter;
    }
    float max() const noexcept { return dq.empty() ? 0.0f : dq.front().second; }
  };

  struct DelayLine {
    std::vector<float> line;
    int head = 0;
    void prepare(int size) noexcept {
      line.assign(static_cast<size_t>(std::max(1, size)), 0.0f);
      head = 0;
    }
    void reset() noexcept {
      std::fill(line.begin(), line.end(), 0.0f);
      head = 0;
    }
    float push(float x) noexcept {
      const float y = line[static_cast<size_t>(head)];
      line[static_cast<size_t>(head)] = x;
      head = (head + 1) % static_cast<int>(line.size());
      return y;
    }
    int size() const noexcept { return static_cast<int>(line.size()); }
  };

  struct LinearPhaseIIR1 {
    int blockSize = 2;
    int half = 1;
    Filter1 forward, backward;
    std::vector<float> input, output0, output1;
    int offset = 0;
    int outputIndex = 0;

    void prepare(int block) noexcept {
      blockSize = std::max(2, block);
      half = blockSize / 2;
      input.assign(static_cast<size_t>(blockSize), 0.0f);
      output0.assign(static_cast<size_t>(blockSize + half), 0.0f);
      output1.assign(static_cast<size_t>(blockSize + half), 0.0f);
      offset = 0;
      outputIndex = 0;
      reset();
    }
    void setHalfIIR(float a0, float a1, float b1) noexcept {
      forward.a0 = a0;
      forward.a1 = a1;
      forward.b1 = b1;
    }
    int latency() const noexcept { return blockSize + half; }
    void reset() noexcept {
      std::fill(input.begin(), input.end(), 0.0f);
      std::fill(output0.begin(), output0.end(), 0.0f);
      std::fill(output1.begin(), output1.end(), 0.0f);
      offset = 0;
      outputIndex = 0;
      forward.reset();
      backward.reset();
    }
    float process(float x) noexcept {
      std::vector<float>& out = outputIndex == 0 ? output0 : output1;
      std::vector<float>& other = outputIndex == 0 ? output1 : output0;
      float y = out[static_cast<size_t>(offset)];
      if (offset < half) {
        y += other[static_cast<size_t>(offset + blockSize)];
      }
      input[static_cast<size_t>(offset)] = x;
      ++offset;
      if (offset >= blockSize) {
        outputIndex = 1 - outputIndex;
        std::vector<float>& next = outputIndex == 0 ? output0 : output1;
        std::fill(next.begin(), next.begin() + half, 0.0f);
        std::copy(input.begin(), input.end(), next.begin() + half);
        backward.reset();
        int p = blockSize + half - 1;
        for (int i = 0; i < blockSize; ++i) {
          next[static_cast<size_t>(p)] = backward.process(next[static_cast<size_t>(p)]);
          --p;
        }
        for (int i = 0; i < half; ++i) {
          next[static_cast<size_t>(p)] = backward.process(0.0f);
          --p;
        }
        offset = 0;
      }
      return forward.process(y);
    }
  };

  struct Limiter {
    SlidingMax attackHold;
    LinearPhaseIIR1 attackFilter;
    DelayLine slidedAttackDelay;
    SlidingMax holdHold;
    Filter1 holdFilter;
    Filter1 releaseFilter;
    DelayLine grCompensation;
    DelayLine xLeftCompensation;
    DelayLine xRightCompensation;
    float threshold = 0.5f;
    float gr = 1.0f;

    void prepare(float sr) noexcept {
      const int attackSamples = std::max(1, static_cast<int>(std::floor(sr * 0.001f)));
      const int odd = (attackSamples % 2 == 0) ? attackSamples + 1 : attackSamples;
      attackHold.prepare(2 * odd - 1);
      const float coeff = std::exp(-2.0f / static_cast<float>(attackSamples));
      const float blockLen = std::log(1.0e-6f) / std::log(coeff);
      attackFilter.prepare(static_cast<int>(std::lround(blockLen)));
      attackFilter.setHalfIIR(1.0f - coeff, 0.0f, coeff);
      slidedAttackDelay.prepare(attackFilter.latency());
      const int holdWindow = std::max(1, static_cast<int>(std::floor(sr * 0.001f)));
      holdHold.prepare(holdWindow);
      holdFilter.butLp(7.0f / sr);
      releaseFilter.butLp((800.0f / 3000.0f) / sr);
      const int latency = holdWindow + attackFilter.latency();
      grCompensation.prepare(latency);
      xLeftCompensation.prepare(latency);
      xRightCompensation.prepare(latency);
      reset();
    }
    void setThreshold(float t) noexcept { threshold = t; }
    int latencySamples() const noexcept { return grCompensation.size(); }
    float lastGrDb() const noexcept {
      return -gainToDb(std::max(gr, 1.0e-6f));
    }
    void reset() noexcept {
      attackHold.reset();
      attackFilter.reset();
      slidedAttackDelay.reset();
      holdHold.reset();
      holdFilter.reset();
      releaseFilter.reset();
      grCompensation.reset();
      xLeftCompensation.reset();
      xRightCompensation.reset();
      gr = 1.0f;
    }
    void process(float xL, float xR, float& yL, float& yR) noexcept {
      const float level = std::max(std::fabs(xL), std::fabs(xR));
      float flipGr = 1.0f - threshold / std::max(level, threshold);
      attackHold.push(flipGr);
      float slidedAttack = attackHold.max();
      const float attack = attackFilter.process(slidedAttack);
      slidedAttack = slidedAttackDelay.push(slidedAttack);
      flipGr = grCompensation.push(flipGr);
      holdHold.push(slidedAttack);
      const float slidedRelease = holdHold.max();
      const float holdOutput = holdFilter.process(slidedRelease);
      const float releaseOutput = releaseFilter.process(std::max(slidedRelease, holdOutput));
      const float release = std::max(holdOutput, releaseOutput);
      yL = xLeftCompensation.push(xL);
      yR = xRightCompensation.push(xR);
      const float total = std::max(std::max(attack, release), flipGr);
      gr = 1.0f - total;
      yL *= gr;
      yR *= gr;
    }
  };

  float sr_ = 48000.0f;
  float inputGain_ = 1.0f;
  float threshold_ = 0.9f;
  float outputGain_ = 1.0f;
  float mix_ = 1.0f;
  float lastGrDb_ = 0.0f;
  Limiter limiter_;
};

} // namespace hdsp::modules
