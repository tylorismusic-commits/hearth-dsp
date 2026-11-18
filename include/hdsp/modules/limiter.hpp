// hearth-dsp — lookahead peak limiter module
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A compact 500-series peak limiter with 3 ms lookahead, true-peak detection
// and a linked stereo ceiling. It uses the same sliding-max and Catmull-Rom
// detector ideas as Brick, reduced to the controls a rack module needs.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <hdsp/db.hpp>
#include <hdsp/smoother.hpp>
#include <hdsp/truepeak.hpp>

namespace hdsp::modules {

struct LimiterControls {
  float input = 0.50f;    // 0..1 -> -12..+12 dB
  float ceiling = 0.90f;  // 0..1 -> -6..0 dB
  float release = 0.50f;  // 0..1 -> 20..1000 ms
  float link = 1.00f;     // 0..1
};

class Limiter {
public:
  static constexpr float kMaxLookaheadMs = 3.0f;

  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    maxDelay_ = std::max(1, static_cast<int>(std::lround(
                                  kMaxLookaheadMs * 0.001f * sr_)));
    delaySize_ = maxDelay_ + 4;
    delayL_.assign(static_cast<size_t>(delaySize_), 0.0f);
    delayR_.assign(static_cast<size_t>(delaySize_), 0.0f);
    write_ = 0;
    maxL_.prepare(maxDelay_ + 2);
    maxR_.prepare(maxDelay_ + 2);
    maxL_.setWindow(maxDelay_);
    maxR_.setWindow(maxDelay_);
    inputSmooth_.setTime(20.0f, sr_);
    ceilingSmooth_.setTime(20.0f, sr_);
    releaseSmooth_.setTime(20.0f, sr_);
    linkSmooth_.setTime(20.0f, sr_);
    inputSmooth_.reset(1.0f);
    ceilingSmooth_.reset(dbToGain(-1.0f));
    releaseSmooth_.reset(100.0f);
    linkSmooth_.reset(1.0f);
    reset();
  }

  void setControls(const LimiterControls& c) noexcept {
    inputSmooth_.setTarget(dbToGain(-12.0f + 24.0f * clamp01(c.input)));
    ceilingSmooth_.setTarget(dbToGain(-6.0f + 6.0f * clamp01(c.ceiling)));
    releaseSmooth_.setTarget(20.0f * std::pow(50.0f, clamp01(c.release)));
    linkSmooth_.setTarget(clamp01(c.link));
  }

  inline void process(float& left, float& right) noexcept {
    const float inGain = inputSmooth_.next();
    const float ceiling = ceilingSmooth_.next();
    const float releaseMs = releaseSmooth_.next();
    const float link = linkSmooth_.next();

    const float xL = left * inGain;
    const float xR = right * inGain;
    const float tpL = truePeakL_.process(xL);
    const float tpR = truePeakR_.process(xR);
    const float detL = std::max(std::fabs(xL), tpL);
    const float detR = std::max(std::fabs(xR), tpR);

    maxL_.push(detL);
    maxR_.push(detR);
    const float peakL = maxL_.max();
    const float peakR = maxR_.max();

    const float ceil = std::max(ceiling, 1.0e-6f);
    const float targetL = std::min(1.0f, ceil / std::max(peakL, ceil));
    const float targetR = std::min(1.0f, ceil / std::max(peakR, ceil));
    const float attackMs = 0.05f;
    const float attackCoeff =
        std::exp(-1.0f / std::max(1.0f, attackMs * 0.001f * sr_));
    const float releaseCoeff =
        std::exp(-1.0f / std::max(1.0f, releaseMs * 0.001f * sr_));
    if (targetL < gainL_) {
      gainL_ = attackCoeff * gainL_ + (1.0f - attackCoeff) * targetL;
    } else {
      gainL_ = releaseCoeff * gainL_ + (1.0f - releaseCoeff) * targetL;
    }
    if (targetR < gainR_) {
      gainR_ = attackCoeff * gainR_ + (1.0f - attackCoeff) * targetR;
    } else {
      gainR_ = releaseCoeff * gainR_ + (1.0f - releaseCoeff) * targetR;
    }
    gainL_ = std::min(gainL_, targetL);
    gainR_ = std::min(gainR_, targetR);

    const float linked = std::min(gainL_, gainR_);
    const float outGainL = gainL_ + link * (linked - gainL_);
    const float outGainR = gainR_ + link * (linked - gainR_);

    delayL_[write_] = xL;
    delayR_[write_] = xR;
    int read = write_ - maxDelay_;
    if (read < 0) read += delaySize_;
    float outL = delayL_[read] * outGainL;
    float outR = delayR_[read] * outGainR;
    write_ = (write_ + 1) % delaySize_;

    outL = std::min(std::max(outL, -ceil), ceil);
    outR = std::min(std::max(outR, -ceil), ceil);
    left = outL;
    right = outR;
    lastGrDb_ = -gainToDb(std::max(1.0e-6f, std::min(outGainL, outGainR)));
  }

  void reset() noexcept {
    std::fill(delayL_.begin(), delayL_.end(), 0.0f);
    std::fill(delayR_.begin(), delayR_.end(), 0.0f);
    write_ = 0;
    maxL_.reset();
    maxR_.reset();
    truePeakL_.reset();
    truePeakR_.reset();
    gainL_ = gainR_ = 1.0f;
    lastGrDb_ = 0.0f;
    inputSmooth_.snap();
    ceilingSmooth_.snap();
    releaseSmooth_.snap();
    linkSmooth_.snap();
  }

  int latencySamples() const noexcept { return maxDelay_; }
  float lastGrDb() const noexcept { return lastGrDb_; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  class SlidingMax {
  public:
    void prepare(int maxWindow) noexcept {
      maxWindow_ = std::max(2, maxWindow);
      values_.assign(static_cast<size_t>(maxWindow_ + 2), 0.0f);
      indices_.assign(static_cast<size_t>(maxWindow_ + 2), 0);
      capacity_ = maxWindow_ + 2;
      head_ = tail_ = count_ = 0;
      counter_ = 0;
      window_ = 1;
    }

    void reset() noexcept {
      head_ = tail_ = count_ = 0;
      counter_ = 0;
      std::fill(values_.begin(), values_.end(), 0.0f);
    }

    void setWindow(int window) noexcept {
      window_ = std::min(std::max(window, 1), maxWindow_);
    }

    inline void push(float value) noexcept {
      while (count_ > 0) {
        const int last = (tail_ + capacity_ - 1) % capacity_;
        if (values_[last] <= value) {
          tail_ = last;
          --count_;
        } else {
          break;
        }
      }
      values_[tail_] = value;
      indices_[tail_] = counter_;
      tail_ = (tail_ + 1) % capacity_;
      ++count_;
      while (count_ > 0 && indices_[head_] + window_ <= counter_) {
        head_ = (head_ + 1) % capacity_;
        --count_;
      }
      ++counter_;
    }

    float max() const noexcept {
      return count_ > 0 ? values_[head_] : 0.0f;
    }

  private:
    std::vector<float> values_;
    std::vector<uint64_t> indices_;
    int maxWindow_ = 2;
    int capacity_ = 4;
    int head_ = 0;
    int tail_ = 0;
    int count_ = 0;
    int window_ = 1;
    uint64_t counter_ = 0;
  };

  float sr_ = 48000.0f;
  int maxDelay_ = 1;
  int delaySize_ = 8;
  int write_ = 0;
  std::vector<float> delayL_;
  std::vector<float> delayR_;
  SlidingMax maxL_;
  SlidingMax maxR_;
  TruePeakDetector truePeakL_;
  TruePeakDetector truePeakR_;
  Smoothed inputSmooth_;
  Smoothed ceilingSmooth_;
  Smoothed releaseSmooth_;
  Smoothed linkSmooth_;
  float gainL_ = 1.0f;
  float gainR_ = 1.0f;
  float lastGrDb_ = 0.0f;
};

} // namespace hdsp::modules
