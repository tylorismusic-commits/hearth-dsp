// hearth-dsp — JClones L2 brickwall limiter
// SPDX-License-Identifier: GPL-3.0-or-later
//
// C++ translation of the MIT-licensed JClones L2 JSFX. It uses a sliding
// look-ahead minimum, dual moving averages, manual or program-dependent auto
// release, and the original 24-bit IDR dither/noise-shaping stage.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct L2Controls {
  float threshold = 1.0f;  // 0..1 -> -30..0 dB
  float ceiling = 1.0f;    // 0..1 -> -30..0 dB
  bool autoRelease = true;
  float release = 0.4f;    // 0..1 -> slider 0..5 ms (log)
  int quantize = 4;        // 0..4 -> 16,18,20,22,24 bit
  int dither = 1;          // 0 none, 1 type 1, 2 type 2
  int shaping = 2;         // 0 none, 1 moderate, 2 normal, 3 ultra
};

class L2 {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    lookahead_ =
        static_cast<int>(std::floor((0.0014f * sr_) / 64.0f + 0.6f)) * 64;
    if (lookahead_ < 64) lookahead_ = 64;
    delayMin_.init(lookahead_);
    moving1_.init(lookahead_ / 2 + 1);
    moving2_.init(lookahead_ / 2 + 1);
    delayL_.assign(static_cast<size_t>(lookahead_), 0.0f);
    delayR_.assign(static_cast<size_t>(lookahead_), 0.0f);
    delayHead_ = 0;

    slowAttackG_ = filterG(0.250f);
    slowReleaseG_ = filterG(0.500f);
    dynamicG1_ = filterG(0.281f);
    dynamicG2_ = filterG(0.057f);
    dynamicG3_ = filterG(0.00054f);

    sampleRateIndex_ = sr_ <= 44100.0f ? 0 : sr_ <= 48000.0f ? 1
                       : sr_ <= 88200.0f ? 2 : 3;

    reset();
    dirty_ = true;
  }

  void setControls(const L2Controls& c) noexcept {
    threshold_ = clamp01(c.threshold);
    ceiling_ = clamp01(c.ceiling);
    autoRelease_ = c.autoRelease;
    release_ = clamp01(c.release);
    quantize_ = std::clamp(c.quantize, 0, 4);
    ditherType_ = std::clamp(c.dither, 0, 2);
    shaping_ = std::clamp(c.shaping, 0, 3);
    dirty_ = true;
  }

  inline void process(float& l, float& r) noexcept {
    updateParameters();
    const float inGain = inputGain_;
    const float outGain = outputGain_;

    const float level =
        std::max(std::fabs(l * inGain), std::fabs(r * inGain));
    float gr = level < 1.0f ? 1.0f : 1.0f / level;

    delayMin_.push(gr);
    const float minGr = delayMin_.minValue();

    float outputGr;
    if (!autoRelease_) {
      if (minGr < fastGrS_) {
        fastGrS_ = minGr;
      } else {
        fastGrS_ = filter1(fastGrS_, fastGrG_, minGr);
      }
      outputGr = fastGrS_;
    } else {
      if (minGr < slowGrS_) {
        slowGrS_ = filter1(slowGrS_, slowAttackG_, minGr);
      } else {
        slowGrS_ = filter1(slowGrS_, slowReleaseG_, minGr);
      }
      const float combined = std::min(minGr, slowGrS_);
      if (combined < dynamicGrS_) {
        dynamicGrS_ = combined;
      } else {
        dynamicGrS_ = filter1(dynamicGrS_, dynamicReleaseG_, combined);
      }
      outputGr = dynamicGrS_;

      const float dynamicRelease = std::pow(1.0f - outputGr, 4.0f);
      const float dynamicReleaseG1 =
          std::max(filterG(dynamicRelease), kFilter1GMin);
      const float dynamicFactor = std::min(slowGrS_ * 2.0f - outputGr, 1.0f);
      float dynamicReleaseG2;
      if (dynamicFactor < 0.25f) {
        dynamicReleaseG2 = 1.0f;
      } else if (dynamicFactor < 0.5f) {
        dynamicReleaseG2 =
            (dynamicFactor - 0.25f) / 0.25f * (dynamicG1_ - 1.0f) + 1.0f;
      } else if (dynamicFactor < 0.71f) {
        dynamicReleaseG2 = (dynamicFactor - 0.5f) / 0.21f *
                               (dynamicG2_ - dynamicG1_) +
                           dynamicG1_;
      } else {
        dynamicReleaseG2 = (dynamicFactor - 0.71f) / 0.29f *
                               (dynamicG3_ - dynamicG2_) +
                           dynamicG2_;
      }
      dynamicReleaseG_ = dynamicReleaseG1 * dynamicReleaseG2;
    }

    outputGr = moving1_.push(outputGr);
    outputGr = moving2_.push(outputGr);
    holdMaxGr_ = std::min(holdMaxGr_, outputGr);

    l *= inGain;
    r *= inGain;
    const float delayedL = delayL_[static_cast<size_t>(delayHead_)];
    const float delayedR = delayR_[static_cast<size_t>(delayHead_)];
    delayL_[static_cast<size_t>(delayHead_)] = l;
    delayR_[static_cast<size_t>(delayHead_)] = r;
    delayHead_ = (delayHead_ + 1) % lookahead_;

    l = delayedL * outputGr * outGain;
    r = delayedR * outputGr * outGain;
    dither_.process(l, r, ditherType_, shaping_, sampleRateIndex_);
    lastGrDb_ = -gainToDb(std::max(outputGr, 1.0e-6f));
  }

  void reset() noexcept {
    delayMin_.reset(1.0f);
    moving1_.reset(1.0f);
    moving2_.reset(1.0f);
    std::fill(delayL_.begin(), delayL_.end(), 0.0f);
    std::fill(delayR_.begin(), delayR_.end(), 0.0f);
    delayHead_ = 0;
    slowGrS_ = 1.0f;
    fastGrS_ = 1.0f;
    dynamicGrS_ = 1.0f;
    dynamicReleaseG_ = filterG(0.001f);
    dither_.reset();
    holdMaxGr_ = 1.0f;
    lastGrDb_ = 0.0f;
  }

  int latencySamples() const noexcept { return lookahead_; }
  float lastGrDb() const noexcept { return lastGrDb_; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }
  static float filter1(float s, float g, float x) noexcept {
    return x + g * (s - x);
  }
  float filterG(float timeRef20) const noexcept {
    return std::exp(-1.0f / (timeRef20 * sr_ / kLn10));
  }

  struct DelayLineMin {
    std::vector<float> line;
    int head = 0;
    void init(int size) noexcept {
      line.assign(static_cast<size_t>(std::max(1, size)), 1.0f);
      head = 0;
    }
    void reset(float v) noexcept {
      std::fill(line.begin(), line.end(), v);
      head = 0;
    }
    void push(float x) noexcept {
      line[static_cast<size_t>(head)] = x;
      head = (head + 1) % static_cast<int>(line.size());
    }
    float minValue() const noexcept {
      float m = line[0];
      for (float v : line) m = std::min(m, v);
      return m;
    }
  };

  struct MovingAverage {
    std::vector<float> line;
    int head = 0;
    float sum = 0.0f;
    void init(int size) noexcept {
      line.assign(static_cast<size_t>(std::max(1, size)), 1.0f);
      head = 0;
    }
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

  struct Dither {
    float historyL[9] = {};
    float historyR[9] = {};
    int index = 0;
    uint32_t type1State = 0;
    float type2A = 0.0f;
    float type2B = 0.25f;
    float scaling = 1.0f;

    void reset() noexcept {
      type1State = 0;
      type2A = 0.0f;
      type2B = 0.25f;
      for (int i = 0; i < 9; ++i) historyL[i] = historyR[i] = 0.0f;
      index = 0;
    }

    static int32_t type1Rand(uint32_t& state) noexcept {
      state = 10368889u * state + 1u;
      if (state > 2147483648u)
        return static_cast<int32_t>(static_cast<int64_t>(state) - 4294967296LL);
      return static_cast<int32_t>(state);
    }

    float shape(const float* history, int shaping, int srIndex) noexcept {
      const float* ir = kDitherTable + 9 * (shaping + 4 * srIndex);
      float filtered = 0.0f;
      int i = index;
      for (int k = 0; k < 9; ++k) {
        filtered += history[i] * ir[k];
        i = (i + 1) % 9;
      }
      return filtered;
    }

    void process(float& l, float& r, int ditherType, int shaping,
                 int srIndex) noexcept {
      float r1L = 0.0f, r1R = 0.0f, r2L = 0.0f, r2R = 0.0f;
      if (ditherType == 1) {
        const int32_t a = type1Rand(type1State);
        const int32_t b = type1Rand(type1State);
        r1L = static_cast<float>(a + b) / 4294967296.0f;
        r1R = static_cast<float>(a - b) / 4294967296.0f;
      } else if (ditherType == 2) {
        type2A -= 0.16666667f;
        if (type2A < -0.16666667f) type2A = 0.16666667f;
        type2B = -type2B;
        r2L = type2A + type2B;
        r2R = type2A - type2B;
      }

      const float filteredL = shape(historyL, shaping, srIndex);
      const float filteredR = shape(historyR, shaping, srIndex);

      auto convert = [&](float in, float filtered, float rand1,
                         float rand2) {
        in = std::min(1.0f, std::max(-1.0f, in));
        const float x1 = in * 8388608.0f;
        const float x2 = x1 + rand2 * scaling;
        const float x3 = x2 + (rand1 + 0.5f) * scaling + filtered;
        float out = std::floor(x3);
        out = std::min(8388607.0f, std::max(-8388608.0f, out));
        if (scaling != 1.0f) out = std::floor(out / scaling) * scaling;
        error_ = x2 - out;
        return out / 8388608.0f;
      };

      l = convert(l, filteredL, r1L, r2L);
      const float errL = error_;
      r = convert(r, filteredR, r1R, r2R);
      historyL[index] = errL;
      historyR[index] = error_;
      index = (index + 1) % 9;
    }

   private:
    float error_ = 0.0f;
  };

  void updateParameters() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    inputGain_ = 1.0f / dbToGain(-30.0f + 30.0f * threshold_);
    outputGain_ = dbToGain(-30.0f + 30.0f * ceiling_);
    const float releaseMsSlider = 5.0f * release_;
    fastGrG_ = filterG(std::pow(10.0f, releaseMsSlider - 5.0f));
    const int bits = 16 + quantize_ * 2;
    dither_.scaling = static_cast<float>(1u << (24 - bits));
  }

  static constexpr float kLn10 = 2.302585092994046f;
  static constexpr float kFilter1GMin = 0.36787944117144233f;

  float sr_ = 48000.0f;
  int lookahead_ = 64;
  int sampleRateIndex_ = 1;
  float threshold_ = 1.0f, ceiling_ = 1.0f;
  bool autoRelease_ = true;
  float release_ = 0.4f;
  int quantize_ = 4, ditherType_ = 1, shaping_ = 2;
  bool dirty_ = true;

  float inputGain_ = 1.0f, outputGain_ = 1.0f;
  float fastGrG_ = 0.99f;
  float slowAttackG_ = 0.99f, slowReleaseG_ = 0.99f;
  float dynamicG1_ = 0.99f, dynamicG2_ = 0.99f, dynamicG3_ = 0.99f;
  float dynamicReleaseG_ = 0.99f;
  float slowGrS_ = 1.0f, fastGrS_ = 1.0f, dynamicGrS_ = 1.0f;
  float holdMaxGr_ = 1.0f;
  float lastGrDb_ = 0.0f;

  DelayLineMin delayMin_;
  MovingAverage moving1_, moving2_;
  std::vector<float> delayL_, delayR_;
  int delayHead_ = 0;
  Dither dither_;

  // Noise-shaping FIR coefficients: 9 taps x 4 modes x 4 sample rates.
  static constexpr float kDitherTable[144] = {
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      -0.054383024f, -0.20654947f, -0.3822527f, -0.39219186f, -0.36627461f, -0.49556099f, -0.29294844f, 0.61599216f, 1.3177642f,
      -0.070082023f, -0.29561589f, -0.5451639f, -0.5487125f, -0.55320585f, -0.75886502f, -0.25751337f, 1.2202875f, 1.9089777f,
      -0.10739672f, -0.43569984f, -0.72708491f, -0.66456244f, -0.73261389f, -1.0359325f, -0.16065249f, 1.8239374f, 2.3525222f,
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      -0.1002716f, -0.29584958f, -0.40982519f, -0.38606261f, -0.44715518f, -0.52985251f, -0.10806594f, 0.82622085f, 1.3482654f,
      -0.11617057f, -0.41429373f, -0.69819106f, -0.84679904f, -0.99510928f, -0.8137319f, 0.33485791f, 1.8264681f, 2.072573f,
      -0.15380712f, -0.5469209f, -0.91673806f, -1.1670967f, -1.4463654f, -1.0427872f, 0.84138339f, 2.7803198f, 2.624423f,
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 0.088072546f, 0.36459012f, 0.88040251f, 1.5499839f, 2.1484938f, 2.4151576f, 2.2070139f, 1.5982079f,
      -0.00390274f, 0.1073065f, 0.63257259f, 1.8682127f, 3.5575584f, 5.014459f, 5.3379144f, 4.3814994f, 2.5787734f,
      -0.021631265f, 0.1594025f, 1.0213817f, 2.9718205f, 5.5339018f, 7.4154889f, 7.5202313f, 5.7418745f, 3.1130463f,
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 0.15717715f, 0.54336716f, 1.1581471f, 1.8591082f, 2.4045727f, 2.5688185f, 2.2673639f, 1.6110487f,
      -0.011899905f, 0.12947208f, 0.63891136f, 1.7815208f, 3.4063984f, 4.8728995f, 5.2856435f, 4.3677134f, 2.6052934f,
      -0.021628401f, 0.15915684f, 1.0202237f, 2.9688732f, 5.5289454f, 7.4093702f, 7.5145789f, 5.7382228f, 3.1116338f,
  };
};

} // namespace hdsp::modules
