// hearth-dsp — JClones CL-1B optical compressor
// SPDX-License-Identifier: GPL-3.0-or-later
//
// C++ translation of the MIT-licensed JClones CL1B JSFX. It follows the
// measured optical detector tables, the fixed/manual attack-release modes and
// the 20 kHz post-EQ of the original.
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct CL1BControls {
  float ratio = 0.5f;      // 0..1 -> 2..10 : 1
  float threshold = 0.8f;  // 0..1 -> -40..+10 dB
  float attack = 0.5f;     // 0..1 -> 0..10
  float release = 0.5f;    // 0..1 -> 0..10
  int mode = 2;            // 0 fixed, 1 fixed/manual, 2 manual
  float output = 0.5f;     // 0..1 -> -24..+24 dB
};

class CL1B {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    lpf1Attack_ = std::exp(-1.0f / (sr_ * 0.001324200f));
    lpf1Release_ = std::exp(-1.0f / (sr_ * 0.001782562f));
    lpf2Attack_ = std::exp(-1.0f / (sr_ * 0.028011420f));
    lpf2Release_ = std::exp(-1.0f / (sr_ * 0.026260180f));
    releaseK_ = std::exp(-1.0f / (sr_ * 5.898f));
    postEqK_ = 1.0f - std::exp(-2.0f * kPi * (20000.0f / sr_));
    reset();
    dirty_ = true;
  }

  void setControls(const CL1BControls& c) noexcept {
    ratio_ = clamp01(c.ratio);
    thresholdDb_ = -40.0f + 50.0f * clamp01(c.threshold);
    attack_ = clamp01(c.attack);
    release_ = clamp01(c.release);
    mode_ = std::clamp(c.mode, 0, 2);
    outputDb_ = -24.0f + 48.0f * clamp01(c.output);
    dirty_ = true;
  }

  inline void process(float& l, float& r) noexcept {
    updateParameters();
    float y1, y2;
    processStereo(l, r, y1, y2);
    l = y1;
    r = y2;
  }

  void reset() noexcept {
    for (auto& ch : ch_) ch.reset();
    lastGrDb_ = 0.0f;
  }

  int latencySamples() const noexcept { return 0; }
  float lastGrDb() const noexcept { return lastGrDb_; }

private:
  static constexpr float kPi = 3.14159265358979323846f;

  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }
  static float clampSigned(float x) noexcept {
    return std::min(std::max(x, -0.99999988f), 0.99999988f);
  }

  static float interpLin(float x, const float* table, int size) noexcept {
    x = clampSigned(x);
    if (x < 0.0f) x = 0.0f;
    x *= static_cast<float>(size - 1);
    const int idx = static_cast<int>(std::floor(x));
    if (idx == size - 1) return table[size - 1];
    const float frac = x - static_cast<float>(idx);
    return table[idx] * (1.0f - frac) + table[idx + 1] * frac;
  }

  static float interpExp(float x, const float* table) noexcept {
    x = clampSigned(x);
    if (x * 1000000.0f == 0.0f) return table[23];
    int exp = static_cast<int>(std::ceil(std::log(std::fabs(x)) / kLn2));
    float mant = x / std::pow(2.0f, static_cast<float>(exp));
    if (mant == 1.0f) {
      mant = 0.5f;
      ++exp;
    }
    int index = 1 - exp;
    float frac;
    if (index < 0) {
      exp = 0;
    }
    if (index > 22) {
      frac = mant * std::pow(2.0f, static_cast<float>(22 + exp));
      index = 23;
    } else {
      frac = mant <= 0.0f ? (mant + 0.5f) * 2.0f : (mant - 0.5f) * 2.0f;
    }
    return frac * (table[index] - table[index + 1]) + table[index + 1];
  }

  // Signed-magnitude table with unity at index 23.
  static float interpExpNeg(float x, const float* table) noexcept {
    const float* t = table + 23;
    x = clampSigned(x);
    if (x * 1000000.0f == 0.0f) return t[23];
    int exp = static_cast<int>(std::ceil(std::log(std::fabs(x)) / kLn2));
    float mant = x / std::pow(2.0f, static_cast<float>(exp));
    if (std::fabs(mant) == 1.0f) {
      mant *= 0.5f;
      ++exp;
    }
    int index = 1 - exp;
    float frac;
    if (index < 0) {
      exp = 0;
    }
    if (index > 22) {
      frac = mant * std::pow(2.0f, static_cast<float>(22 + exp));
      index = 23;
    } else {
      frac = mant <= 0.0f ? (mant + 0.5f) * 2.0f : (mant - 0.5f) * 2.0f;
    }
    if (x < 0.0f) {
      index = -index;
      frac += 1.0f;
    }
    return frac * (t[index] - t[index + 1]) + t[index + 1];
  }

  void updateParameters() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    t3_ = interpExp(ratio_, kTable3);
    t10_ = interpExp(ratio_, kTable10);
    t8_ = interpLin(attack_, kTable8, 46) * (44100.0f / sr_);
    t9_ = interpLin(release_, kTable9, 24) * (44100.0f / sr_);
    t4_ = interpLin(t3_, kTable4, 24);
    t7_ = dbToGain(-40.0f - thresholdDb_);
    t11_ = dbToGain(-30.0f + outputDb_);
  }

  struct Channel {
    float lpf1 = 0.0f;
    float lpf2 = 0.0f;
    float level = 0.0f;
    float post1 = 0.0f;
    float post2 = 0.0f;
    void reset() noexcept {
      lpf1 = lpf2 = level = post1 = post2 = 0.0f;
    }
  };

  void processStereo(float x1, float x2, float& y1, float& y2) noexcept {
    const float invGr = ch_[0].lpf1 * 0.2998201f + ch_[0].lpf2 * 0.079904087f;
    const float gainReduction =
        0.0029900903f / std::min(invGr + 0.0029900903f, 0.99999988f);

    const float t5 = interpExp(invGr, kTable5);
    const float t6 = interpLin(t5, kTable6, 24);
    const float m1 = 0.01193628f * t6 + 0.9323384f * (1.0f - t6);
    const float m2 = 0.4595526f * t6 + 1.0f * (1.0f - t6);
    const float mult = m1 * (1.0f - t4_) + m2 * t4_;
    const float t2On = 0.08098298f;

    const float in1 = interpExpNeg((x1 * t2On) * mult * t7_, kTable12);
    const float in2 = interpExpNeg((x2 * t2On) * mult * t7_, kTable12);
    const float level = std::max(std::fabs(in1), std::fabs(in2));

    float levelState = ch_[0].level;
    if (mode_ != 0) {
      if (level >= levelState) {
        levelState = std::min(levelState + t8_, level);
      } else {
        levelState = std::max(levelState * releaseK_ - t9_, level);
      }
      ch_[0].level = levelState;
    }

    float combined;
    if (mode_ == 0) {
      combined = level;
    } else if (mode_ == 2) {
      combined = levelState;
    } else {
      combined = std::max(level, levelState);
    }

    const float t13 = interpLin(combined, kTable13, 252);
    float k1 = t13 > ch_[0].lpf1 ? lpf1Attack_ : lpf1Release_;
    ch_[0].lpf1 = t13 + (ch_[0].lpf1 - t13) * k1;
    float k2 = t13 > ch_[0].lpf2 ? lpf2Attack_ : lpf2Release_;
    ch_[0].lpf2 = t13 + (ch_[0].lpf2 - t13) * k2;

    const float gain = t10_ * t11_ * gainReduction * 33.768673f;
    y1 = x1 * gain;
    y2 = x2 * gain;

    ch_[0].post1 += (y1 - ch_[0].post1) * postEqK_;
    ch_[1].post2 += (y2 - ch_[1].post2) * postEqK_;
    y1 = ch_[0].post1;
    y2 = ch_[1].post2;

    lastGrDb_ = -gainToDb(std::max(gainReduction, 1.0e-6f));
  }

  static constexpr float kLn2 = 0.6931471805599453f;

  // Measured tables from the JClones CL1B JSFX.
  static constexpr float kTable3[25] = {
      0.999999f, 0.99f, 0.5626293f, 0.2993541f, 0.1536661f, 0.07558671f,
      0.036547f, 0.01702715f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f,
      0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f,
      0.01f, 0.01f};
  static constexpr float kTable4[24] = {
      0.0f, 0.03416149f, 0.07852706f, 0.1228926f, 0.1672582f, 0.2116238f,
      0.2559893f, 0.3003549f, 0.3447205f, 0.3890861f, 0.4334517f, 0.4778172f,
      0.5221828f, 0.5665483f, 0.6109139f, 0.6552795f, 0.6996451f, 0.7440106f,
      0.7883762f, 0.8327418f, 0.8771074f, 0.921473f, 0.9658385f, 0.999999f};
  static constexpr float kTable5[25] = {
      0.01f, 1.0f, 0.9947661f, 0.9844928f, 0.9651101f, 0.9302186f,
      0.8630559f, 0.755419f, 0.6082814f, 0.4397123f, 0.2796561f, 0.162245f,
      0.08780019f, 0.04508f, 0.02209106f, 0.01019185f, 0.004130001f,
      0.001069335f, 0.00001f, 0.00001f, 0.00001f, 0.00001f, 0.00001f,
      0.00001f, 0.00001f};
  static constexpr float kTable6[24] = {
      0.0f, 0.0434687f, 0.08694739f, 0.1304261f, 0.1739048f, 0.2173835f,
      0.2608622f, 0.3043409f, 0.3478196f, 0.3912983f, 0.434777f, 0.4782557f,
      0.5217344f, 0.565213f, 0.6086918f, 0.6521704f, 0.6956491f, 0.7391278f,
      0.7826065f, 0.8260852f, 0.8695639f, 0.9130426f, 0.9565213f, 0.999999f};
  static constexpr float kTable8[46] = {
      0.002257127f, 0.002257127f, 0.002257127f, 0.002257127f, 0.002257127f,
      0.002257127f, 0.002257127f, 0.002257127f, 0.002257127f, 0.002257127f,
      0.002257127f, 0.000807641f, 0.0002590034f, 0.0001466583f, 0.000105361f,
      0.00008688696f, 0.00007712693f, 0.00007082194f, 0.00006535164f,
      0.00005942077f, 0.00005248035f, 0.00004474115f, 0.00003699339f,
      0.00002985739f, 0.00002377450f, 0.00001915936f, 0.00001565825f,
      0.00001302099f, 0.00001102717f, 0.000009554097f, 0.000008418394f,
      0.000007519858f, 0.000006788958f, 0.000006188009f, 0.000005677829f,
      0.000005232528f, 0.000004838532f, 0.000004491788f, 0.000004193505f,
      0.000003938723f, 0.000003725471f, 0.000003552736f, 0.000003421820f,
      0.000003326748f, 0.000003267550f, 0.000003245660f};
  static constexpr float kTable9[24] = {
      0.00004848326f, 0.00004848326f, 0.00004848326f, 0.00004188835f,
      0.00002785662f, 0.00001560057f, 0.00001201397f, 0.000008427365f,
      0.000005328864f, 0.000004453937f, 0.000003579009f, 0.000002704082f,
      0.000002101815f, 0.000001772209f, 0.000001442603f, 0.000001112997f,
      0.0000008481028f, 0.0000006281776f, 0.0000004082524f, 0.0000002060375f,
      0.0000001126142f, 0.00000001919095f, 0.0000000003280834f,
      -0.000000004332800f};
  static constexpr float kTable10[25] = {
      0.8766871f, 0.8766871f, 0.9343757f, 0.966794f, 0.9838194f, 0.9926132f,
      0.9970101f, 0.9992085f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
  static constexpr float kTable12[48] = {
      0.000002987261f, 0.000005974523f, 0.00001194905f, 0.00002389809f,
      0.00004779618f, 0.00009559237f, 0.0001911847f, 0.0003823695f,
      0.0007647389f, 0.001529478f, 0.003058956f, 0.006117912f, 0.01223582f,
      0.02447165f, 0.04894329f, 0.09788658f, 0.1957732f, 0.3915463f,
      0.7830927f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 0.7810927f, 0.3895463f, 0.1937732f, 0.09588659f, 0.04694329f,
      0.02247165f, 0.01023582f, 0.004117912f, 0.001058956f, 0.0f, 0.0f,
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  static constexpr float kTable13[252] = {
      0.0f, 0.002895139f, 0.01001967f, 0.01859283f, 0.0278201f, 0.03739988f,
      0.04719125f, 0.05711957f, 0.06714159f, 0.07723056f, 0.087369f,
      0.09754504f, 0.1077503f, 0.1179788f, 0.1282261f, 0.1384886f, 0.1487638f,
      0.1590496f, 0.1693444f, 0.1796468f, 0.1899559f, 0.2002707f, 0.2105905f,
      0.2209146f, 0.2312426f, 0.2415741f, 0.2519086f, 0.2622458f, 0.2725855f,
      0.2829274f, 0.2932713f, 0.303617f, 0.3139643f, 0.3243132f, 0.3346635f,
      0.345015f, 0.3553677f, 0.3657215f, 0.3760763f, 0.3864319f, 0.3967885f,
      0.4071458f, 0.4175039f, 0.4278626f, 0.4382221f, 0.4485821f, 0.4589426f,
      0.4693037f, 0.4796653f, 0.4900274f, 0.5003899f, 0.5107529f, 0.5211161f,
      0.5314798f, 0.5418439f, 0.5522082f, 0.562573f, 0.5729379f, 0.5833032f,
      0.5936688f, 0.6040345f, 0.6144006f, 0.6247668f, 0.6351333f, 0.6455f,
      0.6558669f, 0.666234f, 0.6766013f, 0.6869688f, 0.6973364f, 0.7077042f,
      0.7180721f, 0.7284402f, 0.7388085f, 0.7491769f, 0.7595453f, 0.769914f,
      0.7802827f, 0.7906516f, 0.8010206f, 0.8113897f, 0.8217589f, 0.8321282f,
      0.8424976f, 0.8528671f, 0.8632367f, 0.8736063f, 0.883976f, 0.8943459f,
      0.9047158f, 0.9150858f, 0.9254559f, 0.935826f, 0.9461962f, 0.9565665f,
      0.9669368f, 0.9773072f, 0.9876777f, 0.9980482f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

  float sr_ = 48000.0f;
  float ratio_ = 0.5f;
  float thresholdDb_ = 0.0f;
  float attack_ = 0.5f;
  float release_ = 0.5f;
  int mode_ = 2;
  float outputDb_ = 0.0f;
  bool dirty_ = true;

  float t3_ = 1.0f, t4_ = 0.0f, t7_ = 1.0f, t8_ = 0.0f, t9_ = 0.0f,
        t10_ = 1.0f, t11_ = 1.0f;
  float lpf1Attack_ = 0.99f, lpf1Release_ = 0.99f;
  float lpf2Attack_ = 0.99f, lpf2Release_ = 0.99f;
  float releaseK_ = 0.99f, postEqK_ = 0.9f;
  float lastGrDb_ = 0.0f;
  Channel ch_[2];
};

} // namespace hdsp::modules
