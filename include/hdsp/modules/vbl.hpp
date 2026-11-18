// hearth-dsp — JClones VBL tube broadcast limiter
// SPDX-License-Identifier: GPL-3.0-or-later
//
// C++ translation of the MIT-licensed JClones VBL JSFX. It combines a
// two-band feedback compressor with a tube saturator, transformer phase
// rotator and crosstalk. Stereo link and transformer are always on.
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct VBLControls {
  float input = 0.5f;      // 0..1 -> -24..+24 dB
  float comp = 0.0f;       // 0..1 -> 0..48 dB
  float emphasis = 0.5f;   // 0..1 -> 0..100 %
  float bias = 0.5f;       // 0..1 -> 0..100 %
  float amp = 0.0f;        // 0..1 -> 0..100 %
  float brilliance = 0.0f; // 0..1 -> 0..100 %
  float dryMix = 0.0f;     // 0..1 -> 0..100 %
  float output = 0.5f;     // 0..1 -> -24..+24 dB
};

class VBL {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    emphasis1_ = designLs(415.0f / sr_, 0.5f, 0.25f);
    emphasis2_ = designHs(1500.0f / sr_, 0.49f, 3.802f);
    crossoverK_ = filter1Lpf(2611.0f / sr_);
    compGrFilter1K_ = filter1Lpf(15.9f / sr_);
    compGrFilter2_ = designLp(500.0f / sr_, 0.72f);
    attackLoK_ = 1.0f - 2.0f * kPi / (2.0f * kPi + sr_ * 21.55685f / 1000.0f);
    attackHiK_ = 1.0f - 2.0f * kPi / (2.0f * kPi + sr_ * 10.77843f / 1000.0f);

    satScMixK_ = filter1Lpf(32.0f / sr_);
    satCrossoverK_ = filter1Lpf(2175.0f / sr_);
    satGrFilter_ = designLp(3000.0f / sr_, 0.72f);
    satCrosstalk_ = designHs(7000.0f / sr_, 0.5f, 10.0f);

    setTrafo(true);
    reset();
    dirty_ = true;
  }

  void setControls(const VBLControls& c) noexcept {
    inputGain_ = dbToGain(-24.0f + 48.0f * clamp01(c.input));
    compDb_ = 48.0f * clamp01(c.comp);
    emphasisK_ = clamp01(c.emphasis);
    biasK_ = clamp01(c.bias);
    ampK_ = clamp01(c.amp);
    brillianceK_ = clamp01(c.brilliance);
    dryMixK_ = clamp01(c.dryMix);
    outputGain_ = dbToGain(-24.0f + 48.0f * clamp01(c.output));
    dirty_ = true;
  }

  inline void process(float& l, float& r) noexcept {
    updateParameters();
    const float dryL = l, dryR = r;

    l *= inputGain_;
    r *= inputGain_;
    l += ch_[0].sat.feedbackAmp * ampK_;
    r += ch_[1].sat.feedbackAmp * ampK_;

    const float thisL = ch_[0].comp.preprocess(*this);
    const float thisR = ch_[1].comp.preprocess(*this);
    l = ch_[0].comp.process(l, thisL, thisR, *this);
    r = ch_[1].comp.process(r, thisR, thisL, *this);

    ch_[0].sat.preprocess(ch_[0].comp.sc, *this);
    ch_[1].sat.preprocess(ch_[1].comp.sc, *this);

    const float rmsLo =
        std::sqrt((ch_[0].sat.rmsLo * ch_[0].sat.rmsLo +
                   ch_[1].sat.rmsLo * ch_[1].sat.rmsLo) *
                  0.5f);
    const float rmsHi =
        std::sqrt((ch_[0].sat.rmsHi * ch_[0].sat.rmsHi +
                   ch_[1].sat.rmsHi * ch_[1].sat.rmsHi) *
                  0.5f);
    ch_[0].sat.rmsLo = ch_[1].sat.rmsLo = rmsLo;
    ch_[0].sat.rmsHi = ch_[1].sat.rmsHi = rmsHi;

    l = ch_[0].sat.postprocess(l, *this);
    r = ch_[1].sat.postprocess(r, *this);

    l = dryL * dryMixK_ + (1.0f - dryMixK_) * l * outputGain_;
    r = dryR * dryMixK_ + (1.0f - dryMixK_) * r * outputGain_;

    const float tL = ch_[0].sat.applyTrafo(l, r, *this);
    const float tR = ch_[1].sat.applyTrafo(r, tL, *this);
    l = tL;
    r = tR;

    lastGrDb_ = std::min(
        60.0f, -gainToDb(std::max(std::min(ch_[0].sat.feedbackGr,
                                           ch_[1].sat.feedbackGr),
                                  1.0e-3f)));
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
  static float filter1Lpf(float freqK) noexcept {
    return 1.0f - std::exp(-(2.0f * kPi * freqK));
  }

  struct Coeffs {
    float a0 = 1, a1 = 0, a2 = 0, b1 = 0, b2 = 0;
  };

  struct Biquad {
    float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    void reset() noexcept { x1 = x2 = y1 = y2 = 0.0f; }
    void resetTo(float v) noexcept { x1 = x2 = y1 = y2 = v; }
    inline float process(float x, const Coeffs& c) noexcept {
      const float y = x * c.a0 + x1 * c.a1 + x2 * c.a2 + y1 * c.b1 + y2 * c.b2;
      x2 = x1;
      x1 = x;
      y2 = y1;
      y1 = y;
      return y;
    }
  };

  static Coeffs designLp(float freqK, float q) noexcept {
    const float w0 = freqK * 2.0f * kPi;
    const float kk = 1.0f / std::tan(w0 / 2.0f);
    const float kk2 = kk * kk;
    const float b0 = kk2 + kk / q + 1.0f;
    const float b1 = 2.0f * (1.0f - kk2);
    const float b2 = kk2 - kk / q + 1.0f;
    Coeffs c;
    c.a0 = 1.0f / b0;
    c.a1 = 2.0f / b0;
    c.a2 = 1.0f / b0;
    c.b1 = -b1 / b0;
    c.b2 = -b2 / b0;
    return c;
  }

  static Coeffs designHp(float freqK, float q) noexcept {
    const float omega0 = freqK * 2.0f * kPi;
    const float alpha = std::sin(omega0) / (2.0f * q);
    const float cosw = std::cos(omega0);
    Coeffs c;
    c.a0 = ((1.0f + cosw) / 2.0f) / (1.0f + alpha);
    c.a1 = -(1.0f + cosw) / (1.0f + alpha);
    c.a2 = ((1.0f + cosw) / 2.0f) / (1.0f + alpha);
    c.b1 = -(-2.0f * cosw) / (1.0f + alpha);
    c.b2 = -(1.0f - alpha) / (1.0f + alpha);
    return c;
  }

  static Coeffs designLs(float freqK, float q, float a) noexcept {
    a = std::sqrt(a);
    const float w0 = freqK * 2.0f * kPi;
    const float alpha = std::sin(w0) / (2.0f * q);
    const float cosw = std::cos(w0);
    const float sq = std::sqrt(a);
    const float a0 = a * ((a + 1.0f) - (a - 1.0f) * cosw + 2.0f * sq * alpha);
    const float a1 = 2.0f * a * ((a - 1.0f) - (a + 1.0f) * cosw);
    const float a2 = a * ((a + 1.0f) - (a - 1.0f) * cosw - 2.0f * sq * alpha);
    const float b0 = (a + 1.0f) + (a - 1.0f) * cosw + 2.0f * sq * alpha;
    const float b1 = -2.0f * ((a - 1.0f) + (a + 1.0f) * cosw);
    const float b2 = (a + 1.0f) + (a - 1.0f) * cosw - 2.0f * sq * alpha;
    Coeffs c;
    c.a0 = a0 / b0;
    c.a1 = a1 / b0;
    c.a2 = a2 / b0;
    c.b1 = -b1 / b0;
    c.b2 = -b2 / b0;
    return c;
  }

  static Coeffs designHs(float freqK, float q, float a) noexcept {
    a = std::sqrt(a);
    const float w0 = freqK * 2.0f * kPi;
    const float alpha = std::sin(w0) / (2.0f * q);
    const float cosw = std::cos(w0);
    const float sq = std::sqrt(a);
    const float a0 = a * ((a + 1.0f) + (a - 1.0f) * cosw + 2.0f * sq * alpha);
    const float a1 = -2.0f * a * ((a - 1.0f) + (a + 1.0f) * cosw);
    const float a2 = a * ((a + 1.0f) + (a - 1.0f) * cosw - 2.0f * sq * alpha);
    const float b0 = (a + 1.0f) - (a - 1.0f) * cosw + 2.0f * sq * alpha;
    const float b1 = 2.0f * ((a - 1.0f) - (a + 1.0f) * cosw);
    const float b2 = (a + 1.0f) - (a - 1.0f) * cosw - 2.0f * sq * alpha;
    Coeffs c;
    c.a0 = a0 / b0;
    c.a1 = a1 / b0;
    c.a2 = a2 / b0;
    c.b1 = -b1 / b0;
    c.b2 = -b2 / b0;
    return c;
  }

  static float fastTanh(float x) noexcept {
    const float x2 = x * x;
    return ((((x2 + 378.0f) * x2 + 17325.0f) * x2 + 135135.0f) * x) /
           (((x2 * 28.0f + 3150.0f) * x2 + 62370.0f) * x2 + 135135.0f);
  }

  struct OctFilter {
    float b0 = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0, b6 = 0, b7 = 0;
    void reset() noexcept { b0 = b1 = b2 = b3 = b4 = b5 = b6 = b7 = 0.0f; }
    inline float process(float x) noexcept {
      b0 = x;
      b1 = b1 * 0.99886f + b0 * 0.0555179f;
      b2 = b2 * 0.99332f + b0 * 0.0750759f;
      b3 = b3 * 0.969f + b0 * 0.153852f;
      b4 = b4 * 0.8665f + b0 * 0.310486f;
      b5 = b5 * 0.55f + b0 * 0.532952f;
      b6 = b6 * -0.7616f + b0 * 0.016898f;
      const float y = b1 + b2 + b3 + b4 + b5 + b6 + b7 + b0 * 0.5362f;
      b7 = b0 * 0.115926f;
      return y;
    }
  };

  struct Trafo {
    float c1 = 0, c2 = 0, c3 = 0, c4 = 0, c5 = 0, c6 = 0, c7 = 0, c8 = 0;
    void reset() noexcept { c1 = c2 = c3 = c4 = c5 = c6 = c7 = c8 = 0.0f; }
    inline float process(float x) noexcept {
      c1 = c2 - 0.9698154f * x;
      c2 = x + 0.9698154f * c1;
      c3 = c4 - 0.9698154f * c1;
      c4 = c1 + 0.9698154f * c3;
      c5 = c6 - 0.9698154f * c3;
      c6 = c3 + 0.9698154f * c5;
      c7 = c8 - 0.9698154f * c5;
      c8 = c5 + 0.9698154f * c7;
      return c7;
    }
  };

  struct Comp {
    float c1 = 0, c2 = 0, c3 = 0;
    float crossover1 = 0, crossover2 = 0;
    float grLo = 1, grHi = 1;
    float arLo = 0, arHi = 0;
    float feedbackY = 0, feedbackGr = 1;
    float feedbackGrLo = 1, feedbackGrHi = 1;
    float sc = 0;
    Biquad scHpf, emph1, emph2, grFilter2;

    void reset() noexcept {
      crossover1 = crossover2 = 0.0f;
      grLo = grHi = 1.0f;
      arLo = arHi = 0.0f;
      feedbackY = 0.0f;
      feedbackGr = 1.0f;
      feedbackGrLo = feedbackGrHi = 1.0f;
      sc = 0.0f;
      scHpf.reset();
      emph1.reset();
      emph2.reset();
      grFilter2.resetTo(1.0f);
    }

    inline float preprocess(const VBL& o) noexcept {
      const float scIn = scHpf.process(feedbackY, o.compScHpf_);
      sc = scIn;
      float scSc = emph1.process(scIn, o.emphasis1_);
      scSc = emph2.process(scSc, o.emphasis2_);
      const float scSat = 2.778890f - (1.667f - scSc * 0.313f) * (1.667f - scSc * 0.313f);
      return (1.0f - feedbackGr) * scSat + feedbackGr * scIn;
    }

    inline float getDynamicRelease(float gr, float maxReleaseMs,
                                   const VBL& o) const noexcept {
      const float x = 7.0f * (gr - 1.0f);
      const float x2 = x * x;
      const float x4 = x2 * x2;
      const float y = (1680.0f + x * 840.0f + x2 * 180.0f + x2 * x * 20.0f + x4) /
                      (1680.0f - x * 840.0f + x2 * 180.0f - x2 * x * 20.0f + x4);
      const float releaseMs = std::max(y * maxReleaseMs, 30.0f);
      return 1.0f - 2.0f * kPi / (2.0f * kPi + o.sr_ * releaseMs / 1000.0f);
    }

    inline float feedbackGrUpdate(float scIn, const VBL& o) noexcept {
      crossover1 = crossover1 + (scIn - crossover1) * o.crossoverK_;
      crossover2 = crossover2 + (crossover1 - crossover2) * o.crossoverK_;
      const float scLo = crossover2;
      const float scHi = scIn - scLo;

      grLo = grLo + (feedbackGrLo * feedbackGrLo - grLo) * o.compGrFilter1K_;
      grHi = grHi + (feedbackGrHi * feedbackGrHi - grHi) * o.compGrFilter1K_;
      const float smoothLo = std::sqrt(std::max(grLo, 0.0f));
      const float smoothHi = std::sqrt(std::max(grHi, 0.0f));

      const float releaseKLo =
          getDynamicRelease(smoothLo, (2.0f - feedbackGrHi) * 14750.0f, o);
      const float releaseKHi = getDynamicRelease(smoothHi, 14750.0f, o);

      const float levelLo = std::fabs(scLo);
      const float levelHi = std::fabs(scHi);
      const float arKLo = levelLo > arLo ? o.attackLoK_ : releaseKLo;
      const float arKHi = levelHi > arHi ? o.attackHiK_ : releaseKHi;
      arLo = levelLo + arKLo * (arLo - levelLo);
      arHi = levelHi + arKHi * (arHi - levelHi);

      feedbackGrLo = std::min(c2 / (arLo + c3) + c1, 1.0f);
      feedbackGrHi = std::min(c2 / (arHi + c3) + c1, 1.0f);
      return std::sqrt((feedbackGrLo * feedbackGrLo + feedbackGrHi * feedbackGrHi) * 0.5f);
    }

    inline float process(float x, float thisSc, float thatSc,
                         const VBL& o) noexcept {
      const float pre = std::sqrt((thisSc * thisSc + thatSc * thatSc) * 0.5f);
      feedbackGr = feedbackGrUpdate(pre, o);
      feedbackY = grFilter2.process(feedbackGr, o.compGrFilter2_) * x;
      return feedbackY;
    }
  };

  struct Sat {
    float scMixS = 0;
    float crossover1 = 0, crossover2 = 0;
    float rmsLoS = 0, rmsHiS = 0;
    float rmsLo = 0, rmsHi = 0;
    float feedbackGr = 1;
    float hfS = 0;
    float hpfX1 = 0, hpfY1 = 0;
    float br1 = 0, br2 = 0;
    float feedbackAmp = 0;
    OctFilter emphasis;
    Biquad grFilter, crosstalk;
    Trafo trafo;

    void reset() noexcept {
      scMixS = crossover1 = crossover2 = 0.0f;
      rmsLoS = rmsHiS = rmsLo = rmsHi = 0.0f;
      feedbackGr = 1.0f;
      hfS = 0.0f;
      hpfX1 = hpfY1 = 0.0f;
      br1 = br2 = 0.0f;
      feedbackAmp = 0.0f;
      emphasis.reset();
      grFilter.reset();
      crosstalk.reset();
      trafo.reset();
    }

    inline void preprocess(float sc, const VBL& o) noexcept {
      float scEmph = emphasis.process(sc);
      scEmph = sc * 0.9f + scEmph * 0.1f;
      float scMix = 1.0f - feedbackGr;
      scMixS = scMixS + (scMix * scMix - scMixS) * o.satScMixK_;
      scMix = std::sqrt(std::max(scMixS, 0.0f));
      sc = sc * (1.0f - scMix) + scEmph * scMix;

      crossover1 = crossover1 + (sc - crossover1) * o.satCrossoverK_;
      crossover2 = crossover2 + (crossover1 - crossover2) * o.satCrossoverK_;
      const float lo = crossover2;
      const float hi = sc - lo;
      rmsLoS = rmsLoS + (lo * lo - rmsLoS) * o.rmsFilterK_;
      rmsHiS = rmsHiS + (hi * hi - rmsHiS) * o.rmsFilterK_;
      rmsLo = std::sqrt(std::max(rmsLoS, 0.0f));
      rmsHi = std::sqrt(std::max(rmsHiS, 0.0f));
    }

    inline float postprocess(float x, const VBL& o) noexcept {
      const float xLo = rmsLo * 4.0f;
      const float xHi = rmsHi * 4.0f;
      const float yLo = fastTanh(xLo);
      const float yHi = fastTanh(xHi);
      const float grLo = yLo / (xLo + 1.0e-18f);
      const float grHi = yHi / (xHi + 1.0e-18f);
      feedbackGr = (grLo + grHi) * 0.5f;
      const float gr = grFilter.process(feedbackGr, o.satGrFilter_);
      float y = x * gr;

      float delta = y - hfS;
      delta = std::min(std::max(delta, -0.5f), 0.5f);
      hfS += delta;
      y = hfS;

      float xx = std::min(std::max(y * 2.0f, -3.722950f), 3.352580f);
      y = (((xx * 9.0f) + 5.0f) * xx - 337.0f) * xx * -0.00277778f;
      y *= 0.5f;
      feedbackAmp = y;

      hpfY1 = (y - hpfX1) + o.hpfK_ * hpfY1;
      hpfX1 = y;
      y = hpfY1;

      br1 = br1 + (y - br1) * o.brillianceK_;
      br2 = br2 + (br1 - br2) * o.brillianceK_;
      y = br2 + (y - br2) * o.paramBrilliance_;
      return y;
    }

    inline float applyTrafo(float x, float thatX, const VBL& o) noexcept {
      float y = x + crosstalk.process(thatX, o.satCrosstalk_) * 0.001f;
      y = trafo.process(y);
      return y;
    }
  };

  struct Channel {
    Comp comp;
    Sat sat;
    void reset() noexcept {
      comp.reset();
      sat.reset();
    }
  };

  void setTrafo(bool on) noexcept {
    trafoOn_ = on;
    const float hpfFreqK = (on ? 19.0f : 6.4f) / sr_;
    const float cw = std::cos(2.0f * kPi * hpfFreqK);
    hpfK_ = cw - std::sqrt(cw * cw - 4.0f * cw + 3.0f);
  }

  void updateParameters() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    compScHpf_ = designHp((40.0f + emphasisK_ * 120.0f) / sr_, 0.70710678f);
    c1_ = dbToGain(-0.9f * compDb_ - 24.843803f);
    c2_ = dbToGain(compDb_ * compDb_ * 0.00727f - 1.0138f * compDb_ + 0.602056f);
    c3_ = c2_ / (1.0f - c1_) - dbToGain(-compDb_);

    const float biasTauMs = biasK_ * (2.283f - 7.000f) + 7.000f;
    rmsFilterK_ = filter1Lpf((1000.0f / biasTauMs) / sr_);
    paramBrilliance_ = dbToGain(brillianceK_ * 18.0f);
    const float brillianceFreq =
        34540.0f * ampK_ * ampK_ * ampK_ - 57183.0f * ampK_ * ampK_ +
        3594.0f * ampK_ + 24504.0f;
    brillianceK_ = filter1Lpf(brillianceFreq / sr_);

    for (auto& ch : ch_) {
      ch.comp.c1 = c1_;
      ch.comp.c2 = c2_;
      ch.comp.c3 = c3_;
    }
  }

  float sr_ = 48000.0f;
  float inputGain_ = 1.0f, outputGain_ = 1.0f;
  float compDb_ = 0.0f, emphasisK_ = 0.5f, biasK_ = 0.5f, ampK_ = 0.0f;
  float brillianceK_ = 0.0f, dryMixK_ = 0.0f;
  bool dirty_ = true;
  bool trafoOn_ = true;

  float c1_ = 0, c2_ = 0, c3_ = 0;
  float crossoverK_ = 0.3f, compGrFilter1K_ = 0.01f, attackLoK_ = 0.99f,
        attackHiK_ = 0.99f;
  float satScMixK_ = 0.01f, satCrossoverK_ = 0.25f, rmsFilterK_ = 0.01f;
  float hpfK_ = 0.99f, paramBrilliance_ = 1.0f;
  float lastGrDb_ = 0.0f;

  Coeffs emphasis1_, emphasis2_, compGrFilter2_, satGrFilter_, satCrosstalk_,
      compScHpf_;
  Channel ch_[2];
};

} // namespace hdsp::modules
