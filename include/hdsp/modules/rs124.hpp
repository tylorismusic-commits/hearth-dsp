// hearth-dsp — JClones RS-124 vari-mu compressor
// SPDX-License-Identifier: GPL-3.0-or-later
//
// C++ translation of the MIT-licensed JClones RS124 JSFX. It models three
// vari-mu hardware revisions with measured gain, release and gain-reduction
// tables, cubic-spline interpolation and a feedback detector.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <hdsp/db.hpp>

namespace hdsp::modules {

struct RS124Controls {
  float model = 0.0f;     // 0..1 -> 0..2 (60050A / 61010B / 60070B)
  float input = 0.5f;     // 0..1 -> -24..+24 dB
  float release = 0.6f;   // 0..1 -> position 1..6
  bool superFuse = false;
  float output = 1.0f;    // 0..1 -> -30..0
  bool boxTone = true;
};

class RS124 {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    buildTables();
    reset();
    dirty_ = true;
  }

  void setControls(const RS124Controls& c) noexcept {
    model_ = std::clamp(static_cast<int>(std::lround(clamp01(c.model) * 2.0f)), 0, 2);
    inputPos_ = 10.0f * clamp01(c.input);
    releasePos_ = 1.0f + 5.0f * clamp01(c.release);
    superFuse_ = c.superFuse;
    outputPos_ = -30.0f + 30.0f * clamp01(c.output);
    boxTone_ = c.boxTone;
    dirty_ = true;
  }

  inline void process(float& l, float& r) noexcept {
    updateParameters();
    l = ch_[0].process(l, *this);
    r = ch_[1].process(r, *this);
    lastGrDb_ = -gainToDb(std::max(std::min(ch_[0].feedbackGr,
                                             ch_[1].feedbackGr),
                                   1.0e-6f));
  }

  void reset() noexcept {
    for (auto& ch : ch_) ch.reset(sr_);
    lastGrDb_ = 0.0f;
  }

  int latencySamples() const noexcept { return 0; }
  float lastGrDb() const noexcept { return lastGrDb_; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  struct Segment {
    float a = 0, b = 0, c = 0, d = 0;
  };

  struct Spline {
    std::vector<float> x, y;
    std::vector<Segment> seg;

    void build(std::vector<float> xs, std::vector<float> ys) {
      x = std::move(xs);
      y = std::move(ys);
      const int n = static_cast<int>(x.size());
      seg.assign(static_cast<size_t>(std::max(1, n - 1)), Segment{});
      for (int i = 0; i < n - 1; ++i) {
        const float x1 = x[static_cast<size_t>(i)];
        const float x2 = x[static_cast<size_t>(i + 1)];
        const float y1 = y[static_cast<size_t>(i)];
        const float y2 = y[static_cast<size_t>(i + 1)];
        float dy1, dy2;
        if (i - 1 >= 0) {
          const float x0 = x[static_cast<size_t>(i - 1)];
          const float y0 = y[static_cast<size_t>(i - 1)];
          dy1 = (y2 - y0) / (x2 - x0);
        } else {
          dy1 = (y2 - y1) / (x2 - x1);
        }
        if (i + 2 < n) {
          const float x3 = x[static_cast<size_t>(i + 2)];
          const float y3 = y[static_cast<size_t>(i + 2)];
          dy2 = (y3 - y1) / (x3 - x1);
        } else {
          dy2 = (y2 - y1) / (x2 - x1);
        }
        const float dx = x2 - x1;
        const float dy = y2 - y1;
        const float sum = dy1 + dy2;
        const float dx2 = dx * dx;
        const float dx3 = dx2 * dx;
        Segment s;
        s.a = (dx * sum - 2.0f * dy) / dx3;
        s.b = -((dy1 + sum) * dx2 + (sum * x1 - dy) * 3.0f * dx -
                6.0f * x1 * dy) /
              dx3;
        s.c = (dy1 * dx3 + (dy1 + sum) * 2.0f * x1 * dx2 +
               (sum * x1 - 2.0f * dy) * 3.0f * x1 * dx -
               6.0f * x1 * x1 * dy) /
              dx3;
        s.d = -((x1 * dy1 - y1) * dx3 + (dy1 + sum) * x1 * x1 * dx2 +
                (sum * x1 - 3.0f * dy) * x1 * x1 * dx -
                2.0f * x1 * x1 * x1 * dy) /
              dx3;
        seg[static_cast<size_t>(i)] = s;
      }
      // Linear ends, matching the original table generator.
      auto setLine = [&](int i) {
        const float x0 = x[static_cast<size_t>(i)];
        const float x1 = x[static_cast<size_t>(i + 1)];
        const float y0 = y[static_cast<size_t>(i)];
        const float y1 = y[static_cast<size_t>(i + 1)];
        Segment s;
        s.a = 0.0f;
        s.b = 0.0f;
        s.c = (y1 - y0) / (x1 - x0);
        s.d = (x1 * y0 - y1 * x0) / (x1 - x0);
        seg[static_cast<size_t>(i)] = s;
      };
      if (n >= 2) {
        setLine(0);
        setLine(n - 2);
      }
    }

    float eval(float xv) const {
      if (x.size() < 2) return 0.0f;
      xv = std::min(x.back(), std::max(x.front(), xv));
      int left = 0;
      int right = static_cast<int>(x.size()) - 1;
      while (right - left > 1) {
        const int mid = (left + right) / 2;
        if (xv < x[static_cast<size_t>(mid)]) {
          right = mid;
        } else {
          left = mid;
        }
      }
      const Segment& s = seg[static_cast<size_t>(std::min(
          left, static_cast<int>(seg.size()) - 1))];
      return xv * xv * xv * s.a + xv * xv * s.b + xv * s.c + s.d;
    }
  };

  struct Filter2 {
    float a0 = 1, a1 = 0, a2 = 0, b1 = 0, b2 = 0;
    float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    void init(float na0, float na1, float na2, float nb1, float nb2) noexcept {
      a0 = na0;
      a1 = na1;
      a2 = na2;
      b1 = nb1;
      b2 = nb2;
    }
    void copyCoeffs(const Filter2& o) noexcept {
      a0 = o.a0;
      a1 = o.a1;
      a2 = o.a2;
      b1 = o.b1;
      b2 = o.b2;
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

  void updateParameters() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    inputGainK_ = dbToGain(inputGain_[static_cast<size_t>(model_)].eval(inputPos_));
    outputGainK_ =
        dbToGain(outputGain_[static_cast<size_t>(model_)].eval(outputPos_) + 30.0f);
    attackK_ = filterCoefficient(attackTimeSec_[static_cast<size_t>(model_)]);
    releaseK_ = superFuse_
                    ? attackK_
                    : filterCoefficient(
                          releaseTime_[static_cast<size_t>(model_)].eval(releasePos_));

    if (model_ == 2) {
      designPk(62.0f / sr_, 0.0f, 1.0f);
      designHpf1(10.0f / sr_);
    } else {
      designPk(62.0f / sr_, 0.8f, 1.0f);
      designHpf1(32.9f / sr_);
    }
    if (model_ != 0) {
      designHs(13000.0f / sr_, 4.0f, 0.6f);
    } else {
      designHs(12000.0f / sr_, 2.8f, 0.6f);
    }
    for (auto& ch : ch_) ch.setCoeffs(hpf_, pk_, hs_, sc_);
  }

  float filterCoefficient(float timeSec) const noexcept {
    return std::pow(0.2f, 1.0f / (timeSec * sr_));
  }

  void designPk(float freqK, float aDb, float q) noexcept {
    if (aDb != 0.0f) {
      const float aLin = dbToGain(aDb);
      const float saLin = dbToGain(aDb / 2.0f);
      const float w0 = freqK * 2.0f * kPi;
      const float x1 = std::sinh(std::log(std::sqrt(1.0f / (q + q) * (1.0f / (q + q)) + 1.0f) +
                                          1.0f / (q + q)) *
                                  (std::sin(w0) / w0));
      const float x1w02 = x1 * w0 + x1 * w0;
      const float aLin2 = aLin * aLin;
      const float saLin2 = saLin * saLin;
      const float abs1 = std::fabs(aLin2 - 1.0f);
      const float abs2 = std::fabs(saLin2 - 1.0f);
      const float w02pi22 = (w0 * w0 - kPi * kPi) * (w0 * w0 - kPi * kPi);
      const float invAbs = 1.0f / std::fabs(aLin2 - saLin2);
      const float x2 = invAbs * kPi * kPi * abs2 * x1w02 * x1w02;
      const float x3 = std::sqrt((aLin * x2 * aLin + w02pi22) / (x2 + w02pi22));
      const float x32 = x3 * x3;
      const float abs3 = std::fabs(aLin2 - x32);
      const float abs4 = std::fabs(saLin2 - x32);
      const float tanW012 = std::tan(w0 * 0.5f);
      const float x4 = tanW012 * (std::sqrt(abs3 / abs1) * tanW012);
      const float x5 = (std::sqrt(abs2 / abs4) * x4 + 1.0f) * std::tan(x1w02 * 0.5f);
      const float abs5 = std::fabs(saLin2 - x3);
      const float sabs42 = std::sqrt(abs4 * abs2);
      const float x6 = x5 * x5 * abs4 - ((abs5 - sabs42) * x4 + (abs5 - sabs42) * x4);
      const float abs6 = std::fabs(aLin2 - x3);
      const float sabs31 = std::sqrt(abs3 * abs1);
      const float x7 = (abs6 - sabs31) * x4 + (abs6 - sabs31) * x4;
      const float x8 = std::sqrt((x7 + x6) * invAbs);
      const float x9 = std::sqrt((x7 * saLin * saLin + x6 * aLin * aLin) * invAbs);

      const float a0 = x4 + x9 + x3;
      const float a1 = (x3 - x4) * -2.0f;
      const float a2 = x4 + x3 - x9;
      const float b0 = x8 + x4 + 1.0f;
      const float b1 = (1.0f - x4) * -2.0f;
      const float b2 = x4 + 1.0f - x8;
      pk_.init(a0 / b0, a1 / b0, a2 / b0, -b1 / b0, -b2 / b0);
    } else {
      pk_.init(1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
  }

  void designHs(float freqK, float aDb, float q) noexcept {
    const float saLin = dbToGain(aDb / 2.0f);
    const float w0 = freqK * 2.0f * kPi;
    const float saLinM = saLin - 1.0f;
    const float cosW0 = std::cos(w0);
    const float sinW0 = std::sin(w0);
    const float xm = saLinM * cosW0;
    const float xs =
        sinW0 * std::sqrt((saLin * saLin + 1.0f) / q - saLinM * saLinM);
    const float saLinP = saLin + 1.0f;
    const float xp = saLinP * cosW0;
    const float xpm = saLinP - xm;
    const float a0 = (xs + saLinP + xm) * saLin;
    const float a1 = (xp + saLinM) * saLin * -2.0f;
    const float a2 = (saLinP + xm - xs) * saLin;
    const float b0 = xpm + xs;
    const float b1 = saLinM - xp + saLinM - xp;
    const float b2 = xpm - xs;
    hs_.init(a0 / b0, a1 / b0, a2 / b0, -b1 / b0, -b2 / b0);
  }

  void designHpf(float freqK, float q) noexcept {
    const float w0 = freqK * 2.0f * kPi;
    const float cosW0 = std::cos(w0);
    const float sinW0 = std::sin(w0);
    const float sh = std::sinh(std::log(2.0f) * (w0 / sinW0) * q * 0.5f) * sinW0;
    const float a0 = 0.5f * (cosW0 + 1.0f);
    const float a1 = -(cosW0 + 1.0f);
    const float a2 = a0;
    const float b0 = sh + 1.0f;
    const float b1 = cosW0 * -2.0f;
    const float b2 = 1.0f - sh;
    hpf_.init(a0 / b0, a1 / b0, a2 / b0, -b1 / b0, -b2 / b0);
  }

  void designHpf1(float freqK) noexcept {
    const float w0 = freqK * 2.0f * kPi;
    const float k = w0 / std::tan(w0 * 0.5f);
    const float a0 = k;
    const float a1 = -k;
    const float b0 = w0 + k;
    const float b1 = w0 - k;
    sc_.init(a0 / b0, a1 / b0, 0.0f, -b1 / b0, 0.0f);
  }

  struct Channel {
    Filter2 hpf, pk, hs, sc;
    float levelState = 0.0f;
    float feedbackGr = 1.0f;
    float startupSum = 0.0f;
    int startupCounter = 0;
    int startupMax = 1;

    void reset(float sr) noexcept {
      hpf.reset();
      pk.reset();
      hs.reset();
      sc.reset();
      levelState = 0.0f;
      feedbackGr = 1.0f;
      startupSum = 0.0f;
      startupCounter = 0;
      startupMax = std::max(1, static_cast<int>(sr * 0.300f));
    }

    void setCoeffs(const Filter2& h, const Filter2& p, const Filter2& hi,
                   const Filter2& s) noexcept {
      hpf.copyCoeffs(h);
      pk.copyCoeffs(p);
      hs.copyCoeffs(hi);
      sc.copyCoeffs(s);
    }

    inline float process(float x, const RS124& o) noexcept {
      float xc;
      if (o.boxTone_) {
        const float xa = hpf.process(x);
        const float xb = pk.process(xa);
        xc = hs.process(xb);
      } else {
        xc = x;
      }
      // Filters are shared coefficients but per-channel state.
      float y = xc * o.inputGainK_ * feedbackGr;
      const float scLevel = sc.process(y);
      const float level = getLevel(scLevel, o);
      feedbackGr = o.gr_[static_cast<size_t>(o.model_)].eval(level * 1.1f);
      return y * o.outputGainK_;
    }

    inline float getLevel(float x, const RS124& o) noexcept {
      float level = std::fabs(x);
      level = std::min(level, 1.0f);
      level = std::max(level, kMinLevel);
      const float filterK = level > levelState ? o.attackK_ : o.releaseK_;
      if (startupCounter >= startupMax) {
        levelState = (1.0f - filterK) * level + levelState * filterK;
      } else {
        startupSum += level * level;
        ++startupCounter;
        levelState = std::sqrt(startupSum / static_cast<float>(startupCounter));
      }
      return levelState;
    }
  };

  void buildTables() noexcept {
    static constexpr float inputX[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    static constexpr float inputY[3][11] = {
        {-90.5f, -49.4f, -35.5f, -27.6f, -24.7f, -22.1f, -16.6f, -9.8f, -5.6f, -2.1f, 0.0f},
        {-105.0f, -47.3f, -38.7f, -30.1f, -25.6f, -21.5f, -15.0f, -10.0f, -5.0f, -1.3f, 0.0f},
        {-92.5f, -48.9f, -33.8f, -26.6f, -23.6f, -21.3f, -16.3f, -11.1f, -6.0f, -3.0f, 0.0f}};
    static constexpr float outputX[] = {-30, -20, -15, -10, -5, 0};
    static constexpr float outputY[3][6] = {
        {-30.000f, -19.970f, -15.029f, -9.979f, -4.940f, -0.452f},
        {-30.000f, -20.060f, -15.163f, -10.239f, -5.110f, -0.470f},
        {-30.000f, -20.369f, -15.300f, -10.240f, -5.328f, -0.603f}};
    static constexpr float releaseX[] = {1, 2, 3, 4, 5, 6, 7};
    static constexpr float releaseY[3][7] = {
        {3.4f, 6.5f, 11.4f, 22.0f, 38.0f, 64.0f, 3000.0f},
        {2.9f, 5.0f, 8.1f, 15.6f, 25.3f, 44.0f, 3000.0f},
        {1.0f, 1.9f, 3.2f, 5.8f, 9.8f, 16.9f, 3000.0f}};
    static constexpr float grX[3][18] = {
        {0.0000000000000f, 0.0002106149709f, 0.0003686990847f, 0.0006501130364f,
         0.0011044513000f, 0.0016804788350f, 0.0023251490470f, 0.0029973624020f,
         0.0036842304960f, 0.0043689564300f, 0.0051289380440f, 0.0061411803430f,
         0.0074524766260f, 0.0089647505090f, 0.0108051531700f, 0.0135162656500f,
         0.0175188865400f, 1.0000000000000f},
        {0.0000000000000f, 0.0001822577969f, 0.0003190966185f, 0.0005625128294f,
         0.0009804053352f, 0.0015538258180f, 0.0022126659440f, 0.0028988540950f,
         0.0035634511280f, 0.0042128857880f, 0.0049078830270f, 0.0057280561550f,
         0.0068850359210f, 0.0083364521810f, 0.0104270474000f, 0.0134988681400f,
         0.0166703550600f, 1.0000000000000f},
        {0.0000000000000f, 0.0002118388963f, 0.0003683106940f, 0.0006512028181f,
         0.0011267457390f, 0.0017141596740f, 0.0023674152850f, 0.0030527255360f,
         0.0037581071970f, 0.0045289696780f, 0.0054706732940f, 0.0066954517650f,
         0.0081442878920f, 0.0096584563910f, 0.0111818552200f, 0.0139228675800f,
         0.0172579735900f, 1.0000000000000f}};
    static constexpr float grY[3][18] = {
        {1.19441334700f, 1.19441334700f, 1.17514954100f, 1.16456995300f,
         1.11193381000f, 0.95086977030f, 0.73942550540f, 0.53572080760f,
         0.37008517710f, 0.24665404760f, 0.16273974240f, 0.10951501340f,
         0.07469268520f, 0.05049766445f, 0.03420736713f, 0.02404922493f,
         0.01751888654f, 0.00000000000f},
        {1.03359767900f, 1.03359767900f, 1.01705228000f, 1.00764867400f,
         0.98704744970f, 0.87920535940f, 0.70365451860f, 0.51811434480f,
         0.35795275110f, 0.23784291470f, 0.15572572970f, 0.10214781390f,
         0.06900549260f, 0.04695851429f, 0.03301034542f, 0.02401826988f,
         0.01667035506f, 0.00000000000f},
        {1.20135432000f, 1.20135432000f, 1.17391162900f, 1.16652211600f,
         1.13437929100f, 0.96992748780f, 0.75286667980f, 0.54561590170f,
         0.37750617640f, 0.25568776440f, 0.17358290440f, 0.11939927650f,
         0.08162638572f, 0.05440524970f, 0.03539994489f, 0.02477268373f,
         0.01725797359f, 0.00000000000f}};

    if (tablesBuilt_) return;
    tablesBuilt_ = true;
    for (int m = 0; m < 3; ++m) {
      inputGain_[static_cast<size_t>(m)].build(
          std::vector<float>(std::begin(inputX), std::end(inputX)),
          std::vector<float>(std::begin(inputY[m]), std::end(inputY[m])));
      outputGain_[static_cast<size_t>(m)].build(
          std::vector<float>(std::begin(outputX), std::end(outputX)),
          std::vector<float>(std::begin(outputY[m]), std::end(outputY[m])));
      releaseTime_[static_cast<size_t>(m)].build(
          std::vector<float>(std::begin(releaseX), std::end(releaseX)),
          std::vector<float>(std::begin(releaseY[m]), std::end(releaseY[m])));
      gr_[static_cast<size_t>(m)].build(
          std::vector<float>(std::begin(grX[m]), std::end(grX[m])),
          std::vector<float>(std::begin(grY[m]), std::end(grY[m])));
    }
    designHpf(8.0f / sr_, 0.7f);
  }

  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr float kMinLevel = 0.0005623413251903491f; // -65 dB

  float sr_ = 48000.0f;
  int model_ = 0;
  float inputPos_ = 5.0f;
  float releasePos_ = 4.0f;
  float outputPos_ = 0.0f;
  bool superFuse_ = false;
  bool boxTone_ = true;
  bool dirty_ = true;
  bool tablesBuilt_ = false;
  float inputGainK_ = 1.0f;
  float outputGainK_ = 1.0f;
  float attackK_ = 0.99f;
  float releaseK_ = 0.99f;
  float lastGrDb_ = 0.0f;

  std::array<Spline, 3> inputGain_;
  std::array<Spline, 3> outputGain_;
  std::array<Spline, 3> releaseTime_;
  std::array<Spline, 3> gr_;
  std::array<float, 3> attackTimeSec_ = {0.147f, 0.168f, 0.050f};

  Filter2 hpf_, pk_, hs_, sc_;
  Channel ch_[2];
};

} // namespace hdsp::modules
