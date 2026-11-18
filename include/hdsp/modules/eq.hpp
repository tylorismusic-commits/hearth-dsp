// hearth-dsp — EQ module family
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A four-control EQ family used by Suite Rack and future plugins:
//   0 — 4K console-style four-band EQ
//   1 — Hearth inductor console voicing
//   2 — Tilt
//   3 — Air
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/biquad.hpp>

namespace hdsp::modules {

inline float eqHpfHz(float v) noexcept {
  v = std::clamp(v, 0.0f, 1.0f);
  return 20.0f * std::pow(25.0f, v);
}
inline float eqLpfHz(float v) noexcept {
  v = std::clamp(v, 0.0f, 1.0f);
  return 500.0f * std::pow(40.0f, v);
}
inline float airFreqHz(float v) noexcept {
  v = std::clamp(v, 0.0f, 1.0f);
  return 2000.0f * std::pow(8.0f, v);
}

struct EqControls {
  int voicing = 0;  // 0 4K, 1 Hearth, 2 Tilt, 3 Air
  float low = 0.60f;
  float lowMid = 0.45f;
  float highMid = 0.55f;
  float high = 0.57f;
  float hpf = 0.10f;
  float lpf = 1.00f;
};

class Eq {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    reset();
  }

  void setControls(const EqControls& c) noexcept {
    const int voicing = std::clamp(c.voicing, 0, 3);
    if (voicing != lastVoicing_) {
      resetFilters();
      lastVoicing_ = voicing;
    }
    if (voicing == 0) {
      setFourBand(c, 80.0f, 250.0f, 1200.0f, 6000.0f, 0.9f, 0.70f);
    } else if (voicing == 1) {
      setFourBand(c, 100.0f, 350.0f, 1800.0f, 8000.0f, 0.75f, 0.60f);
    } else if (voicing == 2) {
      const float tilt = (c.low - 0.5f) * 24.0f;
      lowL_.lowShelf(400.0f, -tilt, 0.5f, sr_);
      lowR_.lowShelf(400.0f, -tilt, 0.5f, sr_);
      highL_.highShelf(2500.0f, tilt, 0.5f, sr_);
      highR_.highShelf(2500.0f, tilt, 0.5f, sr_);
      setFilters(c.lowMid, c.highMid);
    } else {
      const float air = (c.low - 0.5f) * 24.0f;
      const float freq = airFreqHz(c.lowMid);
      highL_.highShelf(freq, air, 0.5f, sr_);
      highR_.highShelf(freq, air, 0.5f, sr_);
      setFilters(c.highMid, 1.0f);
    }
  }

  inline void process(float& l, float& r) noexcept {
    l = lpL_.process(hpL_.process(highL_.process(
        highMidL_.process(lowMidL_.process(lowL_.process(l))))));
    r = lpR_.process(hpR_.process(highR_.process(
        highMidR_.process(lowMidR_.process(lowR_.process(r))))));
  }

  void reset() noexcept {
    resetFilters();
    lastVoicing_ = -1;
  }

private:
  void resetFilters() noexcept {
    lowL_ = Biquad{};
    lowR_ = Biquad{};
    lowMidL_ = Biquad{};
    lowMidR_ = Biquad{};
    highMidL_ = Biquad{};
    highMidR_ = Biquad{};
    highL_ = Biquad{};
    highR_ = Biquad{};
    hpL_ = Biquad{};
    hpR_ = Biquad{};
    lpL_ = Biquad{};
    lpR_ = Biquad{};
  }

  void setFourBand(const EqControls& c, float lowHz, float lowMidHz,
                   float highMidHz, float highHz, float midQ,
                   float shelfQ) noexcept {
    const float low = (c.low - 0.5f) * 24.0f;
    const float lowMid = (c.lowMid - 0.5f) * 24.0f;
    const float highMid = (c.highMid - 0.5f) * 24.0f;
    const float high = (c.high - 0.5f) * 24.0f;
    lowL_.lowShelf(lowHz, low, shelfQ, sr_);
    lowR_.lowShelf(lowHz, low, shelfQ, sr_);
    lowMidL_.peak(lowMidHz, lowMid, midQ, sr_);
    lowMidR_.peak(lowMidHz, lowMid, midQ, sr_);
    highMidL_.peak(highMidHz, highMid, midQ, sr_);
    highMidR_.peak(highMidHz, highMid, midQ, sr_);
    highL_.highShelf(highHz, high, shelfQ, sr_);
    highR_.highShelf(highHz, high, shelfQ, sr_);
    setFilters(c.hpf, c.lpf);
  }

  void setFilters(float hpf, float lpf) noexcept {
    if (hpf > 0.001f) {
      hpL_.highPass(eqHpfHz(hpf), 0.707f, sr_);
      hpR_.highPass(eqHpfHz(hpf), 0.707f, sr_);
    } else {
      hpL_ = Biquad{};
      hpR_ = Biquad{};
    }
    if (lpf < 0.999f) {
      lpL_.lowPass(eqLpfHz(lpf), 0.707f, sr_);
      lpR_.lowPass(eqLpfHz(lpf), 0.707f, sr_);
    } else {
      lpL_ = Biquad{};
      lpR_ = Biquad{};
    }
  }

  float sr_ = 48000.0f;
  int lastVoicing_ = -1;
  Biquad lowL_, lowR_, lowMidL_, lowMidR_;
  Biquad highMidL_, highMidR_, highL_, highR_;
  Biquad hpL_, hpR_, lpL_, lpR_;
};

} // namespace hdsp::modules
