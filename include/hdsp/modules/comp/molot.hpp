// hearth-dsp shared module — molot.hpp
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include "biquad.hpp"
#include "common.hpp"
#include "controls.hpp"

namespace hdsp::modules::comp {

// Character compressor in the Molot family: a feed-forward peak detector with
// a diode-shaped attack/release, a polynomial soft-knee transfer function, an
// optional saturator and six stereo modes.
//
// Translated from the MIT-licensed JSFX reference.  The original approximates
// `pow()` with a 99-entry polynomial table for speed; this uses `pow()` itself,
// which is the function the table was approximating.
class MolotCompressor {
public:
  enum StereoMode { Mono = 0, DualMono = 1, Mid = 2, Side = 3, MidSide = 4,
                    Stereo = 5 };

  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    signalFreqK_ = std::min(21500.0 / sr_, 0.49);
    reset();
    setControls(controls_);
  }

  void setControls(const Controls& c) noexcept {
    controls_ = c;

    const double inputDb = -12.0 + 24.0 * clampf(c.input, 0.0f, 1.0f);
    const double threshold = std::pow(10.0, (-40.0 + 40.0 * clampf(c.threshold, 0.0f, 1.0f)) / 20.0);
    const double ratio = ratioFromStep(ratioStepFrom(c.ratio));
    const double knee = clampf(c.knee, 0.0f, 1.0f);
    const double attackS = (1.0 + 99.0 * clampf(c.attack, 0.0f, 1.0f)) / 1000.0;
    const double releaseS = (50.0 + 950.0 * clampf(c.release, 0.0f, 1.0f)) / 1000.0;
    // The reference maps Attack Mode 0 (Alpha) to the sharp/exp curve.
    const bool sharp = c.attackMode < 0.5f;
    const double makeup = std::pow(10.0, (-12.0 + 24.0 * clampf(c.makeup, 0.0f, 1.0f)) / 20.0);
    const double drive = -12.0 + 24.0 * clampf(c.satDrive, 0.0f, 1.0f);
    const double satThreshold = std::pow(10.0, (12.0 - drive) / 20.0);

    stereoMode_ = std::clamp(static_cast<int>(c.stereoMode + 0.5f), 0, 5);
    // c.distortion is P_DISTORTION/3, so any non-zero step means on.
    useSat_ = c.distortion > 0.1f;
    // The saturator is bypassed in S and M/S modes to avoid damaging the
    // stereo image, as in the original.
    const bool satAllowed =
        useSat_ && stereoMode_ != Side && stereoMode_ != MidSide;

    // Detector time constants are quantised by the reference: the Attack knob
    // only shapes the gain smoother, not the detector.
    double envAttack = 0.025;
    double envRelease = 0.050;
    double detectorFix = std::pow(10.0, 2.237 / 20.0);
    if (sharp) {
      envAttack = 0.00005;
      envRelease = 0.001;
      detectorFix = std::pow(10.0, 0.878 / 20.0);
    } else if (attackS <= 0.0095) {
      envAttack = 0.001;
      envRelease = 0.050;
      detectorFix = std::pow(10.0, 0.547 / 20.0);
    } else if (attackS <= 0.0245) {
      envAttack = 0.010;
      envRelease = 0.050;
      detectorFix = std::pow(10.0, 1.536 / 20.0);
    }

    const double attackK = secToFreqK(attackS);
    const double releaseK = secToFreqK(releaseS);

    const double scFreq = clampf(c.sidechainHpf, 0.0f, 1.0f);
    const bool usePhon = scFreq > 0.02f;
    const double scHz = 40.0 + 220.0 * scFreq;

    for (int ch = 0; ch < 2; ++ch) {
      Channel& comp = channels_[ch];
      comp.inGain = std::pow(10.0, inputDb / 20.0);
      comp.gain = makeup;
      comp.detectorFix = detectorFix;
      comp.useSat = satAllowed;
      comp.useScFlt = usePhon;
      comp.env.set(secToFreqK(envAttack), secToFreqK(envRelease), signalFreqK_,
                   sharp, sr_);
      comp.envAr.set(attackK, releaseK, signalFreqK_, sharp, sr_);
      comp.clip.set(threshold, knee, ratio);
      comp.sat.threshold = satThreshold;
      if (usePhon) {
        comp.scPhon.set(scHz, sr_);
      } else {
        comp.scFixed.lowShelf(296.0f, -3.02f, 0.7f, sr_);
      }
    }
  }

  // Returns gain reduction in dB (>= 0).
  float process(float& left, float& right,
                const SidechainFrame& sc = {}) noexcept {
    const double x1 = left;
    const double x2 = right;
    double sc1 = sc.active ? sc.left : x1;
    double sc2 = sc.active ? sc.right : x2;

    double y1;
    double y2;
    double gr = 1.0;

    switch (stereoMode_) {
      case Mono:
        y1 = channels_[0].process(x1, sc1);
        y2 = y1;
        gr = channels_[0].gr;
        break;
      case DualMono:
        y1 = channels_[0].process(x1, sc1);
        y2 = channels_[1].process(x2, sc2);
        gr = std::min(channels_[0].gr, channels_[1].gr);
        break;
      case Mid: {
        const double m = (x1 + x2) * 0.5;
        const double s = (x1 - x2) * 0.5;
        const double scm = (sc1 + sc2) * 0.5;
        const double ym = channels_[0].process(m, scm);
        y1 = ym + s;
        y2 = ym - s;
        gr = channels_[0].gr;
        break;
      }
      case Side: {
        const double m = (x1 + x2) * 0.5;
        const double s = (x1 - x2) * 0.5;
        const double scs = (sc1 - sc2) * 0.5;
        const double ys = channels_[0].process(s, scs);
        y1 = m + ys;
        y2 = m - ys;
        gr = channels_[0].gr;
        break;
      }
      case Stereo: {
        const double e1 = channels_[0].preprocess(sc1);
        const double e2 = channels_[1].preprocess(sc2);
        const double env = (e1 + e2) * 0.5;
        y1 = channels_[0].postprocess(x1, env);
        y2 = channels_[1].postprocess(x2, env);
        gr = std::min(channels_[0].gr, channels_[1].gr);
        break;
      }
      case MidSide:
      default: {
        const double scm = (sc1 + sc2) * 0.5;
        const double scs = (sc1 - sc2) * 0.5;
        const double e1 = channels_[0].preprocess(scm);
        const double e2 = channels_[1].preprocess(scs);
        const double env = std::max(e1, e2);
        const double m = (x1 + x2) * 0.5;
        const double s = (x1 - x2) * 0.5;
        const double ym = channels_[0].postprocess(m, env);
        const double ys = channels_[1].postprocess(s, env);
        // Saturation is bypassed in M/S, as the comment on satAllowed says.
        // Applying it here unconditionally coloured the signal even with the
        // SAT switch off.
        y1 = ym + ys;
        y2 = ym - ys;
        gr = std::min(channels_[0].gr, channels_[1].gr);
        break;
      }
    }

    left = static_cast<float>(y1);
    right = static_cast<float>(y2);
    return static_cast<float>(-20.0 * std::log10(std::max(gr, 1e-6)));
  }

  void reset() noexcept {
    for (auto& c : channels_) {
      c.reset();
    }
  }

private:
  static constexpr double kTwoPi = 6.283185307179586476925286766559;

  double secToFreqK(double seconds) const noexcept {
    const double f = seconds > 0.0 ? 1.0 / seconds : sr_ / kTwoPi;
    return f / sr_;
  }

  // Diode-shaped asymmetric attack/release with a low-pass, from the reference.
  struct ArFilter {
    Biquad diodeLpf;
    double fa = 0.0;
    double fr = 0.0;
    double s = 0.0;
    double ax[8] = {};

    void set(double attackK, double releaseK, double signalK, bool sharp,
             double sr) noexcept {
      const double xx = 0.025;
      ax[0] = 6435.0 / std::pow(xx, 8);
      ax[1] = -40040.0 / std::pow(xx, 9);
      ax[2] = 108108.0 / std::pow(xx, 10);
      ax[3] = -163800.0 / std::pow(xx, 11);
      ax[4] = 150150.0 / std::pow(xx, 12);
      ax[5] = -83160.0 / std::pow(xx, 13);
      ax[6] = 25740.0 / std::pow(xx, 14);
      ax[7] = -3432.0 / std::pow(xx, 15);

      const double cutoffK =
          sharp ? signalK : std::max(attackK, releaseK);
      diodeLpf.lowPass(static_cast<float>(cutoffK * sr), 0.7071f,
                       static_cast<float>(sr));
      fa = 1.0 / (1.0 / (attackK * kTwoPi) + 1.0);
      fr = 1.0 / (1.0 / (releaseK * kTwoPi) + 1.0);
    }

    double diode(double x) const noexcept {
      if (x < 0.0) {
        return 0.0;
      }
      if (x > 0.025) {
        return x;
      }
      const double x2 = x * x;
      const double x4 = x2 * x2;
      const double x8 = x4 * x4;
      const double x12 = x8 * x4;
      return x8 * x * ax[0] + x8 * x2 * ax[1] + x8 * x2 * x * ax[2] +
             x12 * ax[3] + x12 * x * ax[4] + x12 * x2 * ax[5] +
             x12 * x2 * x * ax[6] + x12 * x4 * ax[7];
    }

    double process(double x) noexcept {
      s += fa * diode(x - s) + fr * (x - s);
      return diodeLpf.process(static_cast<float>(s));
    }

    void reset() noexcept {
      diodeLpf.reset();
      s = 0.0;
    }
  };

  struct ArEnvelope {
    Biquad absLpf;
    ArFilter filter;

    void set(double attackK, double releaseK, double signalK, bool sharp,
             double sr) noexcept {
      absLpf.lowPass(static_cast<float>(signalK * sr), 0.7071f,
                     static_cast<float>(sr));
      filter.set(attackK, releaseK, signalK, sharp, sr);
    }

    double processAC(double x) noexcept {
      return filter.process(absLpf.process(static_cast<float>(std::fabs(x))));
    }

    double processDC(double x) noexcept { return filter.process(x); }

    void reset() noexcept {
      absLpf.reset();
      filter.reset();
    }
  };

  // Transfer function: soft knee below the threshold, ratio above it.
  struct EnvelopeClip {
    double thresh = 1.0;
    double thresh2 = 1.0;
    double threshRatio = 1.0;
    double ratioExp = 1.0;
    double a3 = 0.0;
    double a4 = 0.0;
    bool limiter = false;

    void set(double threshold, double knee, double ratio) noexcept {
      thresh = std::max(threshold, 1e-6);
      if (ratio <= 10.9) {
        ratio = std::floor(ratio * 10.0) / 10.0;
        ratio = std::max(ratio, 1.1);
        ratioExp = 1.0 / ratio;
        threshRatio = std::pow(1.0 / thresh, ratioExp) * thresh;
        limiter = false;
      } else {
        limiter = true;
        ratioExp = 1.0;
        threshRatio = thresh;
      }

      const double delta = (1.0 - knee * 0.95) * thresh;
      thresh2 = thresh + delta;
      const double x2 = std::max(thresh2, 1e-6);
      const double y2 = h2(x2);
      const double dy2 = dh2(x2);
      a3 = -(3.0 * x2 - 4.0 * y2 + x2 * dy2) / (x2 * x2 * x2);
      a4 = (2.0 * x2 - 3.0 * y2 + x2 * dy2) / (x2 * x2 * x2 * x2);
    }

    double h2(double x) const noexcept {
      if (limiter) {
        return thresh;
      }
      x = std::min(x, 10.0);
      return std::pow(std::max(x, 0.0), ratioExp) * threshRatio;
    }

    double dh2(double x) const noexcept {
      if (limiter || x <= 0.0) {
        return 0.0;
      }
      return h2(x) * ratioExp / x;
    }

    double h1(double x) const noexcept {
      return x + a3 * x * x * x + a4 * x * x * x * x;
    }

    double h(double x) const noexcept {
      return x < thresh2 ? h1(x) : h2(x);
    }

    double process(double x) const noexcept {
      return x < 1.0e-6 ? 1.0 : h(x) / x;
    }
  };

  struct Saturator {
    double threshold = 1.0;

    static double curve(double x) noexcept {
      x = std::min(std::max(x, -1.927384823613229), 1.925547073273503);
      const double x2 = x * x;
      const double x4 = x2 * x2;
      return 1.043653138515045 * x + 0.051012188210767 * x2 -
             0.048182108389193 * x2 * x - 0.076851680685586 * x4 -
             0.076851680685586 * x4 * x + 0.019456557521838 * x4 * x2 +
             0.011322488095808 * x4 * x2 * x -
             0.001614452827801 * x4 * x4;
    }

    double process(double x) const noexcept {
      return curve(x / threshold) * threshold;
    }
  };

  // Two cascaded 35.15 Hz high-passes whose Q follows the requested frequency.
  struct PhonHighPass {
    Biquad a;
    Biquad b;

    void set(double freqHz, double sr) noexcept {
      const double x = std::max(freqHz, 23.756);
      const double ff = x / 35.15;
      const double ff2 = ff * ff;
      const double denom = (std::sqrt(2.0) - 1.0) * ff2 + 2.0 - 1.0 / ff2;
      const double q = 1.0 / std::sqrt(std::max(denom, 1e-6));
      a.highPass(35.15f, static_cast<float>(q), static_cast<float>(sr));
      b.highPass(35.15f, static_cast<float>(q), static_cast<float>(sr));
    }

    double process(double x) noexcept { return b.process(a.process(x)); }

    void reset() noexcept {
      a.reset();
      b.reset();
    }
  };

  struct Channel {
    ArEnvelope env;
    ArEnvelope envAr;
    PhonHighPass scPhon;
    Biquad scFixed;
    EnvelopeClip clip;
    Saturator sat;
    bool useScFlt = false;
    bool useSat = false;
    double gain = 1.0;
    double inGain = 1.0;
    double detectorFix = 1.0;
    double gr = 1.0;

    double preprocess(double x) noexcept {
      const double sc = useScFlt ? scPhon.process(x) : scFixed.process(x);
      return env.processAC(sc * inGain) * detectorFix;
    }

    double postprocess(double xDry, double envValue) noexcept {
      const double envGr = clip.process(envValue);
      const double smoothed = 2.0 - envAr.processDC(2.0 - envGr);
      gr = smoothed;
      double wet = xDry * smoothed * inGain;
      if (useSat) {
        wet = sat.process(wet);
      }
      // Dry/wet is applied once, by the engine: doing it here as well made the
      // mix control quadratic.
      return wet * gain;
    }

    double process(double x, double sc) noexcept {
      return postprocess(x, preprocess(sc));
    }

    void reset() noexcept {
      env.reset();
      envAr.reset();
      scPhon.reset();
      scFixed.reset();
      gr = 1.0;
    }
  };

  double sr_ = 48000.0;
  double signalFreqK_ = 0.448;
  Controls controls_;
  int stereoMode_ = Stereo;
  bool useSat_ = false;
  Channel channels_[2];
};

} // namespace hdsp::modules::comp
