// hearth-dsp shared module — tape.hpp
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "filters.hpp"
#include "preamp.hpp"
#include "smoother.hpp"
#include "util.hpp"

namespace hdsp::reel {

// A reel-to-reel / cassette tape stage with record pre-emphasis, head bump,
// odd-order tape saturation, playback bandwidth limiting and modulated wow /
// flutter.  Dry and wet are delayed by the same transport delay so the mix
// does not comb, and the reported latency is constant whether the tape is on
// or off.
class Tape {
public:
  static constexpr float kBaseDelayMs = 4.0f;

  void prepare(float sampleRate) noexcept {
    sampleRate_ = sampleRate;
    size_ = std::max(1024, static_cast<int>(sampleRate * 0.08f)) + 4;
    dryL_.assign(static_cast<size_t>(size_), 0.0f);
    dryR_.assign(static_cast<size_t>(size_), 0.0f);
    wetL_.assign(static_cast<size_t>(size_), 0.0f);
    wetR_.assign(static_cast<size_t>(size_), 0.0f);
    baseSamples_ = std::max(1, static_cast<int>(std::lround(
                                    kBaseDelayMs * 0.001f * sampleRate)));
    write_ = 0;

    drive_.setTime(20.0f, sampleRate);
    wow_.setTime(20.0f, sampleRate);
    mix_.setTime(20.0f, sampleRate);
    active_.setTime(10.0f, sampleRate);
    cassette_.setTime(30.0f, sampleRate);

    drive_.reset(0.2f);
    wow_.reset(0.1f);
    mix_.reset(0.25f);
    active_.reset(1.0f);
    cassette_.reset(0.0f);

    wowPhase_ = 0.0f;
    flutterPhase_ = 0.0f;
    update();
    reset();
  }

  void setTargets(float drivePct, float wowPct, float mix01, bool enabled,
                  int type) noexcept {
    drive_.setTarget(clamp(drivePct * 0.01f, 0.0f, 1.0f));
    wow_.setTarget(clamp(wowPct * 0.01f, 0.0f, 1.0f));
    mix_.setTarget(clamp(mix01, 0.0f, 1.0f));
    active_.setTarget(enabled ? 1.0f : 0.0f);
    cassette_.setTarget(type == 1 ? 1.0f : 0.0f);
  }

  void tick() noexcept {
    drive_.next();
    wow_.next();
    mix_.next();
    active_.next();
    cassette_.next();
  }

  void update() noexcept {
    const float c = cassette_.current();
    const float preHz = 3000.0f - 900.0f * c;
    const float headHz = 60.0f + 35.0f * c;
    const float playHz = 19000.0f - 10000.0f * c;
    const float rumbleHz = 20.0f + 20.0f * c;
    for (auto& f : preHp_) f.set(preHz, sampleRate_);
    for (auto& f : headLp_) f.set(headHz, sampleRate_);
    for (auto& f : playLp_) f.set(std::min(playHz, sampleRate_ * 0.49f),
                                 sampleRate_);
    for (auto& f : rumbleHp_) f.set(rumbleHz, sampleRate_);
    for (auto& f : dc_) f.set(20.0f, sampleRate_);

    emphasis_ = 0.35f + 0.35f * c;
    headBump_ = 0.05f + 0.10f * c;
    driveGain_ = dbToGain(12.0f * drive_.current());
    driveAmount_ = drive_.current();
    wowDepthMs_ = (0.15f + 0.85f * c) * wow_.current();
    flutterDepthMs_ = (0.02f + 0.10f * c) * wow_.current();
  }

  inline void process(float& left, float& right) noexcept {
    const float inL = left;
    const float inR = right;
    const float wl = shape(0, inL);
    const float wr = shape(1, inR);

    dryL_[write_] = inL;
    dryR_[write_] = inR;
    wetL_[write_] = wl;
    wetR_[write_] = wr;

    const float c = cassette_.current();
    wowPhase_ += 2.0f * kPi * (0.6f + 0.3f * c) / sampleRate_;
    flutterPhase_ += 2.0f * kPi * (6.5f + 1.0f * c) / sampleRate_;
    if (wowPhase_ > 2.0f * kPi) wowPhase_ -= 2.0f * kPi;
    if (flutterPhase_ > 2.0f * kPi) flutterPhase_ -= 2.0f * kPi;

    const float modMs =
        wowDepthMs_ * std::sin(wowPhase_) +
        flutterDepthMs_ * std::sin(flutterPhase_);
    float delay = static_cast<float>(baseSamples_) +
                  modMs * 0.001f * sampleRate_ * active_.current();
    delay = clamp(delay, 1.0f, static_cast<float>(size_ - 2));

    float readPos = static_cast<float>(write_) - delay;
    while (readPos < 0.0f) {
      readPos += static_cast<float>(size_);
    }
    const int i0 = static_cast<int>(readPos);
    const float frac = readPos - static_cast<float>(i0);
    const int i1 = (i0 + 1) % size_;

    // The dry side is read at the fixed transport delay so Tape Mix at 0 is
    // clean; only the wet side follows the wow/flutter modulation.
    int dryIndex = write_ - baseSamples_;
    if (dryIndex < 0) {
      dryIndex += size_;
    }
    const float dryReadL = dryL_[dryIndex];
    const float dryReadR = dryR_[dryIndex];
    const float wetReadL = wetL_[i0] + frac * (wetL_[i1] - wetL_[i0]);
    const float wetReadR = wetR_[i0] + frac * (wetR_[i1] - wetR_[i0]);

    const float amount = mix_.current() * active_.current();
    left = dryReadL + amount * (wetReadL - dryReadL);
    right = dryReadR + amount * (wetReadR - dryReadR);

    write_ = (write_ + 1) % size_;
  }

  int latencySamples() const noexcept { return baseSamples_; }

  void reset() noexcept {
    std::fill(dryL_.begin(), dryL_.end(), 0.0f);
    std::fill(dryR_.begin(), dryR_.end(), 0.0f);
    std::fill(wetL_.begin(), wetL_.end(), 0.0f);
    std::fill(wetR_.begin(), wetR_.end(), 0.0f);
    write_ = 0;
    for (auto& s : sat_) s.reset();
    for (auto& f : preHp_) f.reset();
    for (auto& f : headLp_) f.reset();
    for (auto& f : playLp_) f.reset();
    for (auto& f : rumbleHp_) f.reset();
    for (auto& f : dc_) f.reset();
    drive_.snap();
    wow_.snap();
    mix_.snap();
    active_.snap();
    cassette_.snap();
    update();
  }

private:
  inline float shape(int channel, float x) noexcept {
    // record-side high-frequency pre-emphasis
    x = x + emphasis_ * preHp_[channel].hp(x);
    // low-frequency head bump / bass alignment
    x = x + headBump_ * headLp_[channel].lp(x);
    // odd-order magnetic saturation
    const float d = driveGain_;
    const float saturated =
        d > 1.0e-6f ? sat_[channel].process(x * d, 0.0f) / d : x;
    x = x + driveAmount_ * (saturated - x);
    // playback head + tape bandwidth
    x = playLp_[channel].lp(x);
    x = rumbleHp_[channel].hp(x);
    x = dc_[channel].hp(x);
    return x;
  }

  float sampleRate_ = 48000.0f;
  int size_ = 0;
  int write_ = 0;
  int baseSamples_ = 1;
  std::vector<float> dryL_;
  std::vector<float> dryR_;
  std::vector<float> wetL_;
  std::vector<float> wetR_;

  Smoothed drive_;
  Smoothed wow_;
  Smoothed mix_;
  Smoothed active_;
  Smoothed cassette_;

  OnePole preHp_[2];
  OnePole headLp_[2];
  OnePole playLp_[2];
  OnePole rumbleHp_[2];
  OnePole dc_[2];
  AdaaTanh sat_[2];

  float emphasis_ = 0.35f;
  float headBump_ = 0.05f;
  float driveGain_ = 1.0f;
  float driveAmount_ = 0.2f;
  float wowDepthMs_ = 0.0f;
  float flutterDepthMs_ = 0.0f;
  float wowPhase_ = 0.0f;
  float flutterPhase_ = 0.0f;
};

} // namespace hdsp::reel
