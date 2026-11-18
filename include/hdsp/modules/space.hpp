// hearth-dsp — Space reverb, delay and modulation wrappers
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The three Space Suite processors, exposed as compact 500-series modules.
// Each wrapper fixes one mode and maps normalised controls onto the underlying
// Space controls.
#pragma once

#include <algorithm>
#include <cmath>

#include "space/controls.hpp"
#include "space/delay.hpp"
#include "space/modulation.hpp"
#include "space/reverb.hpp"

namespace hdsp::modules {

struct SpaceReverbControls {
  int mode = 0;          // 0 Plate, 1 Room, 2 Hall, 3 Spring
  float size = 0.50f;
  float decay = 0.18f;   // 0..1 -> 0.3..12 s
  float predelay = 0.12f;// 0..1 -> 0..200 ms
  float tone = 0.50f;
  float width = 0.70f;
  float mix = 0.30f;
};

class SpaceReverb {
public:
  void prepare(float sampleRate) noexcept {
    reverb_.prepare(sampleRate);
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
  }
  void setControls(const SpaceReverbControls& c) noexcept {
    hdsp::space::Controls sc;
    sc.reverbOn = true;
    sc.reverbMode = std::clamp(c.mode, 0, 3);
    sc.reverbSize = 100.0f * std::clamp(c.size, 0.0f, 1.0f);
    sc.reverbDecay = 0.3f + 11.7f * std::clamp(c.decay, 0.0f, 1.0f);
    sc.reverbPreDelay = 200.0f * std::clamp(c.predelay, 0.0f, 1.0f);
    sc.reverbTone = 100.0f * std::clamp(c.tone, 0.0f, 1.0f);
    sc.reverbWidth = 100.0f * std::clamp(c.width, 0.0f, 1.0f);
    sc.reverbDuck = 30.0f;
    sc.reverbMix = 100.0f * std::clamp(c.mix, 0.0f, 1.0f);
    sc.reverbFreeze = false;
    reverb_.setControls(sc);
  }
  inline void process(float& l, float& r) noexcept { reverb_.process(l, r); }
  void reset() noexcept { reverb_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  float sr_ = 48000.0f;
  hdsp::space::Reverb reverb_;
};

struct SpaceDelayControls {
  int mode = 0;          // 0 Digital, 1 Tape, 2 Analog, 3 Ping-Pong
  float time = 0.19f;    // 0..1 -> 1..2000 ms
  float feedback = 0.35f;
  float tone = 0.50f;
  float mod = 0.15f;
  float width = 0.50f;
  float mix = 0.25f;
};

class SpaceDelay {
public:
  void prepare(float sampleRate) noexcept {
    delay_.prepare(sampleRate);
  }
  void setControls(const SpaceDelayControls& c) noexcept {
    hdsp::space::Controls sc;
    sc.delayOn = true;
    sc.delayMode = std::clamp(c.mode, 0, 3);
    sc.delaySync = false;
    sc.delayDiv = 3;
    sc.delayTimeMs = 1.0f + 1999.0f * std::clamp(c.time, 0.0f, 1.0f);
    sc.delayFeedback = 100.0f * std::clamp(c.feedback, 0.0f, 1.0f);
    sc.delayTone = 100.0f * std::clamp(c.tone, 0.0f, 1.0f);
    sc.delayMod = 100.0f * std::clamp(c.mod, 0.0f, 1.0f);
    sc.delayWidth = 100.0f * std::clamp(c.width, 0.0f, 1.0f);
    sc.delayDuck = 30.0f;
    sc.delayMix = 100.0f * std::clamp(c.mix, 0.0f, 1.0f);
    sc.bpm = 120.0f;
    delay_.setControls(sc, 120.0f);
  }
  inline void process(float& l, float& r) noexcept { delay_.process(l, r); }
  void reset() noexcept { delay_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  hdsp::space::Delay delay_;
};

struct SpaceModControls {
  int mode = 0;          // 0 Chorus, 1 Flanger, 2 Phaser, 3 Dimension
  float rate = 0.25f;    // 0..1 -> 0.05..10 Hz
  float depth = 0.50f;
  float feedback = 0.20f;
  float width = 0.70f;
  float mix = 0.40f;
};

class SpaceMod {
public:
  void prepare(float sampleRate) noexcept {
    mod_.prepare(sampleRate);
  }
  void setControls(const SpaceModControls& c) noexcept {
    hdsp::space::Controls sc;
    sc.modOn = true;
    sc.modMode = std::clamp(c.mode, 0, 3);
    sc.modRate = 0.05f * std::pow(200.0f, std::clamp(c.rate, 0.0f, 1.0f));
    sc.modDepth = 100.0f * std::clamp(c.depth, 0.0f, 1.0f);
    sc.modFeedback = 100.0f * std::clamp(c.feedback, 0.0f, 1.0f);
    sc.modWidth = 100.0f * std::clamp(c.width, 0.0f, 1.0f);
    sc.modMix = 100.0f * std::clamp(c.mix, 0.0f, 1.0f);
    mod_.setControls(sc);
  }
  inline void process(float& l, float& r) noexcept { mod_.process(l, r); }
  void reset() noexcept { mod_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  hdsp::space::Modulation mod_;
};

} // namespace hdsp::modules
