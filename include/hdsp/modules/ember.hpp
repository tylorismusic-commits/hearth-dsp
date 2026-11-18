// hearth-dsp — Ember finish module
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include "ember/ember_strip.hpp"
#include "ember/params.hpp"

namespace hdsp::modules {

struct EmberControls {
  float amount = 0.0f;  // 0..1
  float tone = 0.5f;    // 0 dark .. 1 bright
  float mix = 1.0f;     // dry/wet
};

class Ember {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    strip_.prepare(sr_);
    reset();
  }

  void setControls(const EmberControls& c) noexcept {
    hdsp::ember::Params p;
    p.amount = std::clamp(c.amount, 0.0f, 1.0f);
    p.enabled = true;
    strip_.setTargets(p);
    mix_ = std::clamp(c.mix, 0.0f, 1.0f);
    const float toneDb = (std::clamp(c.tone, 0.0f, 1.0f) - 0.5f) * 12.0f;
    lowL_.lowShelf(400.0f, -toneDb * 0.5f, 0.6f, sr_);
    lowR_.lowShelf(400.0f, -toneDb * 0.5f, 0.6f, sr_);
    highL_.highShelf(3500.0f, toneDb * 0.5f, 0.6f, sr_);
    highR_.highShelf(3500.0f, toneDb * 0.5f, 0.6f, sr_);
  }

  inline void process(float& l, float& r) noexcept {
    const float dryL = l, dryR = r;
    float grDb = 0.0f;
    float outDb = 0.0f;
    strip_.process(&l, &r, &l, &r, 1, grDb, outDb);
    l = highL_.process(lowL_.process(l));
    r = highR_.process(lowR_.process(r));
    l = dryL + (l - dryL) * mix_;
    r = dryR + (r - dryR) * mix_;
    lastGrDb_ = -grDb;
  }

  void reset() noexcept {
    strip_.reset();
    lowL_.reset();
    lowR_.reset();
    highL_.reset();
    highR_.reset();
    lastGrDb_ = 0.0f;
  }

  int latencySamples() const noexcept { return 0; }
  float lastGrDb() const noexcept { return lastGrDb_; }

private:
  hdsp::ember::EmberStrip strip_;
  Biquad lowL_, lowR_, highL_, highR_;
  float sr_ = 48000.0f;
  float mix_ = 1.0f;
  float lastGrDb_ = 0.0f;
};

} // namespace hdsp::modules
