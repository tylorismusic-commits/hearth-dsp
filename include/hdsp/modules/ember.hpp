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
};

class Ember {
public:
  void prepare(float sampleRate) noexcept {
    strip_.prepare(sampleRate);
  }

  void setControls(const EmberControls& c) noexcept {
    hdsp::ember::Params p;
    p.amount = std::clamp(c.amount, 0.0f, 1.0f);
    p.enabled = true;
    strip_.setTargets(p);
  }

  inline void process(float& l, float& r) noexcept {
    float grDb = 0.0f;
    float outDb = 0.0f;
    strip_.process(&l, &r, &l, &r, 1, grDb, outDb);
    lastGrDb_ = -grDb;
  }

  void reset() noexcept {
    strip_.reset();
    lastGrDb_ = 0.0f;
  }

  int latencySamples() const noexcept { return 0; }
  float lastGrDb() const noexcept { return lastGrDb_; }

private:
  hdsp::ember::EmberStrip strip_;
  float lastGrDb_ = 0.0f;
};

} // namespace hdsp::modules
