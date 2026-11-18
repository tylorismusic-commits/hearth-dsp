// hearth-dsp — Trowel Tame module
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>

#include "tame/params.hpp"
#include "tame/tame_dynamic.hpp"

namespace hdsp::modules {

struct TameControls {
  float amount = 0.00f;
  float focus = 0.50f;
  float selectivity = 0.50f;
  float sensitivity = 0.50f;
  float mix = 1.00f;
};

class Tame {
public:
  void prepare(float sampleRate) noexcept {
    tame_.prepare(sampleRate);
  }

  void setControls(const TameControls& c) noexcept {
    hdsp::tame::TameParams p;
    p.amount = std::clamp(c.amount, 0.0f, 1.0f);
    p.focus = std::clamp(c.focus, 0.0f, 1.0f);
    p.selectivity = std::clamp(c.selectivity, 0.0f, 1.0f);
    p.sensitivity = std::clamp(c.sensitivity, 0.0f, 1.0f);
    p.mix = std::clamp(c.mix, 0.0f, 1.0f);
    p.attackMs = 10.0f;
    p.releaseMs = 250.0f;
    tame_.setTargets(p);
  }

  inline void process(float& l, float& r) noexcept {
    tame_.process(&l, &r, 1);
  }

  void reset() noexcept { tame_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  hdsp::tame::TameDynamic tame_;
};

} // namespace hdsp::modules
