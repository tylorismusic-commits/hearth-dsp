// hearth-dsp — Reel tape module
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>

#include "reel/tape.hpp"

namespace hdsp::modules {

struct ReelControls {
  int type = 0;        // 0 reel, 1 cassette
  float drive = 0.20f;
  float wow = 0.10f;
  float mix = 0.25f;
};

class Reel {
public:
  void prepare(float sampleRate) noexcept {
    tape_.prepare(sampleRate);
  }

  void setControls(const ReelControls& c) noexcept {
    tape_.setTargets(100.0f * std::clamp(c.drive, 0.0f, 1.0f),
                     100.0f * std::clamp(c.wow, 0.0f, 1.0f),
                     std::clamp(c.mix, 0.0f, 1.0f), true,
                     std::clamp(c.type, 0, 1));
  }

  inline void process(float& l, float& r) noexcept {
    tape_.tick();
    if ((counter_++ & 15u) == 0u) {
      tape_.update();
    }
    tape_.process(l, r);
  }

  void reset() noexcept {
    tape_.reset();
    counter_ = 0;
  }

  int latencySamples() const noexcept { return tape_.latencySamples(); }

private:
  hdsp::reel::Tape tape_;
  unsigned counter_ = 0;
};

} // namespace hdsp::modules
