// hearth-dsp shared module — Tame parameters
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

namespace hdsp::tame {

struct TameParams {
  float amount = 0.0f;
  float focus = 0.5f;
  float selectivity = 0.5f;
  float sensitivity = 0.5f;
  float attackMs = 10.0f;
  float releaseMs = 250.0f;
  float mix = 1.0f;
  bool listen = false;
  bool msMode = false;
};

} // namespace hdsp::tame
