// hearth-dsp shared module — macro.hpp
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "util.hpp"

namespace hdsp::ember {

// The whole point of Ember is that one control drives a complete, staged
// mastering chain.  Different parts of the chain fade in at different points
// in the knob travel: tone first, then glue, then harmonics and air.  Keeping
// the mapping in one place makes the character easy to tune and lets the UI
// draw the same "under the hood" picture.
struct MacroSettings {
  float lowFreq = 110.0f;
  float lowGainDb = 0.0f;
  float presenceFreq = 2600.0f;
  float presenceGainDb = 0.0f;
  float presenceQ = 0.55f;
  float airFreq = 10500.0f;
  float airGainDb = 0.0f;

  float compThresholdDb = -8.0f;
  float compRatio = 1.0f;
  float compAttackMs = 32.0f;
  float compReleaseMs = 450.0f;
  float compKneeDb = 10.0f;
  float compMakeupDb = 0.0f;
  float compMix = 0.0f;
  bool compOn = false;

  float satDrive = 1.0f;
  float satBias = 0.0f;
  float satMix = 0.0f;

  float outputGainDb = 0.0f;

  // Display weights for the front-panel chain meters.
  float toneAmount = 0.0f;
  float glueAmount = 0.0f;
  float harmonicAmount = 0.0f;
  float airAmount = 0.0f;
};

inline MacroSettings macroFor(float amount01) noexcept {
  const float a = clamp(amount01, 0.0f, 1.0f);

  // Each stage has its own entrance point in the knob travel.
  const float tone = smoothStep(0.0f, 0.90f, a);
  const float glue = smoothStep(0.06f, 1.0f, a);
  const float harm = smoothStep(0.18f, 1.0f, a);
  const float air = smoothStep(0.32f, 1.0f, a);

  MacroSettings m;

  m.lowFreq = 110.0f - 20.0f * tone;
  m.lowGainDb = 3.2f * tone;

  m.presenceFreq = 2600.0f + 1000.0f * tone;
  m.presenceGainDb = 2.8f * smoothStep(0.04f, 0.90f, a);
  m.presenceQ = 0.55f + 0.25f * tone;

  m.airFreq = 10500.0f + 3000.0f * air;
  m.airGainDb = 2.2f * air;

  m.compThresholdDb = -8.0f - 13.0f * glue;
  m.compRatio = 1.10f + 2.90f * glue;
  m.compAttackMs = 32.0f - 22.0f * glue;
  m.compReleaseMs = 450.0f - 240.0f * glue;
  m.compKneeDb = 10.0f - 6.0f * glue;
  m.compMakeupDb = 0.0f;
  m.compMix = 0.85f + 0.15f * glue;
  m.compOn = glue > 0.0001f;

  m.satDrive = 1.0f + 1.35f * harm;
  m.satBias = 0.040f * harm;
  m.satMix = 0.55f * harm;

  // A small finishing lift follows the glue amount, while the compressor's
  // auto makeup keeps the loudness from simply jumping.
  m.outputGainDb = 0.8f * glue;

  m.toneAmount = tone;
  m.glueAmount = glue;
  m.harmonicAmount = harm;
  m.airAmount = air;

  return m;
}

} // namespace hdsp::ember
