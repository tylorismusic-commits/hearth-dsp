// hearth-dsp shared module — params.hpp
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

namespace hdsp::ember {

// Control values in their natural DSP units.  The LV2 wrapper converts from
// the port units (percent) before filling this in.
struct Params {
  float amount = 0.0f; // 0..1
  bool enabled = true;
};

} // namespace hdsp::ember
