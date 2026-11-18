// hearth-dsp shared module — biquad.hpp
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Compatibility layer over hearth-dsp.
#pragma once

#include <hdsp/biquad.hpp>

namespace hdsp::modules::comp {
using ::hdsp::Biquad;
using ::hdsp::settleK;
} // namespace hdsp::modules::comp
