// hearth-dsp shared module — util.hpp
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace hdsp::ember {

constexpr float kPi = 3.14159265358979323846f;

inline float clamp(float x, float lo, float hi) noexcept {
  return std::min(std::max(x, lo), hi);
}

inline float dbToGain(float db) noexcept {
  return std::pow(10.0f, db * 0.05f);
}

inline float gainToDb(float gain) noexcept {
  return 20.0f * std::log10(std::max(gain, 1.0e-12f));
}

// Numerically stable log(cosh(x)).  Used as the antiderivative of tanh() for
// first-order antiderivative anti-aliasing (ADAA).
inline float logCosh(float x) noexcept {
  const float ax = std::fabs(x);
  return ax + std::log1p(std::exp(-2.0f * ax)) - 0.6931471805599453f;
}

inline bool isFinite(float x) noexcept {
  return std::isfinite(x);
}

// Smooth 0..1 fade used to bring each part of the one-knob chain in at a
// different point in the travel.
inline float smoothStep(float edge0, float edge1, float x) noexcept {
  if (edge1 <= edge0) {
    return x >= edge1 ? 1.0f : 0.0f;
  }
  const float t = clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

} // namespace hdsp::ember
