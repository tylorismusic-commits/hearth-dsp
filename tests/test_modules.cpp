// hearth-dsp — shared processor module tests
// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include <hdsp/modules/modules.hpp>

namespace {
int gFailures = 0;
void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++gFailures;
}
constexpr float kSr = 48000.0f;
constexpr int kN = 24000;  // 0.5 s

std::vector<float> tone(float hz, float amp = 0.25f) {
  std::vector<float> x(kN);
  for (int i = 0; i < kN; ++i)
    x[i] = amp * std::sin(2.0f * 3.14159265f * hz * i / kSr);
  return x;
}

double rms(const std::vector<float>& x, int a, int b) {
  double s = 0.0;
  for (int i = a; i < b; ++i) s += double(x[i]) * x[i];
  return std::sqrt(s / std::max(1, b - a));
}

bool finite(const std::vector<float>& x) {
  for (float v : x)
    if (!std::isfinite(v)) return false;
  return true;
}
} // namespace

int main() {
  using namespace hdsp::modules;

  // EQ high-pass
  {
    Eq eq;
    eq.prepare(kSr);
    EqControls c;
    c.voicing = 0;
    c.hpf = 1.0f;
    eq.setControls(c);
    auto dry = tone(100.0f);
    auto wet = dry;
    for (int i = 0; i < kN; ++i) eq.process(wet[i], wet[i]);
    check(finite(wet), "EQ stays finite");
    check(rms(wet, 2000, kN) < rms(dry, 2000, kN) * 0.5,
          "EQ high-pass removes low tone");
  }

  // EQ voicings
  for (int v = 0; v < 4; ++v) {
    Eq eq;
    eq.prepare(kSr);
    EqControls c;
    c.voicing = v;
    eq.setControls(c);
    auto x = tone(1000.0f);
    for (int i = 0; i < kN; ++i) eq.process(x[i], x[i]);
    check(finite(x), "EQ voicing stays finite");
  }

  // Compressor gain reduction
  {
    Comp comp;
    comp.prepare(kSr);
    CompControls c;
    c.model = 0;
    c.threshold = 0.75f;
    c.ratio = 0.75f;
    c.makeup = 0.4f;
    c.mix = 1.0f;
    comp.setControls(c);
    auto x = tone(1000.0f);
    float gr = 0.0f;
    for (int i = 0; i < kN; ++i) gr = std::max(gr, comp.process(x[i], x[i]));
    check(finite(x), "compressor stays finite");
    check(gr > 0.5f, "compressor reports gain reduction");
  }

  // Colour models
  for (int model = 0; model < 5; ++model) {
    Colour colour;
    colour.prepare(kSr);
    ColourControls c;
    c.model = model;
    colour.setControls(c);
    auto x = tone(1000.0f);
    for (int i = 0; i < kN; ++i) colour.process(x[i], x[i]);
    check(finite(x), "colour model stays finite");
  }

  // Gate closes on a low-level signal
  {
    Gate gate;
    gate.prepare(kSr);
    GateControls c;
    c.threshold = 0.6f;
    c.range = 0.8f;
    c.attack = 0.0f;
    c.release = 0.2f;
    c.hold = 0.0f;
    gate.setControls(c);
    auto dry = tone(1000.0f, 0.01f);
    auto wet = dry;
    for (int i = 0; i < kN; ++i) gate.process(wet[i], wet[i]);
    check(finite(wet), "gate stays finite");
    check(rms(wet, kN / 2, kN) < rms(dry, kN / 2, kN) * 0.5,
          "gate attenuates a low-level signal");
  }

  // Bleed Out
  {
    BleedOut bleed;
    bleed.prepare(kSr);
    BleedOutControls c;
    c.profile = 1;
    c.threshold = 0.5f;
    c.deBleed = 0.8f;
    c.ghost = 0.5f;
    bleed.setControls(c);
    auto x = tone(1000.0f);
    for (int i = 0; i < kN; ++i) bleed.process(x[i], x[i]);
    check(finite(x), "Bleed Out stays finite");
    check(bleed.latencySamples() > 0, "Bleed Out reports lookahead latency");
  }

  // Transient and clip
  {
    Transient transient;
    transient.prepare(kSr);
    TransientControls c;
    transient.setControls(c);
    auto x = tone(1000.0f);
    for (int i = 0; i < kN; ++i) transient.process(x[i], x[i]);
    check(finite(x), "transient stays finite");
  }
  {
    Clip clip;
    clip.prepare(kSr);
    ClipControls c;
    c.drive = 1.0f;
    c.ceiling = 0.2f;
    c.mode = 0;
    c.mix = 1.0f;
    clip.setControls(c);
    auto x = tone(1000.0f, 1.0f);
    for (int i = 0; i < kN; ++i) clip.process(x[i], x[i]);
    float peak = 0.0f;
    for (float v : x) peak = std::max(peak, std::fabs(v));
    check(finite(x), "clip stays finite");
    check(peak < 0.5f, "clip bounds the output");
  }

  // Harmonic density
  {
    Density density;
    density.prepare(kSr);
    DensityControls c;
    density.setControls(c);
    auto x = tone(1000.0f);
    for (int i = 0; i < kN; ++i) density.process(x[i], x[i]);
    check(finite(x), "density stays finite");
  }

  // De-esser
  {
    DeEss deess;
    deess.prepare(kSr);
    DeEssControls c;
    c.frequency = 1.0f;
    c.threshold = 0.2f;
    c.range = 1.0f;
    c.mix = 1.0f;
    deess.setControls(c);
    auto x = tone(10000.0f);
    float gr = 0.0f;
    for (int i = 0; i < kN; ++i) {
      deess.process(x[i], x[i]);
      gr = std::max(gr, deess.lastGrDb());
    }
    check(finite(x), "de-esser stays finite");
    check(gr > 0.1f, "de-esser reports gain reduction");
  }

  std::printf("%d failure(s)\n", gFailures);
  return gFailures == 0 ? 0 : 1;
}
