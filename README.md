# hearth-dsp

Header-only DSP modules shared by Comp Suite, Suite Rack and future plugins.
GPL-3.0-or-later.

Extracted from Comp Suite 1.0.0 so the hard-won pieces — ADAA saturation,
per-channel oversampling, the ratio ladder — live in one place instead of
being copied into every plugin. The shared processor modules now include the
Suite Rack EQ, compressor, colour, gate, transient, clip and de-esser stages.

## Modules

- **Crossover** — 4th-order Linkwitz-Riley low/high split, three-band and
  four-band serial splitters for multiband saturation.
- **Spectrum** — real-time FFT analyser with 128 log-spaced bins for editor
  meters.

| Header | Contents |
|---|---|
| `hdsp/util.hpp` | `kPi`, `clampf` |
| `hdsp/db.hpp` | `dbToGain`, `gainToDb`, `timeCoeff` |
| `hdsp/smoother.hpp` | `Smoothed` one-pole parameter smoother |
| `hdsp/envelope.hpp` | `OnePoleLP`, `HighPass2`, `settleK` |
| `hdsp/biquad.hpp` | RBJ `Biquad` (shelves, peak, LP, HP) |
| `hdsp/adaa.hpp` | `Adaa`, `logCosh`, `AsymShaper` (ADAA saturator) |
| `hdsp/dynamics.hpp` | ratio ladder, soft-knee gain computer |
| `hdsp/sidechain.hpp` | `Topology`, `SidechainFrame` |
| `hdsp/truepeak.hpp` | 4× Catmull-Rom `TruePeakDetector` |
| `hdsp/oversample.hpp` | Off/2×/4× half-band `Oversampler` |

### Processor modules

| Header | Contents |
|---|---|
| `hdsp/modules/eq.hpp` | `Eq`, `EqControls` — 4K, Hearth, Tilt and Air voicings |
| `hdsp/modules/eq510.hpp` | `Eq510`, `Eq510Controls` — fixed-band 510-style EQ |
| `hdsp/modules/comp.hpp` | `Comp`, `CompControls` — 76 Blue, 76 Black, 2A, 3A and VCA |
| `hdsp/modules/comp/` | FET, optical and VCA model implementations |
| `hdsp/modules/colour.hpp` | `Colour`, `ColourControls` — Valve, Console, Bus, Tape, Transformer |
| `hdsp/modules/crush.hpp` | `Crush`, `CrushControls` — bitcrusher |
| `hdsp/modules/image.hpp` | `Image`, `ImageControls` — width, mono-maker, balance, M/S |
| `hdsp/modules/bleedout.hpp` | `BleedOut`, `BleedOutControls` — multiband de-bleed gate |
| `hdsp/modules/filter.hpp` | `Filter`, `FilterControls` — auto-filter / envelope filter |
| `hdsp/modules/gate.hpp` | `Gate`, `GateControls` |
| `hdsp/modules/limiter.hpp` | `Limiter`, `LimiterControls` — lookahead peak limiter |
| `hdsp/modules/lmc1.hpp` | `LMC1` — JClones console compressor |
| `hdsp/modules/modern_limiter.hpp` | `ModernLimiter` — JClones lookahead limiter |
| `hdsp/modules/hyrax.hpp` | `Hyrax` — JClones linear-phase brickwall limiter |
| `hdsp/modules/bloom.hpp` | `Bloom` — JClones harmonic enhancer |
| `hdsp/modules/rs124.hpp` | `RS124` — JClones vari-mu compressor |
| `hdsp/modules/cl1b.hpp` | `CL1B` — JClones optical compressor |
| `hdsp/modules/du_tape.hpp` | `DuTape` — JClones tape saturator |
| `hdsp/modules/vbl.hpp` | `VBL` — JClones tube broadcast limiter |
| `hdsp/modules/motion.hpp` | `Motion`, `MotionControls` — tremolo, auto-pan, rotary |
| `hdsp/modules/pultec.hpp` | `Pultec`, `PultecControls` — low/high boost and cut |
| `hdsp/modules/multiband.hpp` | `Multiband`, `MultibandControls` — three-band compressor |
| `hdsp/modules/reel.hpp` | `Reel`, `ReelControls` — reel/cassette tape stage |
| `hdsp/modules/rider.hpp` | `Rider`, `RiderControls` — auto-level / vocal rider |
| `hdsp/modules/space.hpp` | `SpaceReverb`, `SpaceDelay`, `SpaceMod` wrappers |
| `hdsp/modules/space/` | Space Suite reverb, delay and modulation engines |
| `hdsp/modules/sub.hpp` | `Sub`, `SubControls` — subharmonic synthesizer |
| `hdsp/modules/tame.hpp` | `Tame`, `TameControls` — Trowel resonance suppressor |
| `hdsp/modules/tapehead.hpp` | `TapeHead` — tape saturation |
| `hdsp/modules/tubedriver.hpp` | `TubeDriver` — tube saturation |
| `hdsp/modules/transient.hpp` | `Transient`, `TransientControls` |
| `hdsp/modules/ac1.hpp` | `AC1` — JClones console channel |
| `hdsp/modules/classic_limiter.hpp` | `ClassicLimiter` — JClones-derived limiter |
| `hdsp/modules/cleanup.hpp` | `Cleanup`, `CleanupControls` — de-click / de-hum |
| `hdsp/modules/clip.hpp` | `Clip`, `ClipControls` |
| `hdsp/modules/datube.hpp` | `DaTube` — tube saturation |
| `hdsp/modules/fattener.hpp` | `Fattener` — soft clipper / enhancer |
| `hdsp/modules/deess.hpp` | `DeEss`, `DeEssControls` |
| `hdsp/modules/double.hpp` | `Double`, `DoubleControls` — doubler / microshift |
| `hdsp/modules/dynamiceq.hpp` | `DynamicEq`, `DynamicEqControls` — single-band dynamic EQ |
| `hdsp/modules/ember.hpp` | `Ember`, `EmberControls` — one-knob finish |
| `hdsp/modules/density.hpp` | `Density`, `DensityControls` |

`hdsp/hdsp.hpp` includes both the primitives and the processor modules. The
modules expose small typed control structs rather than plugin-specific
parameter IDs, so a plugin maps its own controls onto them.

## Usage

```cmake
add_subdirectory(hearth-dsp)
target_link_libraries(my_plugin PRIVATE hdsp)
```

```cpp
#include <hdsp/hdsp.hpp>

hdsp::Smoothed gain;
gain.setTime(15.0f, 48000.0f);
gain.setTarget(hdsp::dbToGain(-6.0f));
```

Namespace is `hdsp`. The headers have no dependencies beyond the C++17
standard library and no global state.

## Oversampling

`hdsp::Oversampler` is a stateful 2×/4× converter. Call `prepare(1|2|4)`, feed
it blocks of at most `kMaxBaseFrames` base-rate frames with `up()`, run the
model at the returned higher rate, then call `down()`. It keeps separate filter
state per channel. Reported latency is 0 / 30 / 45 base-rate samples.

## Tests

```sh
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

The tests cover DC unity, latency, left/right isolation (the regression that
caused a right-side buzz in Comp Suite), biquad responses, ADAA bounds, the
gain computer, smoothing, true-peak detection and the shared processor
modules.

## Provenance

See `THIRD_PARTY_NOTICES.md`. The code was extracted from Comp Suite 1.0.0 and
keeps its GPL-3.0-or-later licence. `Comp Suite` itself lives at
<https://github.com/tylorismusic-commits/comp-suite>.
