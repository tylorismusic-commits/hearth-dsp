# Third-party notices

hearth-dsp is GPL-3.0-or-later. It was extracted from Comp Suite 1.0.0.

## JClones JSFX (MIT)

Several modules are C++ translations of MIT-licensed JClones JSFX from
[JClones/JSFXClones](https://github.com/JClones/JSFXClones):

`comp/molot.hpp`, `classic_limiter.hpp`, `tapehead.hpp`, `tubedriver.hpp`,
`datube.hpp`, `fattener.hpp`, `ac1.hpp`, `lmc1.hpp`, `modern_limiter.hpp`,
`hyrax.hpp`, `bloom.hpp`, `rs124.hpp`, `cl1b.hpp`, `du_tape.hpp`, `vbl.hpp`,
`l2.hpp` and `o3.hpp`.

```
MIT License

Copyright (c) 2026 JClones

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Algorithms and prior art

- **RBJ audio EQ cookbook** — the `Biquad` coefficient formulas are the
  standard Robert Bristow-Johnson cookbook equations.
- **ADAA** — first-order antiderivative anti-aliasing follows the method
  described by Parker, Zavalishin and Le Bivic (2016), "Reducing the aliasing
  of non-linear discrete-time systems by first-order antiderivative
  anti-aliasing". The implementation is original.
- **Kaiser window / half-band FIR** — the oversampler coefficients were
  generated with a Kaiser window (beta 8) and normalised to unity DC.
- **Catmull-Rom interpolation** — used by the true-peak detector.
- **NC76 reference law** — the shared `(ratio−1)/ratio` dynamics and the 76
  calibration in Comp Suite were matched against Tukan's MIT-licensed NC76
  reference. No JSFX code is copied into this library.

All other DSP in this repository is original or follows the published
algorithms above. HybridEq and CustomEq are original implementations inspired
by the layouts of well-known hardware/software EQs; no code was copied from
those products.
