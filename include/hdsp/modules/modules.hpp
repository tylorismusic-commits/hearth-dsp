// hearth-dsp — shared processor modules
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Complete, reusable 500-series-style modules built on the hdsp primitives.
// They are deliberately control-agnostic: each module has a small typed
// controls struct, so a plugin can map it to its own parameter model.
#pragma once

#include "bleedout.hpp"
#include "cleanup.hpp"
#include "clip.hpp"
#include "colour.hpp"
#include "crush.hpp"
#include "comp.hpp"
#include "deess.hpp"
#include "double.hpp"
#include "dynamiceq.hpp"
#include "density.hpp"
#include "eq.hpp"
#include "filter.hpp"
#include "gate.hpp"
#include "image.hpp"
#include "limiter.hpp"
#include "motion.hpp"
#include "multiband.hpp"
#include "rider.hpp"
#include "sub.hpp"
#include "transient.hpp"
