// hearth-dsp — shared processor modules
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Complete, reusable 500-series-style modules built on the hdsp primitives.
// They are deliberately control-agnostic: each module has a small typed
// controls struct, so a plugin can map it to its own parameter model.
#pragma once

#include "bleedout.hpp"
#include "classic_limiter.hpp"
#include "cleanup.hpp"
#include "ac1.hpp"
#include "clip.hpp"
#include "colour.hpp"
#include "crush.hpp"
#include "datube.hpp"
#include "fattener.hpp"
#include "comp.hpp"
#include "deess.hpp"
#include "double.hpp"
#include "ember.hpp"
#include "dynamiceq.hpp"
#include "density.hpp"
#include "eq.hpp"
#include "eq510.hpp"
#include "filter.hpp"
#include "gate.hpp"
#include "image.hpp"
#include "limiter.hpp"
#include "lmc1.hpp"
#include "modern_limiter.hpp"
#include "motion.hpp"
#include "pultec.hpp"
#include "multiband.hpp"
#include "reel.hpp"
#include "rider.hpp"
#include "space.hpp"
#include "tapehead.hpp"
#include "tubedriver.hpp"
#include "sub.hpp"
#include "tame.hpp"
#include "transient.hpp"
