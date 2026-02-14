// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#pragma once

#include <cstddef>
#include <cstdint>

namespace rf2 {

enum class ParamId : uint32_t {
  // -- Decode controls (100-199) --
  kBrightness = 100,
  kContrast = 101,
  kSaturation = 102,
  kTintDegrees = 103,
  kSharpness = 104,
  kCombFilter = 105,
  kDotCrawl = 106,
  kChromaDelayPixels = 107,
  kOverscanReveal = 108,
  kHLockInstability = 110,
  kVHoldInstability = 111,
  kBurstLockInstability = 112,
  kDecoderRandomSeed = 113,

  // -- Signal effects (200-299) --
  kNoiseStddevIre = 200,
  kMultipathGain = 201,
  kMultipathDelaySamples = 202,
  kMultipathEnsemble = 203,
  kLineTimeJitterSamples = 204,
  kAfcHunt = 205,
  kRfDrift = 206,
  kAmNonlinearity = 207,
  kImpulseNoise = 208,
  kAgcPump = 209,
  kHum = 210,
  kChromaFlutter = 211,
  kYcCrosstalk = 212,
  kHSyncNoiseIre = 213,
  kVSyncNoiseIre = 214,
  kBurstNoiseIre = 215,
  kVhsTracking = 216,
  kVhsWrinkle = 217,
  kVhsHeadSwitch = 218,
  kVhsDropouts = 219,
  kEffectsRandomSeed = 220,

  // -- Encoder config (300-399) --
  kLumaCutoffHz = 300,
  kICutoffHz = 301,
  kQCutoffHz = 302,
  kChromaModScale = 303,
  kFilterPasses = 304,

  // -- Signal effects (continued) --
  kNoiseColor = 221,
  kGroupDelay = 222,
};

enum class ParamType : uint8_t {
  kFloat,
  kBool,
  kUint32,
};

enum class ParamCategory : uint8_t {
  kDecode,
  kEffects,
  kEncoder,
};

struct ParamMeta {
  ParamId id;
  ParamType type;
  ParamCategory category;
  const char* name;
  const char* key;
  float min_val;
  float max_val;
  float default_val;
  const char* unit;
};

const ParamMeta* GetParamTable(size_t* count);
const ParamMeta* FindParamMeta(ParamId id);

}  // namespace rf2
