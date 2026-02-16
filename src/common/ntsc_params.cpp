// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#include "rf2/ntsc_params.h"

namespace rf2 {

static const ParamMeta kParamTable[] = {
    // -- Decode controls --
    {ParamId::kBrightness, ParamType::kFloat, ParamCategory::kDecode,
     "Brightness", "brightness", -0.5f, 0.5f, 0.0f, ""},
    {ParamId::kContrast, ParamType::kFloat, ParamCategory::kDecode,
     "Contrast", "contrast", 0.2f, 2.0f, 1.0f, ""},
    {ParamId::kSaturation, ParamType::kFloat, ParamCategory::kDecode,
     "Saturation", "saturation", 0.0f, 2.0f, 0.9f, ""},
    {ParamId::kTintDegrees, ParamType::kFloat, ParamCategory::kDecode,
     "Tint", "tint_degrees", -45.0f, 45.0f, 0.0f, "deg"},
    {ParamId::kSharpness, ParamType::kFloat, ParamCategory::kDecode,
     "Sharpness", "sharpness", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kCombFilter, ParamType::kBool, ParamCategory::kDecode,
     "Comb Filter", "comb_filter", 0.0f, 1.0f, 1.0f, ""},
    {ParamId::kDotCrawl, ParamType::kBool, ParamCategory::kDecode,
     "Dot Crawl", "dot_crawl", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kChromaDelayPixels, ParamType::kFloat, ParamCategory::kDecode,
     "Chroma Delay", "chroma_delay_pixels", -6.0f, 6.0f, 0.0f, "px"},
    {ParamId::kOverscanReveal, ParamType::kFloat, ParamCategory::kDecode,
     "Overscan Reveal", "overscan_reveal", 0.0f, 0.25f, 0.0f, ""},
    {ParamId::kHLockInstability, ParamType::kFloat, ParamCategory::kDecode,
     "H Lock Instability", "h_lock_instability", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kVHoldInstability, ParamType::kFloat, ParamCategory::kDecode,
     "V Hold Instability", "v_hold_instability", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kBurstLockInstability, ParamType::kFloat, ParamCategory::kDecode,
     "Burst Lock Instability", "burst_lock_instability", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kDecoderRandomSeed, ParamType::kUint32, ParamCategory::kDecode,
     "Decoder Seed", "decoder_random_seed", 1.0f, 4294967295.0f, 1.0f, ""},

    // -- Signal effects --
    {ParamId::kNoiseStddevIre, ParamType::kFloat, ParamCategory::kEffects,
     "Noise", "noise_stddev_ire", 0.0f, 20.0f, 0.0f, "IRE"},
    {ParamId::kMultipathGain, ParamType::kFloat, ParamCategory::kEffects,
     "Ghost Gain", "multipath_gain", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kMultipathDelaySamples, ParamType::kUint32, ParamCategory::kEffects,
     "Ghost Delay", "multipath_delay_samples", 1.0f, 200.0f, 24.0f, "samples"},
    {ParamId::kMultipathEnsemble, ParamType::kFloat, ParamCategory::kEffects,
     "Ghost Ensemble", "multipath_ensemble", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kLineTimeJitterSamples, ParamType::kFloat, ParamCategory::kEffects,
     "Line Jitter", "line_time_jitter_samples", 0.0f, 2.0f, 0.0f, "samples"},
    {ParamId::kAfcHunt, ParamType::kFloat, ParamCategory::kEffects,
     "AFC Hunt", "afc_hunt", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kRfDrift, ParamType::kFloat, ParamCategory::kEffects,
     "RF Drift", "rf_drift", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kAmNonlinearity, ParamType::kFloat, ParamCategory::kEffects,
     "AM Nonlinearity", "am_nonlinearity", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kImpulseNoise, ParamType::kFloat, ParamCategory::kEffects,
     "Impulse Noise", "impulse_noise", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kAgcPump, ParamType::kFloat, ParamCategory::kEffects,
     "AGC Pump", "agc_pump", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kHum, ParamType::kFloat, ParamCategory::kEffects,
     "Hum", "hum", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kChromaFlutter, ParamType::kFloat, ParamCategory::kEffects,
     "Chroma Flutter", "chroma_flutter", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kYcCrosstalk, ParamType::kFloat, ParamCategory::kEffects,
     "Y/C Crosstalk", "yc_crosstalk", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kHSyncNoiseIre, ParamType::kFloat, ParamCategory::kEffects,
     "H Sync Noise", "h_sync_noise_ire", 0.0f, 16.0f, 0.0f, "IRE"},
    {ParamId::kVSyncNoiseIre, ParamType::kFloat, ParamCategory::kEffects,
     "V Sync Noise", "v_sync_noise_ire", 0.0f, 16.0f, 0.0f, "IRE"},
    {ParamId::kBurstNoiseIre, ParamType::kFloat, ParamCategory::kEffects,
     "Burst Noise", "burst_noise_ire", 0.0f, 16.0f, 0.0f, "IRE"},
    {ParamId::kVhsTracking, ParamType::kFloat, ParamCategory::kEffects,
     "VHS Tracking", "vhs_tracking", -1.0f, 1.0f, 0.0f, ""},
    {ParamId::kVhsWrinkle, ParamType::kFloat, ParamCategory::kEffects,
     "VHS Wrinkle", "vhs_wrinkle", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kVhsHeadSwitch, ParamType::kFloat, ParamCategory::kEffects,
     "VHS Head Switch", "vhs_head_switch", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kVhsDropouts, ParamType::kFloat, ParamCategory::kEffects,
     "VHS Dropouts", "vhs_dropouts", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kEffectsRandomSeed, ParamType::kUint32, ParamCategory::kEffects,
     "Effects Seed", "effects_random_seed", 1.0f, 4294967295.0f, 1.0f, ""},
    {ParamId::kNoiseColor, ParamType::kFloat, ParamCategory::kEffects,
     "Noise Color", "noise_color", 0.0f, 1.0f, 0.0f, ""},
    {ParamId::kGroupDelay, ParamType::kFloat, ParamCategory::kEffects,
     "Group Delay", "group_delay", 0.0f, 1.0f, 0.0f, ""},

    // -- Encoder config --
    {ParamId::kLumaCutoffHz, ParamType::kFloat, ParamCategory::kEncoder,
     "Luma Cutoff", "luma_cutoff_hz", 0.5e6f, 6.0e6f, 4.2e6f, "Hz"},
    {ParamId::kICutoffHz, ParamType::kFloat, ParamCategory::kEncoder,
     "I Cutoff", "i_cutoff_hz", 0.2e6f, 1.5e6f, 1.1e6f, "Hz"},
    {ParamId::kQCutoffHz, ParamType::kFloat, ParamCategory::kEncoder,
     "Q Cutoff", "q_cutoff_hz", 0.1e6f, 0.6e6f, 0.45e6f, "Hz"},
    {ParamId::kChromaModScale, ParamType::kFloat, ParamCategory::kEncoder,
     "Chroma Mod Scale", "chroma_mod_scale", 0.5f, 1.5f, 0.88f, ""},
    {ParamId::kFilterPasses, ParamType::kUint32, ParamCategory::kEncoder,
     "Filter Passes", "filter_passes", 1.0f, 6.0f, 3.0f, ""},
};

static constexpr size_t kParamCount = sizeof(kParamTable) / sizeof(kParamTable[0]);

const ParamMeta* GetParamTable(size_t* count) {
  if (count) {
    *count = kParamCount;
  }
  return kParamTable;
}

const ParamMeta* FindParamMeta(ParamId id) {
  for (size_t i = 0; i < kParamCount; ++i) {
    if (kParamTable[i].id == id) {
      return &kParamTable[i];
    }
  }
  return nullptr;
}

}  // namespace rf2
