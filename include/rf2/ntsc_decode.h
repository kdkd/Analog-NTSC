// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#pragma once

// Backward-compatibility shim. New code should use ntsc_effects.h + ntsc_decoder.h directly.

#include "rf2/ntsc_effects.h"
#include "rf2/ntsc_decoder.h"

namespace rf2 {

// Backward-compatible wrapper that mimics the old combined decode+effects call.
// The old struct types (NtscDecodeControls, NtscEffectControls) are now defined
// in ntsc_decoder.h and ntsc_effects.h respectively. Field names for the lock
// noise params have changed:
//   h_lock_noise_ire  -> h_sync_noise_ire
//   v_hold_noise_ire  -> v_sync_noise_ire
//   burst_lock_noise_ire -> burst_noise_ire
//
// New code should use NtscEffects::Apply() + NtscDecoder::Decode() separately.
inline void DecodeNtscCompositeFrameToRgb(const std::vector<int16_t>& composite,
                                           const NtscSignalConfig& signal,
                                           const NtscDecodeControls& controls,
                                           const NtscEffectControls& effects,
                                           std::vector<uint8_t>* rgb_out) {
  // Convert int16 composite samples to IRE floats.
  std::vector<float> frame_ire(composite.size());
  for (size_t i = 0; i < composite.size(); ++i) {
    frame_ire[i] = SampleToIre(composite[i]);
  }

  // Apply signal-domain effects.
  NtscEffects fx(signal);
  fx.controls() = effects;
  fx.Apply(&frame_ire);

  // Decode to RGB.
  NtscDecoder decoder(signal);
  decoder.controls() = controls;
  // Map sync noise to decoder instability for backward compat:
  // the old API coupled these (noise_ire / 16 was used as instability).
  decoder.controls().h_lock_instability = effects.h_sync_noise_ire / 16.0f;
  decoder.controls().v_hold_instability = effects.v_sync_noise_ire / 16.0f;
  decoder.controls().burst_lock_instability = effects.burst_noise_ire / 16.0f;
  decoder.controls().random_seed = effects.random_seed;
  decoder.Decode(frame_ire, rgb_out);
}

}  // namespace rf2
