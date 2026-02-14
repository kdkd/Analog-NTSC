// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#pragma once

#include <cstdint>
#include <vector>

#include "rf2/ntsc_params.h"
#include "rf2/ntsc_signal.h"

namespace rf2 {

struct NtscEffectControls {
  float noise_stddev_ire = 0.0f;
  float multipath_gain = 0.0f;
  uint32_t multipath_delay_samples = 0;
  float multipath_ensemble = 0.0f;
  float line_time_jitter_samples = 0.0f;
  float afc_hunt = 0.0f;
  float rf_drift = 0.0f;
  float am_nonlinearity = 0.0f;
  float impulse_noise = 0.0f;
  float agc_pump = 0.0f;
  float hum = 0.0f;
  float chroma_flutter = 0.0f;
  float yc_crosstalk = 0.0f;
  float h_sync_noise_ire = 0.0f;
  float v_sync_noise_ire = 0.0f;
  float burst_noise_ire = 0.0f;
  float vhs_tracking = 0.0f;
  float vhs_wrinkle = 0.0f;
  float vhs_head_switch = 0.0f;
  float vhs_dropouts = 0.0f;
  float noise_color = 0.0f;
  float group_delay = 0.0f;
  uint32_t random_seed = 1;
};

class NtscEffects {
 public:
  explicit NtscEffects(const NtscSignalConfig& config = {});

  void Apply(std::vector<float>* frame_ire) const;

  bool HasActiveEffects() const;
  bool GetParam(ParamId id, float* value) const;
  bool SetParam(ParamId id, float value);

  NtscEffectControls& controls() { return controls_; }
  const NtscEffectControls& controls() const { return controls_; }

  const NtscSignalConfig& config() const { return config_; }
  NtscSignalConfig& config() { return config_; }

 private:
  NtscEffectControls controls_{};
  NtscSignalConfig config_{};
};

// Internal implementation: applies effects using controls struct and signal
// config. Called by NtscEffects::Apply and the backward-compat shim.
void ApplyCompositeEffectsInternal(std::vector<float>* frame_ire,
                                   const NtscEffectControls& controls,
                                   const NtscSignalConfig& config);

}  // namespace rf2
