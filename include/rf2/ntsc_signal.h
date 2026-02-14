// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#pragma once

#include <cstdint>
#include <vector>

namespace rf2 {

struct NtscTiming {
  uint32_t lines_per_frame = 525;
  uint32_t samples_per_line = 910;  // 4*fsc sampling.
  uint32_t hsync_samples = 67;
  uint32_t back_porch_samples = 75;
  uint32_t active_start = 142;
  uint32_t active_samples = 754;  // ~52.66us active picture at 4*fsc.
  uint32_t burst_start = 80;
  uint32_t burst_samples = 36;  // 9 cycles at 4 samples/cycle.
};

struct NtscLevels {
  float sync_ire = -40.0f;
  float blank_ire = 0.0f;
  float black_ire = 7.5f;
  float white_ire = 100.0f;
  float burst_peak_ire = 20.0f;  // +/-20 IRE around blank.
  float chroma_gain_ire = 40.0f;
};

struct NtscLineMap {
  int top_field_start = 20;
  int bottom_field_start = 283;
  int lines_per_field_visible = 240;
};

struct NtscSignalConfig {
  NtscTiming timing{};
  NtscLevels levels{};
  NtscLineMap line_map{};
  // Approximate composite channel bandwidth shaping.
  float luma_cutoff_hz = 4.2e6f;
  float i_cutoff_hz = 1.1e6f;
  float q_cutoff_hz = 0.45e6f;
  float chroma_mod_scale = 0.88f;
  uint32_t filter_passes = 3;
};

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 6.2831853071795864769f;

int16_t IreToSample(float ire);
float SampleToIre(int16_t sample);

// Builds one frame of composite samples from 720x480 RGB24 source pixels.
void EncodeNtscCompositeFrame(const uint8_t* rgb720x480,
                              uint32_t frame_index,
                              const NtscSignalConfig& config,
                              std::vector<int16_t>* out_samples);

// Float variant – outputs IRE float samples directly, avoiding the
// lossy int16 quantisation round-trip.  Used by the real-time pipeline.
void EncodeNtscCompositeFrameFloat(const uint8_t* rgb720x480,
                                    uint32_t frame_index,
                                    const NtscSignalConfig& config,
                                    std::vector<float>* out_ire);

// Deprecated: use NtscEffects class from ntsc_effects.h instead.
// This wrapper is kept for backward compatibility.
void ApplyCompositeEffects(std::vector<float>* frame_ire,
                           float noise_stddev_ire,
                           float multipath_gain,
                           uint32_t multipath_delay_samples,
                           float line_time_jitter_samples,
                           float rf_drift,
                           float am_nonlinearity,
                           float impulse_noise,
                           float afc_hunt,
                           float chroma_flutter,
                           float agc_pump,
                           float multipath_ensemble,
                           float hum,
                           float yc_crosstalk,
                           float h_lock_noise_ire,
                           float v_hold_noise_ire,
                           float vhs_tracking,
                           float vhs_wrinkle,
                           float vhs_head_switch,
                           float vhs_dropouts,
                           float burst_lock_noise_ire,
                           uint32_t lines_per_frame,
                           uint32_t samples_per_line,
                           uint32_t hsync_samples,
                           uint32_t burst_start,
                           uint32_t burst_samples,
                           uint32_t random_seed);

}  // namespace rf2
