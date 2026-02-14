// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#pragma once

#include <cstdint>
#include <vector>

#include "rf2/ntsc_params.h"
#include "rf2/ntsc_signal.h"

namespace rf2 {

struct NtscDecodeControls {
  float brightness = 0.0f;
  float contrast = 1.0f;
  float saturation = 0.9f;
  float tint_degrees = 0.0f;
  float sharpness = 0.0f;
  bool comb_filter = true;
  bool dot_crawl = false;
  float chroma_delay_pixels = 0.0f;
  float overscan_reveal = 0.0f;
  float h_lock_instability = 0.0f;
  float v_hold_instability = 0.0f;
  float burst_lock_instability = 0.0f;
  uint32_t random_seed = 1;
};

class NtscDecoder {
 public:
  explicit NtscDecoder(const NtscSignalConfig& config = {});

  // Decodes a composite signal (in IRE float format) to packed RGB888 720x480.
  // The caller is responsible for applying effects beforehand if desired.
  void Decode(const std::vector<float>& frame_ire,
              std::vector<uint8_t>* rgb_out) const;

  bool GetParam(ParamId id, float* value) const;
  bool SetParam(ParamId id, float value);

  NtscDecodeControls& controls() { return controls_; }
  const NtscDecodeControls& controls() const { return controls_; }

  const NtscSignalConfig& config() const { return config_; }
  NtscSignalConfig& config() { return config_; }

 private:
  NtscDecodeControls controls_{};
  NtscSignalConfig config_{};
};

}  // namespace rf2
