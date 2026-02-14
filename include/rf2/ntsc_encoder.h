// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#pragma once

#include <cstdint>
#include <vector>

#include "rf2/ntsc_params.h"
#include "rf2/ntsc_signal.h"

namespace rf2 {

class NtscEncoder {
 public:
  explicit NtscEncoder(const NtscSignalConfig& config = {});

  void EncodeFrame(const uint8_t* rgb720x480, std::vector<int16_t>* out);
  void EncodeFrameAt(const uint8_t* rgb720x480, uint32_t frame_index,
                     std::vector<int16_t>* out);

  // Float variants – output IRE directly, skipping int16 quantisation.
  void EncodeFrameFloat(const uint8_t* rgb720x480, std::vector<float>* out);
  void EncodeFrameAtFloat(const uint8_t* rgb720x480, uint32_t frame_index,
                          std::vector<float>* out);

  void Reset();
  uint32_t frame_index() const { return frame_index_; }
  void set_frame_index(uint32_t idx) { frame_index_ = idx; }

  const NtscSignalConfig& config() const { return config_; }
  NtscSignalConfig& config() { return config_; }

 private:
  NtscSignalConfig config_{};
  uint32_t frame_index_ = 0;
};

}  // namespace rf2
