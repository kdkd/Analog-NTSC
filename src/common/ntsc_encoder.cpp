// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#include "rf2/ntsc_encoder.h"

namespace rf2 {

NtscEncoder::NtscEncoder(const NtscSignalConfig& config) : config_(config) {}

void NtscEncoder::EncodeFrame(const uint8_t* rgb720x480,
                              std::vector<int16_t>* out) {
  EncodeNtscCompositeFrame(rgb720x480, frame_index_, config_, out);
  ++frame_index_;
}

void NtscEncoder::EncodeFrameAt(const uint8_t* rgb720x480,
                                uint32_t frame_index,
                                std::vector<int16_t>* out) {
  EncodeNtscCompositeFrame(rgb720x480, frame_index, config_, out);
}

void NtscEncoder::EncodeFrameFloat(const uint8_t* rgb720x480,
                                    std::vector<float>* out) {
  EncodeNtscCompositeFrameFloat(rgb720x480, frame_index_, config_, out);
  ++frame_index_;
}

void NtscEncoder::EncodeFrameAtFloat(const uint8_t* rgb720x480,
                                      uint32_t frame_index,
                                      std::vector<float>* out) {
  EncodeNtscCompositeFrameFloat(rgb720x480, frame_index, config_, out);
}

void NtscEncoder::Reset() { frame_index_ = 0; }

}  // namespace rf2
