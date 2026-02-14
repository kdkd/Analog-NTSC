// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "rf2/ntsc_decoder.h"
#include "rf2/ntsc_effects.h"
#include "rf2/ntsc_encoder.h"
#include "rf2/ntsc_params.h"
#include "rf2/ntsc_signal.h"

namespace rf2 {

class NtscProcessor {
 public:
  NtscProcessor();
  explicit NtscProcessor(const NtscSignalConfig& config);

  void ProcessFrame(const uint8_t* rgb_in, std::vector<uint8_t>* rgb_out);
  void Reset();

  bool GetParam(ParamId id, float* value) const;
  bool SetParam(ParamId id, float value);

  static const ParamMeta* GetParamTable(size_t* count);

  NtscEncoder& encoder() { return encoder_; }
  NtscEffects& effects() { return effects_; }
  NtscDecoder& decoder() { return decoder_; }

 private:
  NtscEncoder encoder_;
  NtscEffects effects_;
  NtscDecoder decoder_;
  std::vector<float> frame_ire_;
};

}  // namespace rf2
