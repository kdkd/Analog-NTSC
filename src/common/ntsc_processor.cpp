// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#include "rf2/ntsc_processor.h"

namespace rf2 {

NtscProcessor::NtscProcessor() = default;

NtscProcessor::NtscProcessor(const NtscSignalConfig& config)
    : encoder_(config), effects_(config), decoder_(config) {}

void NtscProcessor::ProcessFrame(const uint8_t* rgb_in,
                                 std::vector<uint8_t>* rgb_out) {
  // Encode RGB directly to IRE floats, skipping the int16 round-trip.
  encoder_.EncodeFrameFloat(rgb_in, &frame_ire_);

  // Apply signal-domain effects (noise, multipath, VHS, etc.).
  effects_.Apply(&frame_ire_);

  // Decode the (potentially degraded) composite signal back to RGB.
  decoder_.Decode(frame_ire_, rgb_out);
}

void NtscProcessor::Reset() {
  encoder_.Reset();
}

bool NtscProcessor::GetParam(ParamId id, float* value) const {
  uint32_t raw = static_cast<uint32_t>(id);
  if (raw >= 100 && raw < 200) {
    return decoder_.GetParam(id, value);
  }
  if (raw >= 200 && raw < 300) {
    return effects_.GetParam(id, value);
  }
  // Encoder config params (300+) are accessed via the config struct directly.
  if (raw >= 300 && raw < 400) {
    if (!value) return false;
    const auto& c = encoder_.config();
    switch (id) {
      case ParamId::kLumaCutoffHz: *value = c.luma_cutoff_hz; return true;
      case ParamId::kICutoffHz: *value = c.i_cutoff_hz; return true;
      case ParamId::kQCutoffHz: *value = c.q_cutoff_hz; return true;
      case ParamId::kChromaModScale: *value = c.chroma_mod_scale; return true;
      case ParamId::kFilterPasses: *value = static_cast<float>(c.filter_passes); return true;
      default: return false;
    }
  }
  return false;
}

bool NtscProcessor::SetParam(ParamId id, float value) {
  uint32_t raw = static_cast<uint32_t>(id);
  if (raw >= 100 && raw < 200) {
    return decoder_.SetParam(id, value);
  }
  if (raw >= 200 && raw < 300) {
    return effects_.SetParam(id, value);
  }
  if (raw >= 300 && raw < 400) {
    auto& c = encoder_.config();
    switch (id) {
      case ParamId::kLumaCutoffHz: c.luma_cutoff_hz = value; return true;
      case ParamId::kICutoffHz: c.i_cutoff_hz = value; return true;
      case ParamId::kQCutoffHz: c.q_cutoff_hz = value; return true;
      case ParamId::kChromaModScale: c.chroma_mod_scale = value; return true;
      case ParamId::kFilterPasses: c.filter_passes = static_cast<uint32_t>(value); return true;
      default: return false;
    }
  }
  return false;
}

const ParamMeta* NtscProcessor::GetParamTable(size_t* count) {
  return rf2::GetParamTable(count);
}

}  // namespace rf2
