// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#include "rf2/ntsc_effects.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <random>

namespace rf2 {

namespace {

inline float Clamp(float value, float lo, float hi) {
  return std::max(lo, std::min(hi, value));
}

}  // namespace

void ApplyCompositeEffectsInternal(std::vector<float>* frame_ire,
                                   const NtscEffectControls& controls,
                                   const NtscSignalConfig& config) {
  if (!frame_ire || frame_ire->empty()) {
    return;
  }

  // Unpack struct fields into local variables matching the old parameter names
  // so the body code below is unchanged from the original extraction.
  const float noise_stddev_ire = controls.noise_stddev_ire;
  const float multipath_gain = controls.multipath_gain;
  const uint32_t multipath_delay_samples = controls.multipath_delay_samples;
  const float line_time_jitter_samples = controls.line_time_jitter_samples;
  const float rf_drift = controls.rf_drift;
  const float am_nonlinearity = controls.am_nonlinearity;
  const float impulse_noise = controls.impulse_noise;
  const float afc_hunt = controls.afc_hunt;
  const float chroma_flutter = controls.chroma_flutter;
  const float agc_pump = controls.agc_pump;
  const float multipath_ensemble = controls.multipath_ensemble;
  const float hum = controls.hum;
  const float yc_crosstalk = controls.yc_crosstalk;
  const float h_lock_noise_ire = controls.h_sync_noise_ire;
  const float v_hold_noise_ire = controls.v_sync_noise_ire;
  const float burst_lock_noise_ire = controls.burst_noise_ire;
  const float vhs_tracking = controls.vhs_tracking;
  const float vhs_wrinkle = controls.vhs_wrinkle;
  const float vhs_head_switch = controls.vhs_head_switch;
  const float vhs_dropouts = controls.vhs_dropouts;
  const float noise_color = Clamp(controls.noise_color, 0.0f, 1.0f);
  const float group_delay_strength = Clamp(controls.group_delay, 0.0f, 1.0f);
  const uint32_t random_seed = controls.random_seed;
  const uint32_t lines_per_frame = config.timing.lines_per_frame;
  const uint32_t samples_per_line = config.timing.samples_per_line;
  const uint32_t hsync_samples = config.timing.hsync_samples;
  const uint32_t burst_start = config.timing.burst_start;
  const uint32_t burst_samples = config.timing.burst_samples;

  // ---- Body below is identical to the original ApplyCompositeEffects ----

  std::mt19937 rng(random_seed);
  std::normal_distribution<float> noise(0.0f, std::max(0.0f, noise_stddev_ire));
  std::uniform_real_distribution<float> uni01(0.0f, 1.0f);
  std::normal_distribution<float> gauss01(0.0f, 1.0f);

  const float expected_blank_ire = config.levels.blank_ire;
  const float expected_sync_ire = config.levels.sync_ire;

  // Estimate the blanking-interval (back porch) level for a scanline.
  // Used by DC restoration after multipath and by AGC pump.
  auto estimate_blank_simple = [&](const float* row) -> float {
    float sum = 0.0f;
    uint32_t n = 0;

    const uint32_t pre_start =
        std::min<uint32_t>(samples_per_line, hsync_samples + 6U);
    const uint32_t pre_end = std::min<uint32_t>(
        samples_per_line, (burst_start > 2U) ? (burst_start - 2U) : burst_start);
    for (uint32_t s = pre_start; s < pre_end; ++s) {
      sum += row[s];
      ++n;
    }

    const uint32_t post_start = std::min<uint32_t>(
        samples_per_line, burst_start + burst_samples + 3U);
    const uint32_t active_start = std::max<uint32_t>(
        hsync_samples + 40U, burst_start + burst_samples + 4U);
    const uint32_t post_end = std::min<uint32_t>(
        samples_per_line,
        (active_start > 2U) ? (active_start - 2U) : active_start);
    for (uint32_t s = post_start; s < post_end; ++s) {
      sum += row[s];
      ++n;
    }

    return n > 0 ? (sum / static_cast<float>(n)) : 0.0f;
  };

  // Estimate the flat interior of the horizontal sync pulse (avoid edges).
  auto estimate_sync_simple = [&](const float* row) -> float {
    const uint32_t start = std::min<uint32_t>(samples_per_line, 12U);
    const uint32_t end = std::min<uint32_t>(
        samples_per_line, (hsync_samples > 24U) ? (hsync_samples - 12U) : hsync_samples);
    if (start >= end) {
      return expected_sync_ire;
    }
    float sum = 0.0f;
    uint32_t n = 0;
    for (uint32_t s = start; s < end; ++s) {
      sum += row[s];
      ++n;
    }
    return n > 0 ? (sum / static_cast<float>(n)) : expected_sync_ire;
  };

  if (multipath_gain != 0.0f && multipath_delay_samples > 0) {
    std::vector<float> base = *frame_ire;
    const size_t delay = multipath_delay_samples;
    for (size_t i = 0; i < frame_ire->size(); ++i) {
      const float ghost = (i >= delay) ? base[i - delay] * multipath_gain : 0.0f;
      (*frame_ire)[i] = Clamp(base[i] + ghost, -60.0f, 140.0f);
    }
  }

  const float rf_drift_strength = Clamp(rf_drift, 0.0f, 1.0f);
  const float am_nl_strength = Clamp(am_nonlinearity, 0.0f, 1.0f);
  const float impulse_strength = Clamp(impulse_noise, 0.0f, 1.0f);
  const float afc_hunt_strength = Clamp(afc_hunt, 0.0f, 1.0f);
  const float chroma_flutter_strength = Clamp(chroma_flutter, 0.0f, 1.0f);
  const float agc_pump_strength = Clamp(agc_pump, 0.0f, 1.0f);
  const float multipath_ensemble_strength = Clamp(multipath_ensemble, 0.0f, 1.0f);
  const float hum_strength = Clamp(hum, 0.0f, 1.0f);
  const float yc_crosstalk_strength = Clamp(yc_crosstalk, 0.0f, 1.0f);

  if (multipath_ensemble_strength > 0.0f && samples_per_line > 0) {
    std::vector<float> base = *frame_ire;
    const size_t base_delay = std::max<size_t>(1U, multipath_delay_samples);
    const float phase = 0.0039f * static_cast<float>(random_seed);
    const size_t d1 = base_delay + static_cast<size_t>(3 + std::lrintf(9.0f * multipath_ensemble_strength));
    const size_t d2 = base_delay + static_cast<size_t>(11 + std::lrintf(23.0f * multipath_ensemble_strength));
    const size_t d3 = base_delay + static_cast<size_t>(27 + std::lrintf(41.0f * multipath_ensemble_strength));
    const float gbase = 0.14f * multipath_ensemble_strength + 0.07f * std::fabs(multipath_gain);
    const float g1 = gbase * (0.9f + 0.4f * std::sin(phase * 0.71f));
    const float g2 = -0.75f * gbase * (0.9f + 0.4f * std::sin(phase * 0.49f + 1.3f));
    const float g3 = 0.52f * gbase * (0.9f + 0.4f * std::sin(phase * 0.31f + 2.1f));
    for (size_t i = 0; i < frame_ire->size(); ++i) {
      float v = base[i];
      if (i >= d1) {
        v += g1 * base[i - d1];
      }
      if (i >= d2) {
        v += g2 * base[i - d2];
      }
      if (i >= d3) {
        v += g3 * base[i - d3];
      }
      (*frame_ire)[i] = Clamp(v, -80.0f, 140.0f);
    }
  }

  // DC restoration: a real receiver's black-level clamp measures the back
  // porch each line and restores it to reference black (0 IRE).  Without
  // this, multipath ghosts raise the overall brightness unrealistically.
  if ((multipath_gain != 0.0f || multipath_ensemble_strength > 0.0f) &&
      lines_per_frame > 0 && samples_per_line > 0) {
    // After DC restoration, apply a simple sync-referenced AGC. This avoids
    // permanently compressing the entire signal just because a delayed echo
    // was added (which makes VBI elements like VITC look too dark), while still
    // allowing sync-overlap cases to dim/brighten via AGC like a real receiver.
    const float expected_depth = expected_blank_ire - expected_sync_ire;
    float depth_sum = 0.0f;
    uint32_t depth_count = 0;
    float min_v = 1e9f;
    float max_v = -1e9f;
    for (uint32_t line = 0; line < lines_per_frame; ++line) {
      float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
      const float blank_meas = estimate_blank_simple(row);
      const float sync_meas = estimate_sync_simple(row);
      // Broad vertical sync pulses can extend through the would-be back porch,
      // making "blank" measurements meaningless. Real clamps/AGC loops are
      // typically gated off during these intervals.
      if ((blank_meas - sync_meas) < 15.0f) {
        continue;
      }
      const float blank_err = blank_meas - expected_blank_ire;
      for (uint32_t s = 0; s < samples_per_line; ++s) {
        row[s] -= blank_err;
        min_v = std::min(min_v, row[s]);
        max_v = std::max(max_v, row[s]);
      }

      const float sync_level = estimate_sync_simple(row);
      const float depth = expected_blank_ire - sync_level;
      if (depth > 10.0f) {
        depth_sum += depth;
        ++depth_count;
      }
    }

    // Receiver AGC is typically keyed off sync amplitude, but in the presence
    // of a delayed echo, the sync pulse can appear "deeper" simply because the
    // echo overlaps the later portion of the sync interval. Real sync/AGC paths
    // include limiting/clipping, so we avoid applying AGC gain-down purely
    // because sync got deeper than nominal. We only apply gain-up when sync is
    // weakened (e.g. cancellation).
    float gain = 1.0f;
    if (depth_count > 0 && expected_depth > 1.0f) {
      float measured_depth = depth_sum / static_cast<float>(depth_count);
      measured_depth = std::max(1.0f, std::min(measured_depth, expected_depth));
      if (measured_depth < expected_depth * 0.98f) {
        gain = expected_depth / measured_depth;
      }
    }
    gain = Clamp(gain, 1.0f, 1.7f);

    // Limit gain-up cases (weak sync) to avoid exploding highlights.
    if (max_v > 1e-3f) {
      gain = std::min(gain, 140.0f / max_v);
    }
    if (min_v < -1e-3f) {
      gain = std::min(gain, -80.0f / min_v);
    }
    gain = std::max(0.25f, gain);

    for (size_t i = 0; i < frame_ire->size(); ++i) {
      (*frame_ire)[i] = Clamp((*frame_ire)[i] * gain, -80.0f, 140.0f);
    }
  }

  // Group delay: causal-only lowpass adds frequency-dependent phase shift,
  // creating the characteristic chroma/luma timing misalignment of real
  // analog filters.
  if (group_delay_strength > 0.0f && lines_per_frame > 0 && samples_per_line > 0) {
    constexpr float kCompositeSampleRateHz = 315000000.0f / 22.0f;
    const float cutoff = 6.0e6f - 4.0e6f * group_delay_strength;
    const float gd_alpha = 1.0f - std::exp(-kTwoPi * cutoff / kCompositeSampleRateHz);
    for (uint32_t line = 0; line < lines_per_frame; ++line) {
      float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
      float y = row[0];
      for (uint32_t s = 1; s < samples_per_line; ++s) {
        y += gd_alpha * (row[s] - y);
        row[s] = y;
      }
    }
  }

  if (line_time_jitter_samples > 0.0f) {
    std::normal_distribution<float> jitter_dist(0.0f, line_time_jitter_samples);
    std::vector<float> src_line(samples_per_line);
    for (uint32_t line = 0; line < lines_per_frame; ++line) {
      float* dst = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
      std::copy(dst, dst + samples_per_line, src_line.begin());
      const float shift = jitter_dist(rng);
      for (uint32_t s = 0; s < samples_per_line; ++s) {
        float pos = static_cast<float>(s) - shift;
        pos = Clamp(pos, 0.0f, static_cast<float>(samples_per_line - 1));
        const uint32_t p = static_cast<uint32_t>(std::lrintf(pos));
        dst[s] = src_line[p];
      }
    }
  }

  if (rf_drift_strength > 0.0f && lines_per_frame > 0 && samples_per_line > 0) {
    std::vector<float> drift_scratch(samples_per_line);
    const float frame_phase = 0.0042f * static_cast<float>(random_seed);
    const float global_gain = 1.0f + 0.22f * rf_drift_strength * std::sin(frame_phase * 0.73f);
    const float global_bias = 9.0f * rf_drift_strength * std::sin(frame_phase * 0.19f + 0.7f);
    constexpr int kFieldPairDelta = 263;
    auto paired_phase_line = [&](uint32_t line) -> float {
      const int l = static_cast<int>(line);
      const int p = (l < kFieldPairDelta) ? (l + kFieldPairDelta) : (l - kFieldPairDelta);
      if (p < 0 || static_cast<uint32_t>(p) >= lines_per_frame) {
        return static_cast<float>(l);
      }
      return static_cast<float>(std::min(l, p));
    };
    for (uint32_t line = 0; line < lines_per_frame; ++line) {
      float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
      const float blank = estimate_blank_simple(row);
      const float pair_line = paired_phase_line(line);
      const float line_gain =
          global_gain +
          0.075f * rf_drift_strength *
              std::sin(frame_phase * 0.61f + 0.031f * pair_line);
      const float line_bias =
          global_bias +
          3.5f * rf_drift_strength *
              std::sin(frame_phase * 0.43f + 0.024f * pair_line);
      const float line_wander_phase =
          frame_phase * 0.47f + 0.13f * pair_line + 1.2f;
      for (uint32_t s = 0; s < samples_per_line; ++s) {
        const float centered = row[s] - blank;
        const float wander =
            4.2f * rf_drift_strength *
            std::sin(line_wander_phase + 0.015f * static_cast<float>(s));
        row[s] = Clamp(blank + line_bias + wander + centered * line_gain, -80.0f, 140.0f);
      }

      std::copy(row, row + samples_per_line, drift_scratch.begin());
      const float drift_shift =
          (0.10f + 2.6f * rf_drift_strength) *
          (std::sin(frame_phase * 0.41f + 0.059f * pair_line) +
           0.35f * std::sin(frame_phase * 0.13f + 0.017f * pair_line + 1.1f));
      if (std::fabs(drift_shift) > 0.03f) {
        for (uint32_t s = 0; s < samples_per_line; ++s) {
          float pos = static_cast<float>(s) - drift_shift;
          pos = Clamp(pos, 0.0f, static_cast<float>(samples_per_line - 1));
          const uint32_t p0 = static_cast<uint32_t>(pos);
          const uint32_t p1 = std::min<uint32_t>(p0 + 1, samples_per_line - 1);
          const float frac = pos - static_cast<float>(p0);
          row[s] = drift_scratch[p0] + (drift_scratch[p1] - drift_scratch[p0]) * frac;
        }
      }
    }
  }

  if (am_nl_strength > 0.0f && lines_per_frame > 0 && samples_per_line > 0) {
    const uint32_t am_start_s = std::min<uint32_t>(
        samples_per_line - 1,
        std::max<uint32_t>(hsync_samples + 40U, burst_start + burst_samples + 6U));
    const uint32_t am_end_s = std::max<uint32_t>(am_start_s + 1, samples_per_line - 8U);
    const float hi_knee = 68.0f - 20.0f * am_nl_strength;
    const float lo_knee = 40.0f - 12.0f * am_nl_strength;
    const float hi_comp = 0.010f + 0.085f * am_nl_strength;
    const float lo_comp = 0.013f + 0.110f * am_nl_strength;
    for (uint32_t line = 0; line < lines_per_frame; ++line) {
      float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
      const float blank = estimate_blank_simple(row);
      if (am_end_s <= am_start_s) {
        continue;
      }
      // Compress raw composite directly — luma/chroma intermodulation
      // products emerge naturally from nonlinear processing of the
      // combined signal.
      for (uint32_t s = am_start_s; s < am_end_s; ++s) {
        float v = row[s] - blank;
        if (v > hi_knee) {
          const float over = v - hi_knee;
          v = hi_knee + over / (1.0f + hi_comp * over);
        } else if (v < -lo_knee) {
          const float over = (-lo_knee) - v;
          v = -lo_knee - over / (1.0f + lo_comp * over);
        }
        v += (0.00007f + 0.00055f * am_nl_strength) * v * std::fabs(v);
        row[s] = Clamp(blank + v, -80.0f, 140.0f);
      }
    }
  }

  if (impulse_strength > 0.0f && lines_per_frame > 0 && samples_per_line > 0) {
    const int events = std::max(
        1,
        static_cast<int>(std::lrintf((1.0f + 58.0f * impulse_strength) *
                                     (0.4f + 0.6f * uni01(rng)))));
    for (int i = 0; i < events; ++i) {
      const uint32_t line = static_cast<uint32_t>(std::lrintf(
          uni01(rng) * static_cast<float>(std::max<uint32_t>(1, lines_per_frame - 1))));
      float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
      const uint32_t start = static_cast<uint32_t>(std::lrintf(
          uni01(rng) * static_cast<float>(std::max<uint32_t>(1, samples_per_line - 1))));
      const uint32_t len = 1U + static_cast<uint32_t>(std::lrintf(
                                   (4.0f + 38.0f * impulse_strength) * uni01(rng)));
      const uint32_t end = std::min<uint32_t>(samples_per_line, start + len);
      float amp = (10.0f + 86.0f * impulse_strength) *
                  (uni01(rng) < 0.58f ? 1.0f : -1.0f);
      float walk = 0.0f;
      for (uint32_t s = start; s < end; ++s) {
        walk = 0.76f * walk + gauss01(rng) * (0.5f + 3.8f * impulse_strength);
        row[s] = Clamp(row[s] + amp + walk, -80.0f, 140.0f);
        amp *= 0.70f;
      }
    }
  }

  if (hum_strength > 0.0f && lines_per_frame > 0 && samples_per_line > 0) {
    const float phase = 0.0047f * static_cast<float>(random_seed);
    const float lines_f = std::max(1.0f, static_cast<float>(lines_per_frame));
    const float roll = (0.8f + 4.2f * hum_strength) * (kTwoPi / lines_f);
    const float dir = 1.0f;
    const float approx_fps = 15734.264f / lines_f;
    const float target_period_s = 3.0f - 1.0f * hum_strength;
    const float phase_step =
        (kTwoPi / std::max(1.0f, approx_fps * target_period_s)) * dir;
    const float frame_roll_phase = static_cast<float>(random_seed) * phase_step;
    constexpr int kFieldPairDelta = 263;
    auto paired_phase_line = [&](uint32_t line) -> float {
      const int l = static_cast<int>(line);
      const int p = (l < kFieldPairDelta) ? (l + kFieldPairDelta) : (l - kFieldPairDelta);
      if (p < 0 || static_cast<uint32_t>(p) >= lines_per_frame) {
        return static_cast<float>(l);
      }
      return static_cast<float>(std::min(l, p));
    };
    for (uint32_t line = 0; line < lines_per_frame; ++line) {
      float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
      const float blank = estimate_blank_simple(row);
      const float pl = paired_phase_line(line);
      const float lf = phase * 0.39f + pl * roll + frame_roll_phase;
      const float line_offset =
          (0.5f + 8.8f * hum_strength) * std::sin(lf + 0.2f) +
          (0.2f + 4.6f * hum_strength) * std::sin(2.0f * lf + 1.7f);
      const float line_gain = 1.0f + (0.003f + 0.042f * hum_strength) * std::sin(lf + 0.9f);
      for (uint32_t s = 0; s < samples_per_line; ++s) {
        const float ripple_phase =
            lf + 0.0095f * static_cast<float>(s) +
            0.25f * std::sin(phase * 0.71f + 0.0032f * static_cast<float>(s));
        const float ripple = (0.12f + 3.2f * hum_strength) * std::sin(ripple_phase);
        const float interference =
            (0.03f + 1.1f * hum_strength) *
            std::sin((kPi * 0.5f) * static_cast<float>(s) + 0.63f * lf);
        const float centered = row[s] - blank;
        row[s] = Clamp(blank + centered * line_gain + line_offset + ripple + interference,
                       -80.0f,
                       140.0f);
      }
    }
  }

  if (afc_hunt_strength > 0.0f && lines_per_frame > 0 && samples_per_line > 0) {
    std::vector<float> afc_scratch(samples_per_line);
    const float phase = 0.0051f * static_cast<float>(random_seed);
    constexpr int kFieldPairDelta = 263;
    auto paired_phase_line = [&](uint32_t line) -> float {
      const int l = static_cast<int>(line);
      const int p = (l < kFieldPairDelta) ? (l + kFieldPairDelta) : (l - kFieldPairDelta);
      if (p < 0 || static_cast<uint32_t>(p) >= lines_per_frame) {
        return static_cast<float>(l);
      }
      return static_cast<float>(std::min(l, p));
    };
    for (uint32_t line = 0; line < lines_per_frame; ++line) {
      float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
      std::copy(row, row + samples_per_line, afc_scratch.begin());
      const float pl = paired_phase_line(line);
      const float line_phase = phase * 0.41f + 0.062f * pl;
      const float base_shift =
          (0.08f + 7.2f * afc_hunt_strength) *
          (std::sin(line_phase) + 0.42f * std::sin(phase * 0.19f + 0.029f * pl + 2.0f));
      float slip = 0.0f;
      if (uni01(rng) < (0.003f + 0.12f * afc_hunt_strength * afc_hunt_strength)) {
        slip = (uni01(rng) < 0.5f ? -1.0f : 1.0f) * (0.7f + 6.5f * afc_hunt_strength);
      }
      const float micro = gauss01(rng) * (0.02f + 0.75f * afc_hunt_strength);
      const float fast_decay = 7.0f + 13.0f * (1.0f - afc_hunt_strength);
      const float slow_decay = 34.0f + 76.0f * (1.0f - afc_hunt_strength);
      for (uint32_t s = 0; s < samples_per_line; ++s) {
        const float sf = static_cast<float>(s);
        const float left_edge = std::exp(-sf / slow_decay);
        const float sync_edge = std::exp(-sf / fast_decay);
        const float shift = base_shift * left_edge + (slip + micro) * sync_edge;
        float pos = sf - shift;
        pos = Clamp(pos, 0.0f, static_cast<float>(samples_per_line - 1));
        const uint32_t p0 = static_cast<uint32_t>(pos);
        const uint32_t p1 = std::min<uint32_t>(p0 + 1, samples_per_line - 1);
        const float frac = pos - static_cast<float>(p0);
        row[s] = afc_scratch[p0] + (afc_scratch[p1] - afc_scratch[p0]) * frac;
      }
    }
  }

  if (agc_pump_strength > 0.0f && lines_per_frame > 0 && samples_per_line > 0) {
    const uint32_t active_start_s = std::min<uint32_t>(
        samples_per_line - 1, std::max<uint32_t>(hsync_samples + 40U, burst_start + burst_samples + 4U));
    const uint32_t active_end_s = std::max<uint32_t>(active_start_s + 1, samples_per_line - 1);
    auto smooth_step = [](float edge0, float edge1, float x) -> float {
      if (edge1 <= edge0) {
        return x >= edge1 ? 1.0f : 0.0f;
      }
      const float t = Clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
      return t * t * (3.0f - 2.0f * t);
    };

    float bright_energy = 0.0f;
    float transient_energy = 0.0f;
    float peak_level = 0.0f;
    uint32_t n = 0;
    for (uint32_t line = 0; line < lines_per_frame; ++line) {
      const float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
      const float blank = estimate_blank_simple(row);
      for (uint32_t s = active_start_s; s + 4U < active_end_s; s += 3U) {
        const float y0 = std::max(0.0f, row[s] - blank);
        const float y1 = std::max(0.0f, row[s + 4U] - blank);
        bright_energy += std::max(0.0f, y0 - 78.0f);
        transient_energy += std::fabs(y1 - y0);
        peak_level = std::max(peak_level, std::max(y0, y1));
        ++n;
      }
    }
    if (n == 0) {
      n = 1;
    }
    const float bright_drive = Clamp(bright_energy / (static_cast<float>(n) * 26.0f), 0.0f, 1.0f);
    const float trans_drive = Clamp(transient_energy / (static_cast<float>(n) * 7.0f), 0.0f, 1.0f);
    const float peak_drive = Clamp((peak_level - 96.0f) / 26.0f, 0.0f, 1.0f);
    const float event_drive =
        Clamp(0.50f * bright_drive + 0.35f * trans_drive + 0.30f * peak_drive - 0.40f, 0.0f, 1.0f);
    const float phase = 0.0033f * static_cast<float>(random_seed);
    const float pulse = std::max(0.0f, std::sin(phase * 0.47f + 0.6f)) *
                        (0.78f + 0.22f * std::sin(phase * 0.19f + 1.4f));
    const float pump = (0.06f + 0.46f * agc_pump_strength) *
                       smooth_step(0.08f, 0.95f, event_drive) * pulse;
    if (pump > 0.0001f) {
      for (uint32_t line = 0; line < lines_per_frame; ++line) {
        float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
        const float blank = estimate_blank_simple(row);
        const float line_pump =
            pump * (0.86f + 0.14f * std::sin(phase * 0.53f + 0.024f * static_cast<float>(line)));
        const float knee = 24.0f + 14.0f * (1.0f - agc_pump_strength);
        for (uint32_t s = 0; s < samples_per_line; ++s) {
          float centered = row[s] - blank;
          if (centered > knee) {
            centered = knee + (centered - knee) * (1.0f - 0.92f * line_pump);
          } else if (centered > 0.0f) {
            centered *= (1.0f - 0.20f * line_pump * (centered / (knee + 1.0f)));
          }
          row[s] = Clamp(blank + centered, -80.0f, 140.0f);
        }
      }
    }
  }

  if ((chroma_flutter_strength > 0.0f || yc_crosstalk_strength > 0.0f) &&
      lines_per_frame > 0 && samples_per_line > 0) {
    const uint32_t active_start_s =
        std::min<uint32_t>(samples_per_line - 1, std::max<uint32_t>(hsync_samples + 40U, burst_start + burst_samples + 4U));
    const uint32_t active_end_s = std::max<uint32_t>(active_start_s + 1, samples_per_line - 1);
    std::vector<float> lc_scratch(samples_per_line);
    const float phase = 0.0049f * static_cast<float>(random_seed);
    constexpr int kFieldPairDelta = 263;
    auto paired_phase_line = [&](uint32_t line) -> float {
      const int l = static_cast<int>(line);
      const int p = (l < kFieldPairDelta) ? (l + kFieldPairDelta) : (l - kFieldPairDelta);
      if (p < 0 || static_cast<uint32_t>(p) >= lines_per_frame) {
        return static_cast<float>(l);
      }
      return static_cast<float>(std::min(l, p));
    };
    for (uint32_t line = 0; line < lines_per_frame; ++line) {
      float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
      std::copy(row, row + samples_per_line, lc_scratch.begin());
      const float pl = paired_phase_line(line);
      const float phase_jit =
          (0.22f + 2.4f * chroma_flutter_strength) * chroma_flutter_strength *
              std::sin(phase * 0.31f + 0.043f * pl) +
          gauss01(rng) * (0.65f * chroma_flutter_strength);
      const float amp_jit =
          1.0f +
          (0.06f + 1.18f * chroma_flutter_strength) * chroma_flutter_strength *
              std::sin(phase * 0.47f + 0.071f * pl + 0.9f);
      const uint32_t s_begin = std::max<uint32_t>(1U, active_start_s);
      const uint32_t s_end = std::min<uint32_t>(active_end_s, samples_per_line - 1U);
      float corr = 0.0f;
      for (uint32_t s = s_begin; s + 1 < s_end; ++s) {
        const float omega = (kPi * 0.5f) * static_cast<float>(s) + phase_jit;
        corr = 0.90f * corr + gauss01(rng) * (0.85f * chroma_flutter_strength);
        float v = lc_scratch[s];
        if (chroma_flutter_strength > 0.0f) {
          const float chroma_inject =
              ((0.26f + 4.4f * chroma_flutter_strength) * amp_jit) * std::sin(omega) + corr;
          v += chroma_inject;
        }
        if (yc_crosstalk_strength > 0.0f) {
          const float hp = lc_scratch[s] - 0.5f * (lc_scratch[s - 1] + lc_scratch[s + 1]);
          const float edge = lc_scratch[s] - lc_scratch[s - 1];
          const float gate = Clamp(std::fabs(hp) / (2.0f + 20.0f * (1.0f - yc_crosstalk_strength)),
                                   0.0f,
                                   1.0f);
          const float carrier = std::sin((kPi * 0.5f) * static_cast<float>(s) +
                                         ((line & 1U) ? kPi : 0.0f) +
                                         0.35f * phase_jit + 0.72f);
          float cross =
              (hp * (0.14f + 0.88f * yc_crosstalk_strength) +
               edge * (0.07f + 0.52f * yc_crosstalk_strength)) *
              (0.35f + 0.65f * carrier);
          cross *= (0.35f + 0.65f * gate);
          if ((line & 1U) != 0U) {
            cross = -cross;
          }
          v += cross;
        }
        row[s] = Clamp(v, -80.0f, 140.0f);
      }
    }
  }

  const float tracking_axis = Clamp(vhs_tracking, -1.0f, 1.0f);
  const float tracking_strength = std::pow(std::fabs(tracking_axis), 1.35f);
  const float wrinkle_strength = std::pow(Clamp(vhs_wrinkle, 0.0f, 1.0f), 1.45f);
  const float head_switch_strength = std::pow(Clamp(vhs_head_switch, 0.0f, 1.0f), 1.2f);
  const float dropout_strength = std::pow(Clamp(vhs_dropouts, 0.0f, 1.0f), 1.3f);
  const bool has_vhs =
      (tracking_strength > 0.0f || wrinkle_strength > 0.0f ||
       head_switch_strength > 0.0f || dropout_strength > 0.0f);

  if (has_vhs && lines_per_frame > 0 && samples_per_line > 0) {
    auto estimate_blank = [&](const float* row) -> float {
      float sum = 0.0f;
      uint32_t n = 0;

      const uint32_t pre_start =
          std::min<uint32_t>(samples_per_line, hsync_samples + 6U);
      const uint32_t pre_end = std::min<uint32_t>(
          samples_per_line, (burst_start > 2U) ? (burst_start - 2U) : burst_start);
      for (uint32_t s = pre_start; s < pre_end; ++s) {
        sum += row[s];
        ++n;
      }

      const uint32_t post_start = std::min<uint32_t>(
          samples_per_line, burst_start + burst_samples + 3U);
      const uint32_t active_gate = std::max<uint32_t>(
          hsync_samples + 40U, burst_start + burst_samples + 4U);
      const uint32_t post_end = std::min<uint32_t>(
          samples_per_line,
          (active_gate > 2U) ? (active_gate - 2U) : active_gate);
      for (uint32_t s = post_start; s < post_end; ++s) {
        sum += row[s];
        ++n;
      }

      return n > 0 ? (sum / static_cast<float>(n)) : 0.0f;
    };

    auto shift_line = [&](float* row, std::vector<float>* scratch, float shift) {
      if (std::fabs(shift) < 0.02f) {
        return;
      }
      std::copy(row, row + samples_per_line, scratch->begin());
      for (uint32_t s = 0; s < samples_per_line; ++s) {
        float pos = static_cast<float>(s) - shift;
        pos = Clamp(pos, 0.0f, static_cast<float>(samples_per_line - 1));
        const uint32_t p0 = static_cast<uint32_t>(pos);
        const uint32_t p1 = std::min<uint32_t>(p0 + 1, samples_per_line - 1);
        const float frac = pos - static_cast<float>(p0);
        row[s] = (*scratch)[p0] + ((*scratch)[p1] - (*scratch)[p0]) * frac;
      }
    };

    std::vector<float> line_shift(lines_per_frame, 0.0f);
    std::vector<float> scratch(samples_per_line, 0.0f);
    const uint32_t active_start_s = std::min<uint32_t>(samples_per_line - 1, hsync_samples + 40U);
    const uint32_t active_end_s = std::max<uint32_t>(active_start_s + 1, samples_per_line - 1);
    const uint32_t burst_end = std::min<uint32_t>(samples_per_line, burst_start + burst_samples);

    if (tracking_strength > 0.0f) {
      std::mt19937 tr_rng(static_cast<uint32_t>(0x54A319D1u ^ random_seed));
      std::uniform_real_distribution<float> tr_uni(0.0f, 1.0f);
      std::normal_distribution<float> tr_gauss(0.0f, 1.0f);
      const bool anchor_bottom = (tracking_axis > 0.0f);
      const int band_lines = 2 + static_cast<int>(std::lrintf(136.0f * std::pow(tracking_strength, 1.08f)));
      const int band_start = anchor_bottom ? std::max(0, static_cast<int>(lines_per_frame) - band_lines) : 0;
      const uint32_t hsync_span = std::min<uint32_t>(samples_per_line, hsync_samples + 24U);
      const float dir_sign = anchor_bottom ? -1.0f : 1.0f;
      const float wave_phase = tr_uni(tr_rng) * kTwoPi;
      const float scallop_phase = tr_uni(tr_rng) * kTwoPi;
      const float scallop_amp = 0.8f + 10.0f * tracking_strength;
      const int comb_period = 16 + static_cast<int>(std::lrintf(18.0f * (1.0f - tracking_strength)));
      constexpr int kFieldPairDelta = 263;
      auto paired_line = [&](int line) -> int {
        return (line < kFieldPairDelta) ? (line + kFieldPairDelta) : (line - kFieldPairDelta);
      };
      auto apply_tracking_line = [&](int line, float w, float scallop, std::mt19937* line_rng) {
        if (line < 0 || static_cast<uint32_t>(line) >= lines_per_frame) {
          return;
        }
        float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
        const float blank = estimate_blank(row);

        for (uint32_t s = active_start_s; s < active_end_s; ++s) {
          row[s] = Clamp(row[s] + tr_gauss(*line_rng) * (0.35f + 7.0f * tracking_strength * w), -80.0f, 140.0f);
        }
        for (uint32_t s = 0; s < hsync_span; ++s) {
          row[s] = Clamp(row[s] + tr_gauss(*line_rng) * (1.2f + 14.0f * tracking_strength * w), -80.0f, 140.0f);
        }

        const int dash_count = std::max(1, static_cast<int>(std::lrintf((1.0f + 28.0f * tracking_strength) * w)));
        for (int d = 0; d < dash_count; ++d) {
          uint32_t ds = active_start_s + static_cast<uint32_t>(std::lrintf(
              tr_uni(*line_rng) * static_cast<float>(std::max<uint32_t>(1, active_end_s - active_start_s - 1))));
          if (d < 2 || tr_uni(*line_rng) < 0.45f) {
            const int comb_slot = d % std::max(1, static_cast<int>((active_end_s - active_start_s) / std::max(6, comb_period)));
            ds = std::min<uint32_t>(
                active_end_s - 1,
                active_start_s + static_cast<uint32_t>(comb_slot * comb_period) +
                    static_cast<uint32_t>(std::lrintf((6.0f + 10.0f * tracking_strength) * (0.5f + 0.5f * scallop))));
          }
          const uint32_t dl = 4U + static_cast<uint32_t>(std::lrintf((4.0f + 45.0f * tracking_strength) * tr_uni(*line_rng)));
          const uint32_t de = std::min<uint32_t>(active_end_s, ds + dl);
          for (uint32_t s = ds; s < de; ++s) {
            row[s] = Clamp(blank + 94.0f + tr_gauss(*line_rng) * (2.0f + 6.0f * tracking_strength), -80.0f, 140.0f);
          }
        }

        if (burst_start < burst_end && tr_uni(*line_rng) < (0.1f + 0.7f * tracking_strength * w)) {
          for (uint32_t s = burst_start; s < burst_end; ++s) {
            row[s] = Clamp(blank + tr_gauss(*line_rng) * (1.0f + 5.0f * tracking_strength), -80.0f, 140.0f);
          }
        }
      };

      for (int y = 0; y < band_lines; ++y) {
        const int line = band_start + y;
        if (line < 0 || static_cast<uint32_t>(line) >= lines_per_frame) {
          continue;
        }
        const float edge =
            static_cast<float>(anchor_bottom ? (band_lines - 1 - y) : y) / std::max(1, band_lines - 1);
        const float w = std::pow(1.0f - edge, 0.75f);
        const float scallop = std::sin(scallop_phase + 0.35f * static_cast<float>(y));
        const float line_shift_delta =
            dir_sign * (1.0f + 26.0f * tracking_strength + scallop_amp * scallop * w) *
            (0.35f + 0.65f * w) *
            (0.7f + 0.3f * std::sin(wave_phase + 0.11f * static_cast<float>(line)));
        line_shift[static_cast<size_t>(line)] += line_shift_delta;
        const int pair = paired_line(line);
        if (pair >= 0 && static_cast<uint32_t>(pair) < lines_per_frame && pair != line) {
          line_shift[static_cast<size_t>(pair)] += line_shift_delta;
        }

        std::mt19937 line_rng = tr_rng;
        apply_tracking_line(line, w, scallop, &line_rng);
        if (pair >= 0 && static_cast<uint32_t>(pair) < lines_per_frame && pair != line) {
          std::mt19937 pair_rng = tr_rng;
          apply_tracking_line(pair, w, scallop, &pair_rng);
        }
        tr_rng = line_rng;
      }
    }

    if (head_switch_strength > 0.0f) {
      std::mt19937 hs_rng(static_cast<uint32_t>(0x1C6E9A5Bu ^ random_seed));
      std::uniform_real_distribution<float> hs_uni(0.0f, 1.0f);
      std::normal_distribution<float> hs_gauss(0.0f, 1.0f);
      const int jitter = static_cast<int>(std::lrintf((hs_uni(hs_rng) - 0.5f) * (2.0f + 10.0f * head_switch_strength)));
      const int switch_line = std::max(0, static_cast<int>(lines_per_frame) - 8 + jitter);
      const int hs_band_lines = 2 + static_cast<int>(std::lrintf(28.0f * head_switch_strength));
      const float base_shift = (3.0f + 36.0f * head_switch_strength) * (hs_uni(hs_rng) < 0.5f ? -1.0f : 1.0f);
      constexpr int kFieldPairDelta = 263;
      auto paired_line = [&](int line) -> int {
        return (line < kFieldPairDelta) ? (line + kFieldPairDelta) : (line - kFieldPairDelta);
      };
      auto apply_head_switch_line = [&](int line, float w, bool add_dashes, std::mt19937* line_rng) {
        if (line < 0 || static_cast<uint32_t>(line) >= lines_per_frame) {
          return;
        }
        float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
        std::copy(row, row + samples_per_line, scratch.begin());
        const float blank = estimate_blank(row);
        const uint32_t noisy_end =
            std::min<uint32_t>(samples_per_line, active_start_s + static_cast<uint32_t>(std::lrintf(260.0f + 420.0f * head_switch_strength)));
        const uint32_t noisy_soft_px = 2U + static_cast<uint32_t>(std::lrintf(8.0f * head_switch_strength));
        for (uint32_t s = 0; s < noisy_end; ++s) {
          const float noisy_v = blank + hs_gauss(*line_rng) * (2.5f + 18.0f * head_switch_strength * w) +
                                (hs_uni(*line_rng) - 0.5f) * 14.0f * head_switch_strength * w;
          float alpha = 1.0f;
          if (noisy_soft_px > 0U && noisy_end > 0U) {
            const uint32_t left = s;
            const uint32_t right = noisy_end - 1U - s;
            if (left < noisy_soft_px) {
              alpha *= static_cast<float>(left + 1U) / static_cast<float>(noisy_soft_px + 1U);
            }
            if (right < noisy_soft_px) {
              alpha *= static_cast<float>(right + 1U) / static_cast<float>(noisy_soft_px + 1U);
            }
          }
          row[s] = Clamp((1.0f - alpha) * scratch[s] + alpha * noisy_v, -80.0f, 140.0f);
        }

        if (add_dashes) {
          for (int k = 0; k < 14 + static_cast<int>(std::lrintf(28.0f * head_switch_strength)); ++k) {
            const uint32_t ds = active_start_s + static_cast<uint32_t>(std::lrintf(
                hs_uni(*line_rng) * static_cast<float>(std::max<uint32_t>(1, active_end_s - active_start_s - 1))));
            const uint32_t dl = 8U + static_cast<uint32_t>(std::lrintf(28.0f * hs_uni(*line_rng)));
            const uint32_t de = std::min<uint32_t>(active_end_s, ds + dl);
            const uint32_t dash_soft_px = 2U + static_cast<uint32_t>(std::lrintf(6.0f * head_switch_strength));
            for (uint32_t s = ds; s < de; ++s) {
              const float dash_v = blank + 96.0f + hs_gauss(*line_rng) * (1.5f + 6.0f * head_switch_strength);
              float alpha = 1.0f;
              if (dash_soft_px > 0U && de > ds) {
                const uint32_t left = s - ds;
                const uint32_t right = de - 1U - s;
                if (left < dash_soft_px) {
                  alpha *= static_cast<float>(left + 1U) / static_cast<float>(dash_soft_px + 1U);
                }
                if (right < dash_soft_px) {
                  alpha *= static_cast<float>(right + 1U) / static_cast<float>(dash_soft_px + 1U);
                }
              }
              row[s] = Clamp((1.0f - alpha) * row[s] + alpha * dash_v, -80.0f, 140.0f);
            }
          }
        }
      };

      for (int y = 0; y < hs_band_lines; ++y) {
        const int line = switch_line + y;
        if (line < 0 || static_cast<uint32_t>(line) >= lines_per_frame) {
          continue;
        }
        const float w = 1.0f - (static_cast<float>(y) / std::max(1, hs_band_lines - 1));
        line_shift[static_cast<size_t>(line)] += base_shift * (0.35f + 0.65f * w);
        const int pair = paired_line(line);
        if (pair >= 0 && static_cast<uint32_t>(pair) < lines_per_frame && pair != line) {
          line_shift[static_cast<size_t>(pair)] += base_shift * (0.35f + 0.65f * w);
        }

        std::mt19937 line_rng = hs_rng;
        apply_head_switch_line(line, w, y == 0, &line_rng);
        if (pair >= 0 && static_cast<uint32_t>(pair) < lines_per_frame && pair != line) {
          std::mt19937 pair_rng = hs_rng;
          apply_head_switch_line(pair, w, y == 0, &pair_rng);
        }
        hs_rng = line_rng;
      }
    }

    if (wrinkle_strength > 0.0f) {
      const uint32_t travel_frames =
          120U + static_cast<uint32_t>(std::lrintf(220.0f * (1.0f - wrinkle_strength)));
      const uint32_t gap_frames =
          18U + static_cast<uint32_t>(std::lrintf(55.0f * (1.0f - wrinkle_strength)));
      const uint32_t cycle_frames = std::max<uint32_t>(1U, travel_frames + gap_frames);
      const uint32_t cycle_pos = random_seed % cycle_frames;
      if (cycle_pos < travel_frames) {
        const uint32_t epoch = random_seed / cycle_frames;
        std::mt19937 shape_rng(static_cast<uint32_t>(0x7E319F2Du ^ epoch));
        std::mt19937 frame_rng(static_cast<uint32_t>(0x7E319F2Du ^ random_seed ^ 0x9E3779B9u));
        std::uniform_real_distribution<float> uni(0.0f, 1.0f);
        std::normal_distribution<float> gauss(0.0f, 1.0f);

        const bool moving_down = ((epoch & 1U) == 0U);
        const int lead_dir = moving_down ? 1 : -1;
        const int drop_half = 2 + static_cast<int>(std::lrintf(2.0f + 6.0f * wrinkle_strength));
        const int lead_lines = 3 + static_cast<int>(std::lrintf(6.0f + 20.0f * wrinkle_strength));
        const float wrinkle_base_shift =
            (2.0f + 12.0f * wrinkle_strength) * (uni(shape_rng) < 0.5f ? -1.0f : 1.0f);
        const float phase0 = uni(shape_rng) * kTwoPi;
        const float t = static_cast<float>(cycle_pos) / std::max(1U, travel_frames - 1U);
        const float center = moving_down
                                 ? (-static_cast<float>(drop_half) +
                                    t * (static_cast<float>(lines_per_frame) + 2.0f * drop_half))
                                 : (static_cast<float>(lines_per_frame - 1 + drop_half) -
                                    t * (static_cast<float>(lines_per_frame) + 2.0f * drop_half));
        constexpr int kFieldPairDelta = 263;
        auto paired_line = [&](int line) -> int {
          return (line < kFieldPairDelta) ? (line + kFieldPairDelta) : (line - kFieldPairDelta);
        };
        auto apply_wrinkle_band_line = [&](int line, float core_w, float halo_w, float edge_jitter) {
          if (line < 0 || static_cast<uint32_t>(line) >= lines_per_frame) {
            return;
          }
          float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
          const float blank = estimate_blank(row);
          std::copy(row, row + samples_per_line, scratch.begin());
          const float effect_w = std::max(core_w, halo_w);
          if (effect_w <= 0.0f) {
            return;
          }

          if (core_w > 0.0f) {
            line_shift[static_cast<size_t>(line)] += wrinkle_base_shift * (0.45f + 0.55f * core_w);
          } else if (halo_w > 0.0f) {
            line_shift[static_cast<size_t>(line)] += wrinkle_base_shift * 0.15f * halo_w;
          }

          float corr = 0.0f;
          const float corr_drive = 0.35f + 3.2f * wrinkle_strength * (0.25f + 0.75f * effect_w);
          for (uint32_t s = active_start_s; s < active_end_s; ++s) {
            const float scallop =
                std::sin(phase0 + 0.21f * static_cast<float>(line) +
                         (static_cast<float>(s - active_start_s) * (kTwoPi / 64.0f)) +
                         0.65f * edge_jitter);
            const float edge_mod = 0.65f + 0.35f * scallop;
            corr = 0.965f * corr + gauss(frame_rng) * corr_drive;
            const float texture =
                0.82f * corr + gauss(frame_rng) * (0.24f + 1.6f * wrinkle_strength * effect_w);

            float v = scratch[s];
            if (core_w > 0.0f) {
              v = blank + texture + scallop * (0.2f + 2.0f * wrinkle_strength * core_w);
              if (uni(frame_rng) < (0.02f + 0.22f * wrinkle_strength * core_w)) {
                v = blank + 94.0f + gauss(frame_rng) * (1.2f + 4.8f * wrinkle_strength);
              }
              if (uni(frame_rng) < (0.01f + 0.09f * wrinkle_strength * core_w)) {
                v = blank - (6.0f + 20.0f * wrinkle_strength * core_w) +
                    gauss(frame_rng) * (0.9f + 2.4f * wrinkle_strength);
              }
            } else if (halo_w > 0.0f) {
              const float alpha =
                  Clamp(halo_w * (0.30f + 0.42f * edge_mod + 0.36f * std::fabs(edge_jitter)),
                        0.0f,
                        1.0f);
              const float noisy = blank + texture;
              v = (1.0f - alpha) * scratch[s] + alpha * noisy;
            }
            row[s] = Clamp(v, -80.0f, 140.0f);
          }

          if (halo_w > 0.0f && core_w <= 0.01f && active_end_s > active_start_s + 8U) {
            const int burst_events = std::max(
                1,
                static_cast<int>(std::lrintf((1.0f + 12.0f * wrinkle_strength * halo_w) *
                                             (0.45f + 0.55f * uni(frame_rng)))));
            const uint32_t span = active_end_s - active_start_s;
            for (int b = 0; b < burst_events; ++b) {
              const uint32_t seg_start = active_start_s + static_cast<uint32_t>(std::lrintf(
                                                         uni(frame_rng) * static_cast<float>(span - 1)));
              const uint32_t seg_len = 4U + static_cast<uint32_t>(std::lrintf(
                                              (8.0f + 46.0f * uni(frame_rng)) *
                                              (0.3f + 0.7f * wrinkle_strength * halo_w)));
              const uint32_t seg_end = std::min<uint32_t>(active_end_s, seg_start + seg_len);
              const bool white_hit = (uni(frame_rng) < (0.38f + 0.35f * wrinkle_strength * halo_w));
              float walk = 0.0f;
              for (uint32_t s = seg_start; s < seg_end; ++s) {
                walk = 0.90f * walk + gauss(frame_rng) * (0.35f + 2.2f * wrinkle_strength * halo_w);
                if (white_hit) {
                  row[s] = Clamp(blank + 88.0f + walk + gauss(frame_rng) * (0.9f + 4.2f * wrinkle_strength),
                                 -80.0f,
                                 140.0f);
                } else {
                  row[s] = Clamp(blank - (2.0f + 16.0f * wrinkle_strength * halo_w) + walk,
                                 -80.0f,
                                 140.0f);
                }
              }
            }
          }

          if (core_w > 0.05f && active_end_s > active_start_s + 2U) {
            std::copy(row + active_start_s, row + active_end_s, scratch.begin() + active_start_s);
            for (uint32_t s = active_start_s + 1; s + 1 < active_end_s; ++s) {
              const float lp = 0.22f * scratch[s - 1] + 0.56f * scratch[s] + 0.22f * scratch[s + 1];
              row[s] = Clamp(lp + gauss(frame_rng) * (0.15f + 0.55f * wrinkle_strength * core_w),
                             -80.0f,
                             140.0f);
            }
          }

          if (burst_start < burst_end) {
            const float bw = effect_w;
            if (bw > 0.0f) {
              for (uint32_t s = burst_start; s < burst_end; ++s) {
                row[s] = Clamp(blank + gauss(frame_rng) * (1.1f + 7.2f * wrinkle_strength * bw),
                               -80.0f,
                               140.0f);
              }
            }
          }
        };

        const int halo_half = drop_half + 2 + static_cast<int>(std::lrintf(8.0f * wrinkle_strength));
        for (int line = static_cast<int>(std::floor(center)) - halo_half;
             line <= static_cast<int>(std::ceil(center)) + halo_half;
             ++line) {
          const float edge_wobble =
              (0.35f + 1.7f * wrinkle_strength) *
              (0.55f * std::sin(phase0 + 0.37f * static_cast<float>(line)) +
               0.45f * gauss(frame_rng));
          const float d = std::fabs((static_cast<float>(line) + edge_wobble) - center);
          const float core_w = Clamp(1.0f - d / std::max(1, drop_half), 0.0f, 1.0f);
          float halo_w = 0.0f;
          if (core_w <= 0.0f) {
            halo_w = Clamp(1.0f - (d - static_cast<float>(drop_half)) /
                                     std::max(1, halo_half - drop_half),
                           0.0f,
                           1.0f);
          }
          if (core_w <= 0.0f && halo_w <= 0.0f) {
            continue;
          }
          apply_wrinkle_band_line(line, core_w, halo_w, edge_wobble);
          const int pair = paired_line(line);
          if (pair != line) {
            apply_wrinkle_band_line(pair, core_w * 0.95f, halo_w * 0.95f, edge_wobble * 0.95f);
          }
          if (core_w > 0.0f) {
            const float fringe_w = Clamp(0.15f + 0.58f * core_w, 0.0f, 1.0f);
            apply_wrinkle_band_line(line - 1, 0.0f, fringe_w, edge_wobble * 0.6f);
            apply_wrinkle_band_line(line + 1, 0.0f, fringe_w, edge_wobble * -0.6f);
            if (pair != line) {
              apply_wrinkle_band_line(pair - 1, 0.0f, fringe_w * 0.92f, edge_wobble * 0.5f);
              apply_wrinkle_band_line(pair + 1, 0.0f, fringe_w * 0.92f, edge_wobble * -0.5f);
            }
          }
        }

        for (int i = 0; i < lead_lines; ++i) {
          const int line = static_cast<int>(std::lrintf(center)) + lead_dir * (drop_half + 1 + i);
          if (line < 0 || static_cast<uint32_t>(line) >= lines_per_frame) {
            continue;
          }
          const float w = 1.0f - (static_cast<float>(i) / std::max(1, lead_lines - 1));
          float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
          const float blank = estimate_blank(row);
          std::copy(row, row + samples_per_line, scratch.begin());
          line_shift[static_cast<size_t>(line)] += wrinkle_base_shift * 0.18f * w;

          const int smear_shift = 1 + static_cast<int>(std::lrintf(6.0f * wrinkle_strength * w));
          const float active_span_f = static_cast<float>(std::max<uint32_t>(1, active_end_s - active_start_s));
          const float line_phase = phase0 + 0.91f * static_cast<float>(i) + 0.08f * static_cast<float>(cycle_pos);
          const float left_edge =
              Clamp(0.01f + 0.10f * uni(frame_rng) +
                        0.05f * std::sin(line_phase + 0.7f * gauss(frame_rng)),
                    0.0f,
                    0.45f);
          const float right_edge =
              Clamp(0.99f - 0.10f * uni(frame_rng) +
                        0.05f * std::sin(line_phase + 2.1f + 0.7f * gauss(frame_rng)),
                    0.55f,
                    1.0f);
          float corr = 0.0f;
          float rag = 0.0f;
          for (uint32_t s = active_start_s; s < active_end_s; ++s) {
            const int p0 = std::max<int>(0, static_cast<int>(s) - smear_shift);
            const int p1 = std::max<int>(0, p0 - smear_shift);
            const float base = scratch[s];
            const float smear = 0.58f * base + 0.30f * scratch[static_cast<size_t>(p0)] +
                                0.12f * scratch[static_cast<size_t>(p1)];
            const float x_norm =
                static_cast<float>(s - active_start_s) / active_span_f;
            const float edge_dist =
                std::min(x_norm - left_edge, right_edge - x_norm);
            rag = 0.95f * rag + gauss(frame_rng) * (0.08f + 0.30f * wrinkle_strength * w);
            const float gate_core = Clamp((edge_dist + rag) * (14.0f + 16.0f * wrinkle_strength * w), 0.0f, 1.0f);
            float gate = gate_core;
            if (uni(frame_rng) < (0.02f + 0.24f * wrinkle_strength * w)) {
              gate *= (0.08f + 0.45f * uni(frame_rng));
            }
            float v = blank + (smear - blank) * (0.92f - 0.36f * wrinkle_strength * w);
            const float sub =
                std::sin(phase0 + 0.52f * static_cast<float>(i) + static_cast<float>(s) * (kPi * 0.5f));
            corr = 0.97f * corr + gauss(frame_rng) * (0.28f + 1.2f * wrinkle_strength * w);
            v += sub * (0.24f + 2.0f * wrinkle_strength * w) + corr;
            if (gate < 0.999f) {
              const float bleed = blank + gauss(frame_rng) * (0.16f + 1.6f * wrinkle_strength * w * (1.0f - gate));
              v = gate * v + (1.0f - gate) * bleed;
            }
            row[s] = Clamp(v, -80.0f, 140.0f);
          }
          if (burst_start < burst_end) {
            for (uint32_t s = burst_start; s < burst_end; ++s) {
              row[s] = Clamp(blank + gauss(frame_rng) * (0.9f + 6.5f * wrinkle_strength * w), -80.0f, 140.0f);
            }
          }
          const int pair = paired_line(line);
          if (pair >= 0 && static_cast<uint32_t>(pair) < lines_per_frame) {
            float* prow = frame_ire->data() + static_cast<size_t>(pair) * samples_per_line;
            const float pblank = estimate_blank(prow);
            std::copy(prow, prow + samples_per_line, scratch.begin());
            line_shift[static_cast<size_t>(pair)] += wrinkle_base_shift * 0.16f * w;
            const int pair_smear_shift = 1 + static_cast<int>(std::lrintf(6.0f * wrinkle_strength * w));
            const float pair_left_edge =
                Clamp(0.01f + 0.11f * uni(frame_rng) +
                          0.05f * std::sin(line_phase + 1.2f + 0.7f * gauss(frame_rng)),
                      0.0f,
                      0.45f);
            const float pair_right_edge =
                Clamp(0.99f - 0.11f * uni(frame_rng) +
                          0.05f * std::sin(line_phase + 2.8f + 0.7f * gauss(frame_rng)),
                      0.55f,
                      1.0f);
            float pair_corr = 0.0f;
            float pair_rag = 0.0f;
            for (uint32_t s = active_start_s; s < active_end_s; ++s) {
              const int pp0 = std::max<int>(0, static_cast<int>(s) - pair_smear_shift);
              const int pp1 = std::max<int>(0, pp0 - pair_smear_shift);
              const float base = scratch[s];
              const float smear = 0.58f * base + 0.30f * scratch[static_cast<size_t>(pp0)] +
                                  0.12f * scratch[static_cast<size_t>(pp1)];
              const float x_norm =
                  static_cast<float>(s - active_start_s) / active_span_f;
              const float edge_dist =
                  std::min(x_norm - pair_left_edge, pair_right_edge - x_norm);
              pair_rag = 0.95f * pair_rag + gauss(frame_rng) * (0.08f + 0.30f * wrinkle_strength * w);
              const float gate_core =
                  Clamp((edge_dist + pair_rag) * (14.0f + 16.0f * wrinkle_strength * w), 0.0f, 1.0f);
              float gate = gate_core;
              if (uni(frame_rng) < (0.02f + 0.24f * wrinkle_strength * w)) {
                gate *= (0.08f + 0.45f * uni(frame_rng));
              }
              float v = pblank + (smear - pblank) * (0.92f - 0.36f * wrinkle_strength * w);
              const float sub =
                  std::sin(phase0 + 0.52f * static_cast<float>(i) + static_cast<float>(s) * (kPi * 0.5f));
              pair_corr = 0.97f * pair_corr + gauss(frame_rng) * (0.28f + 1.2f * wrinkle_strength * w);
              v += sub * (0.24f + 2.0f * wrinkle_strength * w) + pair_corr;
              if (gate < 0.999f) {
                const float bleed =
                    pblank + gauss(frame_rng) * (0.16f + 1.6f * wrinkle_strength * w * (1.0f - gate));
                v = gate * v + (1.0f - gate) * bleed;
              }
              prow[s] = Clamp(v, -80.0f, 140.0f);
            }
            if (burst_start < burst_end) {
              for (uint32_t s = burst_start; s < burst_end; ++s) {
                prow[s] = Clamp(pblank + gauss(frame_rng) * (0.9f + 6.0f * wrinkle_strength * w), -80.0f, 140.0f);
              }
            }
          }
        }
      }
    }

    for (uint32_t line = 0; line < lines_per_frame; ++line) {
      float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
      shift_line(row, &scratch, line_shift[line]);
    }

    if (dropout_strength > 0.0f) {
      const int events =
          1 + static_cast<int>(std::lrintf((4.0f + 52.0f * dropout_strength) * (0.4f + 0.6f * uni01(rng))));
      for (int e = 0; e < events; ++e) {
        const float ly_bias = std::pow(uni01(rng), 0.7f);
        const uint32_t line = static_cast<uint32_t>(std::lrintf(
            ly_bias * static_cast<float>(std::max<uint32_t>(1, lines_per_frame - 1))));
        float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;
        const float blank = estimate_blank(row);
        const uint32_t seg_start = active_start_s + static_cast<uint32_t>(std::lrintf(
            uni01(rng) * static_cast<float>(std::max<uint32_t>(1, active_end_s - active_start_s - 1))));
        const uint32_t seg_len = 8U + static_cast<uint32_t>(std::lrintf(
            (20.0f + 180.0f * uni01(rng)) * dropout_strength));
        const uint32_t seg_end = std::min<uint32_t>(active_end_s, seg_start + seg_len);
        const bool white_streak = (uni01(rng) < (0.58f + 0.3f * dropout_strength));

        if (white_streak) {
          for (uint32_t s = seg_start; s < seg_end; ++s) {
            row[s] = Clamp(blank + 96.0f + gauss01(rng) * (2.0f + 8.0f * dropout_strength), -80.0f, 140.0f);
          }
          const uint32_t smear = 2U + static_cast<uint32_t>(std::lrintf(12.0f * dropout_strength));
          for (uint32_t s = seg_end; s < std::min<uint32_t>(active_end_s, seg_end + smear); ++s) {
            const float t = static_cast<float>(s - seg_end + 1) / static_cast<float>(smear + 1U);
            row[s] = Clamp((1.0f - (0.7f * (1.0f - t))) * row[s] + (0.7f * (1.0f - t)) * (blank + 88.0f),
                           -80.0f,
                           140.0f);
          }
        } else {
          std::copy(row, row + samples_per_line, scratch.begin());
          const int smear_shift = 1 + static_cast<int>(std::lrintf(10.0f * dropout_strength));
          for (uint32_t s = seg_start; s < seg_end; ++s) {
            const int p0 = std::max<int>(0, static_cast<int>(s) - smear_shift);
            const int p1 = std::max<int>(0, p0 - smear_shift);
            const float v = 0.58f * scratch[static_cast<size_t>(s)] +
                            0.28f * scratch[static_cast<size_t>(p0)] +
                            0.14f * scratch[static_cast<size_t>(p1)];
            row[s] = Clamp(blank + (v - blank) * (0.55f - 0.25f * dropout_strength) +
                               gauss01(rng) * (1.0f + 6.0f * dropout_strength),
                           -80.0f,
                           140.0f);
          }
          if (burst_start < burst_end) {
            for (uint32_t s = burst_start; s < burst_end; ++s) {
              row[s] = Clamp(blank + gauss01(rng) * (1.5f + 8.0f * dropout_strength), -80.0f, 140.0f);
            }
          }
        }
      }
    }
  }

  if (h_lock_noise_ire > 0.0f || v_hold_noise_ire > 0.0f || burst_lock_noise_ire > 0.0f) {
    std::normal_distribution<float> hsync_noise(0.0f, std::max(0.0f, h_lock_noise_ire));
    std::normal_distribution<float> vsync_noise(0.0f, std::max(0.0f, v_hold_noise_ire));
    std::normal_distribution<float> burst_noise(0.0f, std::max(0.0f, burst_lock_noise_ire));

    const uint32_t top_vsync_end = std::min<uint32_t>(lines_per_frame, 24);
    const uint32_t bot_vsync_start = (lines_per_frame > 263) ? 262U : lines_per_frame;
    const uint32_t bot_vsync_end = std::min<uint32_t>(lines_per_frame, bot_vsync_start + 24U);

    const uint32_t hsync_span = std::min<uint32_t>(samples_per_line, hsync_samples + 10U);
    const uint32_t burst_end = std::min<uint32_t>(samples_per_line, burst_start + burst_samples);

    for (uint32_t line = 0; line < lines_per_frame; ++line) {
      float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;

      if (h_lock_noise_ire > 0.0f) {
        for (uint32_t s = 0; s < hsync_span; ++s) {
          row[s] = Clamp(row[s] + hsync_noise(rng), -80.0f, 140.0f);
        }
      }

      if (v_hold_noise_ire > 0.0f &&
          ((line < top_vsync_end) || (line >= bot_vsync_start && line < bot_vsync_end))) {
        const uint32_t span = std::min<uint32_t>(samples_per_line, samples_per_line / 2U);
        for (uint32_t s = 0; s < span; ++s) {
          row[s] = Clamp(row[s] + vsync_noise(rng), -80.0f, 140.0f);
        }
      }

      if (burst_lock_noise_ire > 0.0f && burst_start < burst_end) {
        for (uint32_t s = burst_start; s < burst_end; ++s) {
          row[s] = Clamp(row[s] + burst_noise(rng), -80.0f, 140.0f);
        }
      }
    }
  }

  if (noise_stddev_ire > 0.0f && lines_per_frame > 0 && samples_per_line > 0) {
    std::vector<float> noise_line(samples_per_line);
    std::vector<float> raw_noise_line(samples_per_line);
    // At low noise levels, keep the existing softer bandwidth-limited feel.
    // As noise rises toward full "snow", widen the effective bandwidth and
    // blend in a little unfiltered detail so static doesn't look over-smoothed.
    constexpr float kCompositeSampleRateHz = 315000000.0f / 22.0f;
    const float noise_norm = Clamp(noise_stddev_ire / 10.0f, 0.0f, 1.0f);
    const float kNoiseBandwidthLoHz = 800000.0f;
    const float kNoiseBandwidthHiHz = 2500000.0f;
    const float kNoiseBandwidthHz =
        kNoiseBandwidthLoHz +
        (kNoiseBandwidthHiHz - kNoiseBandwidthLoHz) * std::pow(noise_norm, 0.85f);
    const float nf_alpha = 1.0f - std::exp(
        -kTwoPi * kNoiseBandwidthHz / kCompositeSampleRateHz);
    // RMS compensation for the bidirectional one-pole (two-pole) filter.
    // Output variance of unit white noise through two cascaded one-pole
    // filters with coefficient a is: a(2-2a+a²)/(2-a)³.
    const float a = nf_alpha;
    const float denom = (2.0f - a) * (2.0f - a) * (2.0f - a);
    const float two_pole_var = a * (2.0f - 2.0f * a + a * a) / denom;
    const float compensate = 1.0f / std::max(0.01f, std::sqrt(two_pole_var));
    const float detail_mix = 0.55f * std::pow(noise_norm, 1.25f);
    const float effective_noise_color =
        Clamp(noise_color * (1.0f - 0.85f * std::pow(noise_norm, 1.1f)), 0.0f, 1.0f);
    const float line_dc_reject = 0.88f * std::pow(noise_norm, 1.05f);

    // Paul Kellet pink noise state (persists across lines).
    float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f;

    for (uint32_t line = 0; line < lines_per_frame; ++line) {
      float* row = frame_ire->data() + static_cast<size_t>(line) * samples_per_line;

      // Generate spectrally-shaped base noise for this line.
      for (uint32_t s = 0; s < samples_per_line; ++s) {
        const float white = noise(rng);
        if (effective_noise_color > 0.001f) {
          b0 = 0.99765f * b0 + white * 0.0990460f;
          b1 = 0.96300f * b1 + white * 0.2965164f;
          b2 = 0.57000f * b2 + white * 1.0526913f;
          const float pink = (b0 + b1 + b2 + white * 0.1848f) * 0.22f;
          raw_noise_line[s] =
              white * (1.0f - effective_noise_color) + pink * effective_noise_color;
        } else {
          raw_noise_line[s] = white;
        }
        noise_line[s] = raw_noise_line[s];
      }

      // Bidirectional one-pole lowpass (forward then backward) gives
      // symmetric grain and a second-order rolloff.
      float state = noise_line[0];
      for (uint32_t s = 1; s < samples_per_line; ++s) {
        state += nf_alpha * (noise_line[s] - state);
        noise_line[s] = state;
      }
      state = noise_line[samples_per_line - 1];
      for (int s = static_cast<int>(samples_per_line) - 2; s >= 0; --s) {
        state += nf_alpha * (noise_line[s] - state);
        noise_line[s] = state;
      }

      float line_sum = 0.0f;
      float line_sq = 0.0f;
      for (uint32_t s = 0; s < samples_per_line; ++s) {
        const float filtered = noise_line[s] * compensate;
        raw_noise_line[s] =
            filtered * (1.0f - detail_mix) + raw_noise_line[s] * detail_mix;
        line_sum += raw_noise_line[s];
        line_sq += raw_noise_line[s] * raw_noise_line[s];
      }
      const float line_mean = line_sum / static_cast<float>(samples_per_line);
      float centered_sq = 0.0f;
      for (uint32_t s = 0; s < samples_per_line; ++s) {
        noise_line[s] = raw_noise_line[s] - line_mean * line_dc_reject;
        centered_sq += noise_line[s] * noise_line[s];
      }
      float renorm = 1.0f;
      if (centered_sq > 1e-6f && line_sq > 1e-6f) {
        renorm = std::sqrt(line_sq / centered_sq);
      }
      renorm = Clamp(renorm, 0.7f, 2.2f);
      for (uint32_t s = 0; s < samples_per_line; ++s) {
        row[s] = Clamp(row[s] + noise_line[s] * renorm, -60.0f, 140.0f);
      }
    }
  }
}

// --- NtscEffects class methods ---

NtscEffects::NtscEffects(const NtscSignalConfig& config) : config_(config) {}

void NtscEffects::Apply(std::vector<float>* frame_ire) const {
  ApplyCompositeEffectsInternal(frame_ire, controls_, config_);
}

bool NtscEffects::HasActiveEffects() const {
  const auto& c = controls_;
  return c.noise_stddev_ire > 0.0f || c.multipath_gain != 0.0f ||
         c.multipath_ensemble > 0.0f || c.line_time_jitter_samples > 0.0f ||
         c.afc_hunt > 0.0f || c.rf_drift > 0.0f || c.am_nonlinearity > 0.0f ||
         c.impulse_noise > 0.0f || c.agc_pump > 0.0f || c.hum > 0.0f ||
         c.chroma_flutter > 0.0f || c.yc_crosstalk > 0.0f ||
         c.h_sync_noise_ire > 0.0f || c.v_sync_noise_ire > 0.0f ||
         c.burst_noise_ire > 0.0f || c.vhs_tracking != 0.0f ||
         c.vhs_wrinkle > 0.0f || c.vhs_head_switch > 0.0f || c.vhs_dropouts > 0.0f ||
         c.group_delay > 0.0f;
}

bool NtscEffects::GetParam(ParamId id, float* value) const {
  if (!value) return false;
  switch (id) {
    case ParamId::kNoiseStddevIre: *value = controls_.noise_stddev_ire; return true;
    case ParamId::kMultipathGain: *value = controls_.multipath_gain; return true;
    case ParamId::kMultipathDelaySamples: *value = static_cast<float>(controls_.multipath_delay_samples); return true;
    case ParamId::kMultipathEnsemble: *value = controls_.multipath_ensemble; return true;
    case ParamId::kLineTimeJitterSamples: *value = controls_.line_time_jitter_samples; return true;
    case ParamId::kAfcHunt: *value = controls_.afc_hunt; return true;
    case ParamId::kRfDrift: *value = controls_.rf_drift; return true;
    case ParamId::kAmNonlinearity: *value = controls_.am_nonlinearity; return true;
    case ParamId::kImpulseNoise: *value = controls_.impulse_noise; return true;
    case ParamId::kAgcPump: *value = controls_.agc_pump; return true;
    case ParamId::kHum: *value = controls_.hum; return true;
    case ParamId::kChromaFlutter: *value = controls_.chroma_flutter; return true;
    case ParamId::kYcCrosstalk: *value = controls_.yc_crosstalk; return true;
    case ParamId::kHSyncNoiseIre: *value = controls_.h_sync_noise_ire; return true;
    case ParamId::kVSyncNoiseIre: *value = controls_.v_sync_noise_ire; return true;
    case ParamId::kBurstNoiseIre: *value = controls_.burst_noise_ire; return true;
    case ParamId::kVhsTracking: *value = controls_.vhs_tracking; return true;
    case ParamId::kVhsWrinkle: *value = controls_.vhs_wrinkle; return true;
    case ParamId::kVhsHeadSwitch: *value = controls_.vhs_head_switch; return true;
    case ParamId::kVhsDropouts: *value = controls_.vhs_dropouts; return true;
    case ParamId::kNoiseColor: *value = controls_.noise_color; return true;
    case ParamId::kGroupDelay: *value = controls_.group_delay; return true;
    case ParamId::kEffectsRandomSeed: *value = static_cast<float>(controls_.random_seed); return true;
    default: return false;
  }
}

bool NtscEffects::SetParam(ParamId id, float value) {
  switch (id) {
    case ParamId::kNoiseStddevIre: controls_.noise_stddev_ire = value; return true;
    case ParamId::kMultipathGain: controls_.multipath_gain = value; return true;
    case ParamId::kMultipathDelaySamples: controls_.multipath_delay_samples = static_cast<uint32_t>(value); return true;
    case ParamId::kMultipathEnsemble: controls_.multipath_ensemble = value; return true;
    case ParamId::kLineTimeJitterSamples: controls_.line_time_jitter_samples = value; return true;
    case ParamId::kAfcHunt: controls_.afc_hunt = value; return true;
    case ParamId::kRfDrift: controls_.rf_drift = value; return true;
    case ParamId::kAmNonlinearity: controls_.am_nonlinearity = value; return true;
    case ParamId::kImpulseNoise: controls_.impulse_noise = value; return true;
    case ParamId::kAgcPump: controls_.agc_pump = value; return true;
    case ParamId::kHum: controls_.hum = value; return true;
    case ParamId::kChromaFlutter: controls_.chroma_flutter = value; return true;
    case ParamId::kYcCrosstalk: controls_.yc_crosstalk = value; return true;
    case ParamId::kHSyncNoiseIre: controls_.h_sync_noise_ire = value; return true;
    case ParamId::kVSyncNoiseIre: controls_.v_sync_noise_ire = value; return true;
    case ParamId::kBurstNoiseIre: controls_.burst_noise_ire = value; return true;
    case ParamId::kVhsTracking: controls_.vhs_tracking = value; return true;
    case ParamId::kVhsWrinkle: controls_.vhs_wrinkle = value; return true;
    case ParamId::kVhsHeadSwitch: controls_.vhs_head_switch = value; return true;
    case ParamId::kVhsDropouts: controls_.vhs_dropouts = value; return true;
    case ParamId::kNoiseColor: controls_.noise_color = value; return true;
    case ParamId::kGroupDelay: controls_.group_delay = value; return true;
    case ParamId::kEffectsRandomSeed: controls_.random_seed = static_cast<uint32_t>(value); return true;
    default: return false;
  }
}

}  // namespace rf2
