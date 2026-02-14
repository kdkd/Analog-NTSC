// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#include "rf2/ntsc_decoder.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <random>

namespace rf2 {

namespace {

inline float Clamp(float value, float lo, float hi) {
  return std::max(lo, std::min(hi, value));
}

float SampleLine(const float* line, uint32_t samples_per_line, int index) {
  if (index < 0) {
    return line[0];
  }
  if (static_cast<uint32_t>(index) >= samples_per_line) {
    return line[samples_per_line - 1];
  }
  return line[index];
}

float EstimateBlankLevel(const float* line, const NtscTiming& t) {
  const uint32_t start = t.hsync_samples + 6;
  const uint32_t end = std::min<uint32_t>(t.burst_start, t.samples_per_line);
  if (start >= end) {
    return 0.0f;
  }
  float sum = 0.0f;
  uint32_t count = 0;
  for (uint32_t s = start; s < end; ++s) {
    sum += line[s];
    ++count;
  }
  return count ? (sum / static_cast<float>(count)) : 0.0f;
}

struct BurstStats {
  float phase = 0.0f;
  float amplitude = 0.0f;
};

// cos(s * pi/2) and sin(s * pi/2) cycle with period 4.
static constexpr float kCos90[4] = { 1.0f,  0.0f, -1.0f,  0.0f};
static constexpr float kSin90[4] = { 0.0f,  1.0f,  0.0f, -1.0f};

BurstStats EstimateBurstStats(const float* line, const NtscTiming& t, float blank) {
  float re = 0.0f;
  float im = 0.0f;
  uint32_t count = 0;
  for (uint32_t i = 0; i < t.burst_samples; ++i) {
    const uint32_t s = t.burst_start + i;
    if (s >= t.samples_per_line) {
      break;
    }
    const float v = line[s] - blank;
    re += v * kCos90[s & 3];
    im += v * kSin90[s & 3];
    ++count;
  }
  BurstStats out;
  out.phase = std::atan2(im, re);
  if (count > 0) {
    out.amplitude = std::sqrt(re * re + im * im) / static_cast<float>(count);
  }
  return out;
}

float MeanPrefix(const float* line, uint32_t samples_per_line, uint32_t count) {
  const uint32_t n = std::min(samples_per_line, count);
  if (n == 0) {
    return 0.0f;
  }
  float sum = 0.0f;
  for (uint32_t i = 0; i < n; ++i) {
    sum += line[i];
  }
  return sum / static_cast<float>(n);
}

int DetectSyncCenter(const float* line, const NtscTiming& t) {
  const uint32_t end = std::min<uint32_t>(t.samples_per_line, t.hsync_samples + 40U);
  if (end == 0) {
    return static_cast<int>(t.hsync_samples / 2U);
  }
  int min_idx = 0;
  float min_v = line[0];
  for (uint32_t s = 1; s < end; ++s) {
    if (line[s] < min_v) {
      min_v = line[s];
      min_idx = static_cast<int>(s);
    }
  }
  if (min_v > -8.0f) {
    return static_cast<int>(t.hsync_samples / 2U);
  }
  return min_idx;
}

int DetectVsyncAnchor(const std::vector<float>& frame_ire,
                      const NtscTiming& t,
                      int start_line,
                      int end_line) {
  if (start_line < 0) {
    start_line = 0;
  }
  if (end_line >= static_cast<int>(t.lines_per_frame)) {
    end_line = static_cast<int>(t.lines_per_frame) - 1;
  }
  if (start_line > end_line) {
    return -1;
  }

  int best_line = -1;
  float best_metric = 1e9f;
  for (int line = start_line; line <= end_line; ++line) {
    const float* row = frame_ire.data() + static_cast<size_t>(line) * t.samples_per_line;
    const float metric = MeanPrefix(row, t.samples_per_line, 220U);
    if (metric < best_metric) {
      best_metric = metric;
      best_line = line;
    }
  }
  return best_line;
}

float AverageSyncCenterError(const std::vector<float>& frame_ire,
                             const NtscTiming& t,
                             int start_line,
                             int end_line) {
  if (start_line < 0) {
    start_line = 0;
  }
  if (end_line >= static_cast<int>(t.lines_per_frame)) {
    end_line = static_cast<int>(t.lines_per_frame) - 1;
  }
  if (start_line > end_line) {
    return 0.0f;
  }

  const float nominal_center = static_cast<float>(t.hsync_samples) * 0.5f;
  float sum = 0.0f;
  int count = 0;
  for (int line = start_line; line <= end_line; ++line) {
    const float* row = frame_ire.data() + static_cast<size_t>(line) * t.samples_per_line;
    const int center = DetectSyncCenter(row, t);
    sum += static_cast<float>(center) - nominal_center;
    ++count;
  }
  return count > 0 ? (sum / static_cast<float>(count)) : 0.0f;
}

float WrapPi(float phase) {
  while (phase > kPi) {
    phase -= kTwoPi;
  }
  while (phase < -kPi) {
    phase += kTwoPi;
  }
  return phase;
}

float EstimateLuma(const float* line, uint32_t samples_per_line, int s, float blank) {
  const float lp = 0.125f * (SampleLine(line, samples_per_line, s - 2) - blank) +
                   0.25f * (SampleLine(line, samples_per_line, s - 1) - blank) +
                   0.25f * (SampleLine(line, samples_per_line, s) - blank) +
                   0.25f * (SampleLine(line, samples_per_line, s + 1) - blank) +
                   0.125f * (SampleLine(line, samples_per_line, s + 2) - blank);
  return lp;
}

void SoftLimitChroma(float* i, float* q, float max_radius) {
  const float mag = std::sqrt((*i) * (*i) + (*q) * (*q));
  if (mag <= max_radius || mag <= 1e-6f) {
    return;
  }
  const float scale = max_radius / mag;
  *i *= scale;
  *q *= scale;
}

}  // namespace

NtscDecoder::NtscDecoder(const NtscSignalConfig& config) : config_(config) {}

void NtscDecoder::Decode(const std::vector<float>& frame_ire,
                         std::vector<uint8_t>* rgb_out) const {
  const auto& t = config_.timing;
  const auto& levels = config_.levels;
  const auto& map = config_.line_map;
  const auto& controls = controls_;
  const size_t expected_samples =
      static_cast<size_t>(t.lines_per_frame) * static_cast<size_t>(t.samples_per_line);

  constexpr float kOutOfFrameIre = 10.0f;
  float bg_y_norm = (kOutOfFrameIre - levels.black_ire) / (levels.white_ire - levels.black_ire);
  bg_y_norm = (bg_y_norm - 0.5f) * controls.contrast + 0.5f + controls.brightness;
  bg_y_norm = Clamp(bg_y_norm, 0.0f, 1.0f);
  const uint8_t bg_u8 = static_cast<uint8_t>(std::lrintf(bg_y_norm * 255.0f));
  rgb_out->assign(720U * 480U * 3U, bg_u8);
  if (frame_ire.size() < expected_samples) {
    return;
  }

  // PLL instability params: use directly (0-1 range) with same power curves
  // as the original code used for (noise_ire / 16.0).
  const float h_lock_strength =
      std::pow(Clamp(controls.h_lock_instability, 0.0f, 1.0f), 1.6f);

  const float v_noise_norm = Clamp(controls.v_hold_instability, 0.0f, 1.0f);
  float v_lock_strength = 0.0f;
  if (v_noise_norm > 0.08f) {
    const float tv = (v_noise_norm - 0.08f) / 0.92f;
    v_lock_strength = std::pow(tv, 2.1f);
  }

  const float burst_noise_norm = Clamp(controls.burst_lock_instability, 0.0f, 1.0f);
  float burst_lock_strength = 0.0f;
  if (burst_noise_norm > 0.12f) {
    const float tb = (burst_noise_norm - 0.12f) / 0.88f;
    burst_lock_strength = std::pow(tb, 2.2f);
  }

  const bool enable_h_lock_tracking = (h_lock_strength > 0.001f);
  const bool enable_v_lock_tracking = (v_lock_strength > 0.001f);
  const bool enable_burst_lock_tracking = (burst_lock_strength > 0.001f);
  std::mt19937 frame_rng(static_cast<uint32_t>(controls.random_seed ^ 0x71C3D29u));
  std::uniform_real_distribution<float> uni01(0.0f, 1.0f);
  std::normal_distribution<float> gauss01(0.0f, 1.0f);

  std::vector<int> line_h_offset(t.lines_per_frame, 0);
  int top_start = map.top_field_start;
  int bottom_start = map.bottom_field_start;
  if (enable_h_lock_tracking) {
    float h_phase = 0.0f;
    const float nominal_center = static_cast<float>(t.hsync_samples) * 0.5f;
    for (uint32_t line = 0; line < t.lines_per_frame; ++line) {
      const float* row = frame_ire.data() + static_cast<size_t>(line) * t.samples_per_line;
      const int center = DetectSyncCenter(row, t);
      const float error = static_cast<float>(center) - nominal_center;
      const float h_k = 0.04f + 0.32f * h_lock_strength;
      h_phase += h_k * (error - h_phase);
      const float h_lim = 2.0f + 22.0f * h_lock_strength;
      h_phase = Clamp(h_phase, -h_lim, h_lim);
      line_h_offset[line] = static_cast<int>(std::lrintf(h_phase));
    }
  }

  if (enable_v_lock_tracking) {
    const int top_search_end = std::min<int>(
        static_cast<int>(t.lines_per_frame) - 1,
        40 + static_cast<int>(std::lrintf(90.0f * v_lock_strength)));
    const int bot_search_start = std::max<int>(
        180, 250 - static_cast<int>(std::lrintf(40.0f * v_lock_strength)));
    const int bot_search_end = std::min<int>(
        static_cast<int>(t.lines_per_frame) - 1,
        320 + static_cast<int>(std::lrintf(90.0f * v_lock_strength)));

    const int detected_top_vsync = DetectVsyncAnchor(
        frame_ire, t, 0, top_search_end);
    const int detected_bottom_vsync = DetectVsyncAnchor(
        frame_ire, t, bot_search_start, bot_search_end);

    const int top_ref = (detected_top_vsync >= 0) ? detected_top_vsync : 3;
    const int bot_ref = (detected_bottom_vsync >= 0) ? detected_bottom_vsync : 265;
    const int tracked_top_start = top_ref + (map.top_field_start - 3);
    const int tracked_bottom_start = bot_ref + (map.bottom_field_start - 265);

    const float v_event_prob = 0.70f * std::pow(v_lock_strength, 1.35f);
    const bool v_event = (uni01(frame_rng) < v_event_prob);
    const float v_gain = v_event ? (0.55f + 3.2f * v_lock_strength)
                                 : (0.05f + 0.45f * v_lock_strength);
    int top_candidate = map.top_field_start + static_cast<int>(std::lrintf(
                                           static_cast<float>(tracked_top_start - map.top_field_start) *
                                           v_gain));
    int bottom_candidate = map.bottom_field_start + static_cast<int>(std::lrintf(
                                                 static_cast<float>(tracked_bottom_start - map.bottom_field_start) *
                                                 v_gain));

    const float top_sync_err = AverageSyncCenterError(
        frame_ire, t, 0, std::min<int>(24, static_cast<int>(t.lines_per_frame) - 1));
    const float bot_sync_err = AverageSyncCenterError(
        frame_ire, t, std::max<int>(0, static_cast<int>(t.lines_per_frame) - 24),
        static_cast<int>(t.lines_per_frame) - 1);
    const float err_scale = v_event ? (1.0f + 2.4f * v_lock_strength)
                                    : (0.15f + 0.8f * v_lock_strength);
    top_candidate += static_cast<int>(std::lrintf((top_sync_err / 5.0f) * err_scale));
    bottom_candidate += static_cast<int>(std::lrintf((bot_sync_err / 5.0f) * err_scale));

    const float jitter_sigma = v_event ? (0.15f + 2.6f * v_lock_strength)
                                       : (0.02f + 0.25f * v_lock_strength);
    const int frame_jitter = static_cast<int>(std::lrintf(gauss01(frame_rng) * jitter_sigma));
    top_candidate += frame_jitter;
    bottom_candidate += frame_jitter;

    const int max_delta = 1 + static_cast<int>(std::lrintf(16.0f * std::pow(v_lock_strength, 1.35f)));
    const int top_delta_raw = top_candidate - map.top_field_start;
    const int bottom_delta_raw = bottom_candidate - map.bottom_field_start;
    const int frame_delta =
        std::max(-max_delta,
                 std::min(max_delta,
                          static_cast<int>(std::lrintf(0.5f *
                                                       static_cast<float>(top_delta_raw + bottom_delta_raw)))));
    top_start = map.top_field_start + frame_delta;
    bottom_start = map.bottom_field_start + frame_delta;

    top_start = std::max(0, std::min(static_cast<int>(t.lines_per_frame) - 1, top_start));
    bottom_start = std::max(0, std::min(static_cast<int>(t.lines_per_frame) - 1, bottom_start));
  }

  const float tint = controls.tint_degrees * (kPi / 180.0f);
  const float tint_cos = std::cos(tint);
  const float tint_sin = std::sin(tint);
  const float overscan_reveal = Clamp(controls.overscan_reveal, 0.0f, 0.25f);
  const float overscan_norm = overscan_reveal / 0.25f;
  const int base_visible = std::max(1, map.lines_per_field_visible);
  const int max_extra = 20;
  const int extra_lines = static_cast<int>(std::lrintf(overscan_reveal * static_cast<float>(max_extra)));
  const int sample_lines = base_visible + extra_lines * 2;
  const int active_start = static_cast<int>(t.active_start);
  const int active_samples = static_cast<int>(t.active_samples);
  const int samples_per_line_i = static_cast<int>(t.samples_per_line);
  const int active_end = active_start + active_samples - 1;
  const int left_margin = std::max(0, active_start);
  const int right_margin = std::max(0, samples_per_line_i - 1 - active_end);
  const int extra_left = static_cast<int>(std::lrintf(overscan_norm * static_cast<float>(left_margin)));
  const int extra_right = static_cast<int>(std::lrintf(overscan_norm * static_cast<float>(right_margin)));
  const int sample_start = std::max(0, active_start - extra_left);
  const int sample_end = std::min(samples_per_line_i - 1, active_end + extra_right);
  const int sample_width = std::max(1, sample_end - sample_start + 1);
  const bool default_h_sample = (sample_start == active_start && sample_width == active_samples);
  constexpr float kCompositeSampleRateHz = 315000000.0f / 22.0f;
  constexpr float kIDecodeCutoffHz = 1.2e6f;
  constexpr float kQDecodeCutoffHz = 0.5e6f;
  const float alpha_i = 1.0f - std::exp(-kTwoPi * kIDecodeCutoffHz / kCompositeSampleRateHz);
  const float alpha_q = 1.0f - std::exp(-kTwoPi * kQDecodeCutoffHz / kCompositeSampleRateHz);
  const int chroma_delay =
      static_cast<int>(std::lrintf(controls.chroma_delay_pixels *
                                   (static_cast<float>(t.active_samples) / 720.0f)));
  float burst_phase_state[2] = {0.0f, 0.0f};
  bool burst_phase_initialized[2] = {false, false};
  float frame_chroma_gate = 1.0f;
  if (enable_burst_lock_tracking) {
    float amp_sum = 0.0f;
    int amp_count = 0;
    for (int i = 0; i < 32; ++i) {
      const int line = top_start + i;
      if (line < 0 || static_cast<uint32_t>(line) >= t.lines_per_frame) {
        continue;
      }
      const float* row = frame_ire.data() + static_cast<size_t>(line) * t.samples_per_line;
      const float blank = EstimateBlankLevel(row, t);
      amp_sum += EstimateBurstStats(row, t, blank).amplitude;
      ++amp_count;
    }
    const float avg_amp = (amp_count > 0) ? (amp_sum / static_cast<float>(amp_count)) : 0.0f;
    const float stress = Clamp((2.6f - avg_amp) / 2.1f, 0.0f, 1.0f);
    const float kill_prob = Clamp(0.005f + 0.30f * std::pow(burst_lock_strength, 1.2f) +
                                      0.35f * std::pow(burst_lock_strength, 1.8f) *
                                          (0.35f + 0.65f * stress),
                                  0.0f, 0.70f);
    const bool color_killed = (uni01(frame_rng) < kill_prob);
    if (color_killed) {
      frame_chroma_gate = 0.0f;
    } else {
      const float flutter = (uni01(frame_rng) - 0.5f) * 0.08f * burst_lock_strength * stress;
      frame_chroma_gate =
          Clamp(1.0f - (0.22f * burst_lock_strength * stress) + flutter, 0.0f, 1.0f);
    }
  }

  for (int y = 0; y < 480; ++y) {
    const int field_row = y / 2;
    const float mapped_row =
        (((static_cast<float>(field_row) + 0.5f) / static_cast<float>(base_visible)) *
             static_cast<float>(sample_lines)) -
        0.5f - static_cast<float>(extra_lines);
    const int row_idx = static_cast<int>(std::lrintf(mapped_row));
    const int line = (y & 1) ? (bottom_start + row_idx) : (top_start + row_idx);
    if (line < 0 || static_cast<uint32_t>(line) >= t.lines_per_frame) {
      continue;
    }

    const float* cur = frame_ire.data() + static_cast<size_t>(line) * t.samples_per_line;
    const float blank = EstimateBlankLevel(cur, t);
    const BurstStats burst = EstimateBurstStats(cur, t, blank);
    float decode_phase = -0.5f * kPi - burst.phase;

    float chroma_lock = 1.0f;
    if (enable_burst_lock_tracking) {
      const int parity = line & 1;
      if (!burst_phase_initialized[parity]) {
        burst_phase_state[parity] = burst.phase;
        burst_phase_initialized[parity] = true;
      }
      const float per_line_err = WrapPi(burst.phase - burst_phase_state[parity]);
      if (burst.amplitude > 2.0f) {
        const float phase_k = 0.02f + 0.05f * burst_lock_strength;
        burst_phase_state[parity] = WrapPi(burst_phase_state[parity] + phase_k * per_line_err);
      }
      decode_phase = -0.5f * kPi - burst_phase_state[parity];

      const float line_gate = Clamp((burst.amplitude - 1.0f) / 7.0f, 0.0f, 1.0f);
      chroma_lock = frame_chroma_gate * (0.9f + 0.1f * line_gate);
    }

    const float* prev = nullptr;
    const float* next = nullptr;
    float prev_blank = blank;
    float next_blank = blank;
    if (line > 0) {
      prev = frame_ire.data() + static_cast<size_t>(line - 1) * t.samples_per_line;
      prev_blank = EstimateBlankLevel(prev, t);
    }
    if (static_cast<uint32_t>(line + 1) < t.lines_per_frame) {
      next = frame_ire.data() + static_cast<size_t>(line + 1) * t.samples_per_line;
      next_blank = EstimateBlankLevel(next, t);
    }

    // Precompute demodulation cos/sin for the 4 phase slots on this line.
    // cos(ss*π/2 + dp) = cos90[ss%4]*cos(dp) - sin90[ss%4]*sin(dp)
    const float dp_cos = std::cos(decode_phase);
    const float dp_sin = std::sin(decode_phase);
    float demod_cos[4], demod_sin[4];
    for (int p = 0; p < 4; ++p) {
      demod_cos[p] = kCos90[p] * dp_cos - kSin90[p] * dp_sin;
      demod_sin[p] = kSin90[p] * dp_cos + kCos90[p] * dp_sin;
    }

    const float inv_chroma_gain = 0.5f / config_.levels.chroma_gain_ire;
    float i_state = 0.0f;
    float q_state = 0.0f;

    for (int x = 0; x < 720; ++x) {
      int s = 0;
      if (default_h_sample) {
        const uint32_t active_x =
            static_cast<uint32_t>((static_cast<uint64_t>(x) * t.active_samples) / 720U);
        s = static_cast<int>(t.active_start + active_x + line_h_offset[line]);
      } else {
        const float mapped_col =
            (((static_cast<float>(x) + 0.5f) / 720.0f) * static_cast<float>(sample_width)) - 0.5f;
        s = sample_start + static_cast<int>(std::lrintf(mapped_col)) + line_h_offset[line];
      }
      s = std::min<int>(std::max<int>(0, s), static_cast<int>(t.samples_per_line - 1));

      float y_ire = EstimateLuma(cur, t.samples_per_line, s, blank);

      int chroma_s = s + chroma_delay;
      chroma_s = std::max(0, std::min(static_cast<int>(t.samples_per_line - 1), chroma_s));

      float i_acc = 0.0f;
      float q_acc = 0.0f;
      for (int tap = -2; tap <= 1; ++tap) {
        const int ss = chroma_s + tap;
        const float vv = SampleLine(cur, t.samples_per_line, ss) - blank;

        float chroma_v = 0.0f;
        if (controls.comb_filter && !controls.dot_crawl && prev && next) {
          const float p = SampleLine(prev, t.samples_per_line, ss) - prev_blank;
          const float n = SampleLine(next, t.samples_per_line, ss) - next_blank;
          chroma_v = 0.5f * (vv - 0.5f * (p + n));
        } else {
          chroma_v = vv - EstimateLuma(cur, t.samples_per_line, ss, blank);
        }

        const int ph = ss & 3;
        i_acc += chroma_v * demod_cos[ph];
        q_acc += chroma_v * demod_sin[ph];
      }
      const float i_est = i_acc * inv_chroma_gain;
      const float q_est = q_acc * inv_chroma_gain;

      i_state += alpha_i * (i_est - i_state);
      q_state += alpha_q * (q_est - q_state);

      float i_rot = i_state * tint_cos - q_state * tint_sin;
      float q_rot = i_state * tint_sin + q_state * tint_cos;
      SoftLimitChroma(&i_rot, &q_rot, 0.42f);
      i_rot *= controls.saturation * chroma_lock;
      q_rot *= controls.saturation * chroma_lock;

      float y_norm = (y_ire - levels.black_ire) / (levels.white_ire - levels.black_ire);
      y_norm = (y_norm - 0.5f) * controls.contrast + 0.5f + controls.brightness;

      if (controls.sharpness > 0.0f) {
        const float yl =
            (EstimateLuma(cur, t.samples_per_line, s - 1, blank) - levels.black_ire) /
            (levels.white_ire - levels.black_ire);
        const float yr =
            (EstimateLuma(cur, t.samples_per_line, s + 1, blank) - levels.black_ire) /
            (levels.white_ire - levels.black_ire);
        const float edge = y_norm - 0.5f * (yl + yr);
        y_norm = y_norm + edge * controls.sharpness * 0.5f;
      }

      float r = y_norm + 0.9563f * i_rot + 0.6210f * q_rot;
      float g = y_norm - 0.2721f * i_rot - 0.6474f * q_rot;
      float b = y_norm - 1.1070f * i_rot + 1.7046f * q_rot;

      r = Clamp(r, 0.0f, 1.0f);
      g = Clamp(g, 0.0f, 1.0f);
      b = Clamp(b, 0.0f, 1.0f);

      const size_t out = static_cast<size_t>(y * 720 + x) * 3U;
      (*rgb_out)[out + 0] = static_cast<uint8_t>(std::lrintf(r * 255.0f));
      (*rgb_out)[out + 1] = static_cast<uint8_t>(std::lrintf(g * 255.0f));
      (*rgb_out)[out + 2] = static_cast<uint8_t>(std::lrintf(b * 255.0f));
    }
  }
}

bool NtscDecoder::GetParam(ParamId id, float* value) const {
  if (!value) return false;
  switch (id) {
    case ParamId::kBrightness: *value = controls_.brightness; return true;
    case ParamId::kContrast: *value = controls_.contrast; return true;
    case ParamId::kSaturation: *value = controls_.saturation; return true;
    case ParamId::kTintDegrees: *value = controls_.tint_degrees; return true;
    case ParamId::kSharpness: *value = controls_.sharpness; return true;
    case ParamId::kCombFilter: *value = controls_.comb_filter ? 1.0f : 0.0f; return true;
    case ParamId::kDotCrawl: *value = controls_.dot_crawl ? 1.0f : 0.0f; return true;
    case ParamId::kChromaDelayPixels: *value = controls_.chroma_delay_pixels; return true;
    case ParamId::kOverscanReveal: *value = controls_.overscan_reveal; return true;
    case ParamId::kHLockInstability: *value = controls_.h_lock_instability; return true;
    case ParamId::kVHoldInstability: *value = controls_.v_hold_instability; return true;
    case ParamId::kBurstLockInstability: *value = controls_.burst_lock_instability; return true;
    case ParamId::kDecoderRandomSeed: *value = static_cast<float>(controls_.random_seed); return true;
    default: return false;
  }
}

bool NtscDecoder::SetParam(ParamId id, float value) {
  switch (id) {
    case ParamId::kBrightness: controls_.brightness = value; return true;
    case ParamId::kContrast: controls_.contrast = value; return true;
    case ParamId::kSaturation: controls_.saturation = value; return true;
    case ParamId::kTintDegrees: controls_.tint_degrees = value; return true;
    case ParamId::kSharpness: controls_.sharpness = value; return true;
    case ParamId::kCombFilter: controls_.comb_filter = (value >= 0.5f); return true;
    case ParamId::kDotCrawl: controls_.dot_crawl = (value >= 0.5f); return true;
    case ParamId::kChromaDelayPixels: controls_.chroma_delay_pixels = value; return true;
    case ParamId::kOverscanReveal: controls_.overscan_reveal = value; return true;
    case ParamId::kHLockInstability: controls_.h_lock_instability = value; return true;
    case ParamId::kVHoldInstability: controls_.v_hold_instability = value; return true;
    case ParamId::kBurstLockInstability: controls_.burst_lock_instability = value; return true;
    case ParamId::kDecoderRandomSeed: controls_.random_seed = static_cast<uint32_t>(value); return true;
    default: return false;
  }
}

}  // namespace rf2
