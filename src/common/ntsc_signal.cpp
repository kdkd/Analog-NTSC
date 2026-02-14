// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#include "rf2/ntsc_signal.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <random>
#include <type_traits>
#if defined(__aarch64__)
#include <arm_neon.h>
#elif defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#endif

#include "rf2/ntsc_effects.h"

namespace rf2 {

namespace {

inline float Clamp(float value, float lo, float hi) {
  return std::max(lo, std::min(hi, value));
}

constexpr float kPhaseCosLut[4][4] = {
    {1.0f, 0.0f, -1.0f, 0.0f},
    {0.0f, -1.0f, 0.0f, 1.0f},
    {-1.0f, 0.0f, 1.0f, 0.0f},
    {0.0f, 1.0f, 0.0f, -1.0f},
};

constexpr float kPhaseSinLut[4][4] = {
    {0.0f, 1.0f, 0.0f, -1.0f},
    {1.0f, 0.0f, -1.0f, 0.0f},
    {0.0f, -1.0f, 0.0f, 1.0f},
    {-1.0f, 0.0f, 1.0f, 0.0f},
};

constexpr float kBurstSinPiOffsetLut[4] = {0.0f, -1.0f, 0.0f, 1.0f};

inline int PhaseIndex(uint32_t frame_index, uint32_t line_index, uint32_t sample_index) {
  const uint32_t frame_q = frame_index & 0x3U;
  const uint32_t line_q = (line_index & 1U) ? 2U : 0U;
  return static_cast<int>((frame_q + line_q + (sample_index & 0x3U)) & 0x3U);
}

inline void ConvertIreLineToSamples(const float* src, uint32_t count, int16_t* dst) {
#if defined(__aarch64__)
  const float32x4_t v_lo = vdupq_n_f32(-80.0f);
  const float32x4_t v_hi = vdupq_n_f32(140.0f);
  const float32x4_t v_scale = vdupq_n_f32(256.0f);
  uint32_t i = 0;
  for (; i + 8 <= count; i += 8) {
    float32x4_t a = vld1q_f32(src + i);
    float32x4_t b = vld1q_f32(src + i + 4);
    a = vmaxq_f32(v_lo, vminq_f32(v_hi, a));
    b = vmaxq_f32(v_lo, vminq_f32(v_hi, b));
    a = vmulq_f32(a, v_scale);
    b = vmulq_f32(b, v_scale);
    const int32x4_t ai = vcvtnq_s32_f32(a);
    const int32x4_t bi = vcvtnq_s32_f32(b);
    const int16x8_t out = vcombine_s16(vqmovn_s32(ai), vqmovn_s32(bi));
    vst1q_s16(dst + i, out);
  }
  for (; i < count; ++i) {
    dst[i] = IreToSample(src[i]);
  }
#elif defined(__x86_64__) || defined(_M_X64)
  const __m128 v_lo = _mm_set1_ps(-80.0f);
  const __m128 v_hi = _mm_set1_ps(140.0f);
  const __m128 v_scale = _mm_set1_ps(256.0f);
  uint32_t i = 0;
  for (; i + 8 <= count; i += 8) {
    __m128 a = _mm_loadu_ps(src + i);
    __m128 b = _mm_loadu_ps(src + i + 4);
    a = _mm_max_ps(v_lo, _mm_min_ps(v_hi, a));
    b = _mm_max_ps(v_lo, _mm_min_ps(v_hi, b));
    a = _mm_mul_ps(a, v_scale);
    b = _mm_mul_ps(b, v_scale);
    const __m128i ai = _mm_cvtps_epi32(a);
    const __m128i bi = _mm_cvtps_epi32(b);
    const __m128i out = _mm_packs_epi32(ai, bi);
    _mm_storeu_si128(reinterpret_cast<__m128i*>(dst + i), out);
  }
  for (; i < count; ++i) {
    dst[i] = IreToSample(src[i]);
  }
#else
  for (uint32_t i = 0; i < count; ++i) {
    dst[i] = IreToSample(src[i]);
  }
#endif
}

void ApplyOnePoleLowpassBidirectional(std::vector<float>* values,
                                      float cutoff_hz,
                                      float sample_rate_hz,
                                      uint32_t passes) {
  if (!values || values->empty() || cutoff_hz <= 0.0f || sample_rate_hz <= 0.0f) {
    return;
  }
  if (passes == 0) {
    passes = 1;
  }

  // Pre-warp cutoff upward to compensate for cascaded stages narrowing the
  // effective -3 dB point.  Factor = sqrt(2^(1/1)-1) / sqrt(2^(1/N)-1).
  static constexpr float kPrewarp[] = {
      1.0f,       // 1 pass (identity)
      1.5538f,    // 2 passes
      2.1140f,    // 3 passes
      2.6645f,    // 4 passes
      3.2046f,    // 5 passes
      3.7359f,    // 6 passes
  };
  const uint32_t idx = std::min<uint32_t>(passes, 6) - 1;
  const float adjusted_cutoff = cutoff_hz * kPrewarp[idx];

  const float alpha = 1.0f - std::exp(-kTwoPi * adjusted_cutoff / sample_rate_hz);
  if (!(alpha > 0.0f && alpha < 1.0f)) {
    return;
  }

  for (uint32_t p = 0; p < passes; ++p) {
    float y = (*values)[0];
    for (size_t i = 1; i < values->size(); ++i) {
      y += alpha * ((*values)[i] - y);
      (*values)[i] = y;
    }

    y = (*values)[values->size() - 1];
    for (size_t i = values->size() - 1; i-- > 0;) {
      y += alpha * ((*values)[i] - y);
      (*values)[i] = y;
    }
  }
}

void ConstrainYiqToRgbGamut(float y, float* i, float* q) {
  // Chroma filtering can push YIQ out of displayable RGB gamut; constrain by
  // scaling chroma while preserving hue.
  float ii = *i;
  float qq = *q;
  for (int iter = 0; iter < 8; ++iter) {
    const float r = y + 0.9563f * ii + 0.6210f * qq;
    const float g = y - 0.2721f * ii - 0.6474f * qq;
    const float b = y - 1.1070f * ii + 1.7046f * qq;
    if (r >= 0.0f && r <= 1.0f && g >= 0.0f && g <= 1.0f && b >= 0.0f && b <= 1.0f) {
      break;
    }
    ii *= 0.9f;
    qq *= 0.9f;
  }
  *i = ii;
  *q = qq;
}

bool IsVisibleTopLine(int line, const NtscLineMap& map) {
  return line >= map.top_field_start &&
         line < map.top_field_start + map.lines_per_field_visible;
}

bool IsVisibleBottomLine(int line, const NtscLineMap& map) {
  return line >= map.bottom_field_start &&
         line < map.bottom_field_start + map.lines_per_field_visible;
}

int SourceRowForLine(int line, const NtscLineMap& map) {
  if (IsVisibleTopLine(line, map)) {
    return (line - map.top_field_start) * 2;
  }
  if (IsVisibleBottomLine(line, map)) {
    return (line - map.bottom_field_start) * 2 + 1;
  }
  return -1;
}

bool IsBroadSyncLine(int line) {
  return (line >= 3 && line <= 5) || (line >= 265 && line <= 267);
}

bool IsEqualizingLine(int line) {
  return (line >= 0 && line <= 2) || (line >= 6 && line <= 8) ||
         (line >= 262 && line <= 264) || (line >= 268 && line <= 270);
}

void WriteNormalSync(float* line, const NtscTiming& t, const NtscLevels& levels) {
  for (uint32_t i = 0; i < t.samples_per_line; ++i) {
    line[i] = levels.blank_ire;
  }
  for (uint32_t i = 0; i < t.hsync_samples; ++i) {
    line[i] = levels.sync_ire;
  }
}

void WriteBroadSync(float* line, const NtscTiming& t, const NtscLevels& levels) {
  for (uint32_t i = 0; i < t.samples_per_line; ++i) {
    line[i] = levels.blank_ire;
  }
  const uint32_t broad_samples = 392;
  const uint32_t pulse = std::min(broad_samples, t.samples_per_line);
  for (uint32_t i = 0; i < pulse; ++i) {
    line[i] = levels.sync_ire;
  }
}

void WriteEqualizingSync(float* line, const NtscTiming& t, const NtscLevels& levels) {
  for (uint32_t i = 0; i < t.samples_per_line; ++i) {
    line[i] = levels.blank_ire;
  }
  const uint32_t short_pulse = 33;
  const uint32_t half = t.samples_per_line / 2;
  for (uint32_t i = 0; i < short_pulse && i < t.samples_per_line; ++i) {
    line[i] = levels.sync_ire;
  }
  for (uint32_t i = 0; i < short_pulse && (half + i) < t.samples_per_line; ++i) {
    line[half + i] = levels.sync_ire;
  }
}

void AddBurst(float* line,
              int line_index,
              uint32_t frame_index,
              const NtscTiming& t,
              const NtscLevels& levels) {
  if (t.burst_start >= t.samples_per_line) {
    return;
  }
  const uint32_t span = std::min<uint32_t>(t.burst_samples, t.samples_per_line - t.burst_start);
  int phase = PhaseIndex(frame_index,
                         static_cast<uint32_t>(line_index),
                         t.burst_start);
  for (uint32_t i = 0; i < span; ++i) {
    const uint32_t s = t.burst_start + i;
    line[s] += levels.burst_peak_ire * kBurstSinPiOffsetLut[phase];
    phase = (phase + 1) & 0x3;
  }
}

uint32_t XorShift32(uint32_t* state) {
  uint32_t x = (*state == 0U) ? 0x6C8E9CF5u : *state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  *state = x;
  return x;
}

void DrawPseudoVitcBits(float* line,
                        int line_index,
                        uint32_t frame_index,
                        const NtscTiming& t,
                        const NtscLevels& levels) {
  if (t.active_start >= t.samples_per_line || t.active_samples < 200) {
    return;
  }
  const uint32_t active_span = std::min<uint32_t>(t.active_samples, t.samples_per_line - t.active_start);
  if (active_span < 180) {
    return;
  }

  const uint32_t s0 = t.active_start;
  const uint32_t s1 = s0 + active_span;
  const float black = levels.black_ire;
  const float white = levels.white_ire;

  // VBI payload area appears dark with bright digital-looking pulses.
  for (uint32_t s = s0; s < s1; ++s) {
    line[s] = black - 3.0f;
  }

  // Low-density pseudo VITC-like cell stream; contiguous high cells create
  // wider "run" bars rather than dense isolated dots.
  constexpr int kCells = 26;
  std::array<uint8_t, kCells> bits{};
  // Stable preamble-like marker.
  static constexpr uint16_t kMarkerA = 0b1110010110101101u;
  for (int i = 0; i < 10; ++i) {
    bits[i] = static_cast<uint8_t>((kMarkerA >> (15 - i)) & 1u);
  }
  // Frame counter bits (slowly changing structure).
  for (int i = 10; i < 18; ++i) {
    bits[i] = static_cast<uint8_t>((frame_index >> ((i - 10) % 8)) & 1u);
  }
  // Field/line identity marker bits.
  for (int i = 18; i < 22; ++i) {
    bits[i] = static_cast<uint8_t>(((line_index + i) & 1) == 0 ? 1 : 0);
  }
  // Remaining bits: pseudo-random with persistence so runs are common.
  uint32_t prng = static_cast<uint32_t>(0x9E3779B9u ^ frame_index * 747796405u ^
                                        static_cast<uint32_t>(line_index * 2891336453u));
  uint8_t prev = bits[21];
  for (int i = 22; i < kCells; ++i) {
    const uint32_t r = XorShift32(&prng);
    if ((r & 7u) < 5u) {
      bits[i] = prev;
    } else {
      bits[i] = static_cast<uint8_t>((r >> 4) & 1u);
    }
    prev = bits[i];
  }

  const uint32_t margin = 14U;
  const uint32_t start = s0 + margin;
  const uint32_t width = (active_span > (2U * margin)) ? (active_span - 2U * margin) : active_span;
  const float cell_w = static_cast<float>(width) / static_cast<float>(kCells);

  // Baseline low level across payload region.
  for (uint32_t s = start; s < std::min<uint32_t>(s1, start + width); ++s) {
    line[s] = black - 3.0f;
  }

  for (int c = 0; c < kCells; ++c) {
    const float cell_start_f = static_cast<float>(start) + static_cast<float>(c) * cell_w;
    const float cell_end_f = static_cast<float>(start) + static_cast<float>(c + 1) * cell_w;
    const uint32_t cell_start = std::min<uint32_t>(s1 - 1, static_cast<uint32_t>(std::floor(cell_start_f)));
    const uint32_t cell_end = std::min<uint32_t>(s1, std::max<uint32_t>(cell_start + 1, static_cast<uint32_t>(std::ceil(cell_end_f))));
    if (cell_start >= cell_end) {
      continue;
    }

    const float v_target = bits[c] ? (white - 2.0f) : (black - 3.0f);
    for (uint32_t s = cell_start; s < cell_end; ++s) {
      line[s] = Clamp(v_target, levels.sync_ire, 120.0f);
    }
  }

  // Soften only at transitions between adjacent cells, preserving contiguous
  // high runs without gaps.
  for (int c = 1; c < kCells; ++c) {
    if (bits[c] == bits[c - 1]) {
      continue;
    }
    const float cell_edge_f = static_cast<float>(start) + static_cast<float>(c) * cell_w;
    const int edge = static_cast<int>(std::lrintf(cell_edge_f));
    const int blend = 2;
    for (int k = -blend; k <= blend; ++k) {
      const int s = edge + k;
      if (s < static_cast<int>(s0) || s >= static_cast<int>(s1)) {
        continue;
      }
      const float t = 0.5f + 0.5f * (static_cast<float>(k) / static_cast<float>(blend + 1));
      const float left_v = bits[c - 1] ? (white - 2.0f) : (black - 3.0f);
      const float right_v = bits[c] ? (white - 2.0f) : (black - 3.0f);
      line[static_cast<size_t>(s)] = Clamp(left_v + (right_v - left_v) * t, levels.sync_ire, 120.0f);
    }
  }
}

void DrawPseudoVitsBars(float* line,
                        int line_index,
                        uint32_t frame_index,
                        const NtscTiming& t,
                        const NtscLevels& levels) {
  if (t.active_start >= t.samples_per_line || t.active_samples < 120) {
    return;
  }
  const uint32_t active_span = std::min<uint32_t>(t.active_samples, t.samples_per_line - t.active_start);
  const uint32_t s0 = t.active_start;
  const uint32_t s1 = s0 + active_span;
  const float black = levels.black_ire;
  const float white = levels.white_ire;

  for (uint32_t s = s0; s < s1; ++s) {
    line[s] = black - 2.0f;
  }

  uint32_t prng = static_cast<uint32_t>(0xA511E9B3u ^ (frame_index * 2654435761u) ^
                                        static_cast<uint32_t>((line_index + 37) * 2246822519u));
  const int bars = 8 + static_cast<int>(XorShift32(&prng) % 6u);
  uint32_t cursor = s0 + 10U + (XorShift32(&prng) % 10u);
  for (int i = 0; i < bars && cursor + 8U < s1; ++i) {
    const uint32_t w = 6U + (XorShift32(&prng) % 40u);
    const uint32_t gap = 4U + (XorShift32(&prng) % 18u);
    const uint32_t end = std::min<uint32_t>(s1, cursor + w);
    const uint32_t soft = std::min<uint32_t>(2, end - cursor);
    for (uint32_t s = cursor; s < end; ++s) {
      float alpha = 1.0f;
      if (soft > 0U) {
        const uint32_t left = s - cursor;
        const uint32_t right = end - 1U - s;
        if (left < soft) {
          alpha *= static_cast<float>(left + 1U) / static_cast<float>(soft + 1U);
        }
        if (right < soft) {
          alpha *= static_cast<float>(right + 1U) / static_cast<float>(soft + 1U);
        }
      }
      line[s] = Clamp((1.0f - alpha) * line[s] + alpha * (white - 1.5f), levels.sync_ire, 120.0f);
    }
    cursor = end + gap;
  }
}

void AddPseudoVbiData(float* line,
                      int line_index,
                      uint32_t frame_index,
                      const NtscSignalConfig& config) {
  const auto& map = config.line_map;
  const auto& t = config.timing;
  const auto& levels = config.levels;

  auto classify = [&](int base) -> int {
    const int off = line_index - base;
    // Two VITS-like bar lines directly above active picture.
    if (off == -1 || off == -2) {
      return 1;
    }
    // Three digital-looking lines slightly higher in VBI.
    if (off <= -3 && off >= -5) {
      return 2;
    }
    return 0;
  };

  int kind = classify(map.top_field_start);
  if (kind == 0) {
    kind = classify(map.bottom_field_start);
  }
  if (kind == 1) {
    DrawPseudoVitsBars(line, line_index, frame_index, t, levels);
  } else if (kind == 2) {
    DrawPseudoVitcBits(line, line_index, frame_index, t, levels);
  }
}

}  // namespace

int16_t IreToSample(float ire) {
  const float scaled = Clamp(ire, -80.0f, 140.0f) * 256.0f;
  const int value = static_cast<int>(std::lrintf(scaled));
  return static_cast<int16_t>(std::max(-32768, std::min(32767, value)));
}

float SampleToIre(int16_t sample) { return static_cast<float>(sample) / 256.0f; }

template<typename OutputT>
static void EncodeCompositeImpl(const uint8_t* rgb720x480,
                                uint32_t frame_index,
                                const NtscSignalConfig& config,
                                std::vector<OutputT>* output) {
  const auto& t = config.timing;
  const auto& levels = config.levels;
  const auto& map = config.line_map;

  output->assign(static_cast<size_t>(t.lines_per_frame) * t.samples_per_line, OutputT{0});
  std::vector<float> line_buf(t.samples_per_line, levels.blank_ire);
  std::vector<float> y_line(t.active_samples, 0.0f);
  std::vector<float> i_line(t.active_samples, 0.0f);
  std::vector<float> q_line(t.active_samples, 0.0f);
  constexpr float kCompositeSampleRateHz = 315000000.0f / 22.0f;  // 4*fsc

  for (uint32_t line = 0; line < t.lines_per_frame; ++line) {
    if (IsBroadSyncLine(static_cast<int>(line))) {
      WriteBroadSync(line_buf.data(), t, levels);
    } else if (IsEqualizingLine(static_cast<int>(line))) {
      WriteEqualizingSync(line_buf.data(), t, levels);
    } else {
      WriteNormalSync(line_buf.data(), t, levels);
      AddBurst(line_buf.data(), static_cast<int>(line), frame_index, t, levels);
      AddPseudoVbiData(line_buf.data(), static_cast<int>(line), frame_index, config);
    }

    const int src_row = SourceRowForLine(static_cast<int>(line), map);
    if (src_row >= 0 && src_row < 480) {
      {
        const int row_off = src_row * 720 * 3;
        const float x_scale = 720.0f / static_cast<float>(t.active_samples);
        constexpr float kInv255 = 1.0f / 255.0f;
        for (uint32_t x = 0; x < t.active_samples; ++x) {
          const float src_x = Clamp((static_cast<float>(x) + 0.5f) * x_scale - 0.5f, 0.0f, 719.0f);
          const int x0 = static_cast<int>(src_x);
          const int x1 = std::min(719, x0 + 1);
          const float frac = src_x - static_cast<float>(x0);
          const float inv = 1.0f - frac;
          const uint8_t* p0 = rgb720x480 + row_off + x0 * 3;
          const uint8_t* p1 = rgb720x480 + row_off + x1 * 3;
          const float r = (p0[0] * inv + p1[0] * frac) * kInv255;
          const float g = (p0[1] * inv + p1[1] * frac) * kInv255;
          const float b = (p0[2] * inv + p1[2] * frac) * kInv255;
          y_line[x] = 0.299f * r + 0.587f * g + 0.114f * b;
          i_line[x] = 0.595716f * r - 0.274453f * g - 0.321263f * b;
          q_line[x] = 0.211456f * r - 0.522591f * g + 0.311135f * b;
        }
      }

      // Approximate transmitter/channel bandwidth shaping.
      const uint32_t fp = std::max(1U, config.filter_passes);
      ApplyOnePoleLowpassBidirectional(&y_line, config.luma_cutoff_hz, kCompositeSampleRateHz, fp);
      ApplyOnePoleLowpassBidirectional(&i_line, config.i_cutoff_hz, kCompositeSampleRateHz, fp);
      ApplyOnePoleLowpassBidirectional(&q_line, config.q_cutoff_hz, kCompositeSampleRateHz, fp);

      uint32_t active_span = t.active_samples;
      if (t.active_start >= t.samples_per_line) {
        active_span = 0;
      } else {
        active_span = std::min<uint32_t>(t.active_samples, t.samples_per_line - t.active_start);
      }
      for (uint32_t x = 0; x < active_span; ++x) {
        float y = Clamp(y_line[x], 0.0f, 1.0f);
        float i = i_line[x];
        float q = q_line[x];
        ConstrainYiqToRgbGamut(y, &i, &q);
        y_line[x] = y;
        i_line[x] = i;
        q_line[x] = q;
      }

      const float luma_gain = (levels.white_ire - levels.black_ire);
      const float chroma_gain = config.chroma_mod_scale * levels.chroma_gain_ire;
      float* line_active = line_buf.data() + t.active_start;
      const int phase_base = PhaseIndex(frame_index, line, t.active_start);
#if defined(__aarch64__)
      uint32_t x = 0;
      const float32x4_t v_luma_black = vdupq_n_f32(levels.black_ire);
      const float32x4_t v_luma_gain = vdupq_n_f32(luma_gain);
      const float32x4_t v_chroma_gain = vdupq_n_f32(chroma_gain);
      const float32x4_t v_sync = vdupq_n_f32(levels.sync_ire);
      const float32x4_t v_max = vdupq_n_f32(120.0f);
      const float32x4_t v_cos = vld1q_f32(kPhaseCosLut[phase_base]);
      const float32x4_t v_sin = vld1q_f32(kPhaseSinLut[phase_base]);
      for (; x + 4 <= active_span; x += 4) {
        const float32x4_t vy = vld1q_f32(y_line.data() + x);
        const float32x4_t vi = vld1q_f32(i_line.data() + x);
        const float32x4_t vq = vld1q_f32(q_line.data() + x);
        const float32x4_t luma = vmlaq_f32(v_luma_black, vy, v_luma_gain);
        float32x4_t mod = vmulq_f32(vi, v_cos);
        mod = vmlaq_f32(mod, vq, v_sin);
        const float32x4_t chroma = vmulq_f32(mod, v_chroma_gain);
        float32x4_t out = vaddq_f32(luma, chroma);
        out = vmaxq_f32(v_sync, vminq_f32(v_max, out));
        vst1q_f32(line_active + x, out);
      }
      int phase = (phase_base + static_cast<int>(x)) & 0x3;
      for (; x < active_span; ++x) {
        const float y = y_line[x];
        const float i = i_line[x];
        const float q = q_line[x];
        const float mod = i * kPhaseCosLut[phase][0] + q * kPhaseSinLut[phase][0];
        const float luma_ire = levels.black_ire + y * luma_gain;
        line_active[x] = Clamp(luma_ire + chroma_gain * mod, levels.sync_ire, 120.0f);
        phase = (phase + 1) & 0x3;
      }
#elif defined(__x86_64__) || defined(_M_X64)
      uint32_t x = 0;
      const __m128 v_luma_black = _mm_set1_ps(levels.black_ire);
      const __m128 v_luma_gain = _mm_set1_ps(luma_gain);
      const __m128 v_chroma_gain = _mm_set1_ps(chroma_gain);
      const __m128 v_sync = _mm_set1_ps(levels.sync_ire);
      const __m128 v_max = _mm_set1_ps(120.0f);
      const __m128 v_cos = _mm_loadu_ps(kPhaseCosLut[phase_base]);
      const __m128 v_sin = _mm_loadu_ps(kPhaseSinLut[phase_base]);
      for (; x + 4 <= active_span; x += 4) {
        const __m128 vy = _mm_loadu_ps(y_line.data() + x);
        const __m128 vi = _mm_loadu_ps(i_line.data() + x);
        const __m128 vq = _mm_loadu_ps(q_line.data() + x);
        const __m128 luma = _mm_add_ps(v_luma_black, _mm_mul_ps(vy, v_luma_gain));
        __m128 mod = _mm_add_ps(_mm_mul_ps(vi, v_cos), _mm_mul_ps(vq, v_sin));
        const __m128 chroma = _mm_mul_ps(mod, v_chroma_gain);
        __m128 out = _mm_add_ps(luma, chroma);
        out = _mm_max_ps(v_sync, _mm_min_ps(v_max, out));
        _mm_storeu_ps(line_active + x, out);
      }
      int phase = (phase_base + static_cast<int>(x)) & 0x3;
      for (; x < active_span; ++x) {
        const float y = y_line[x];
        const float i = i_line[x];
        const float q = q_line[x];
        const float mod = i * kPhaseCosLut[phase][0] + q * kPhaseSinLut[phase][0];
        const float luma_ire = levels.black_ire + y * luma_gain;
        line_active[x] = Clamp(luma_ire + chroma_gain * mod, levels.sync_ire, 120.0f);
        phase = (phase + 1) & 0x3;
      }
#else
      int phase = phase_base;
      for (uint32_t x = 0; x < active_span; ++x) {
        const float y = y_line[x];
        const float i = i_line[x];
        const float q = q_line[x];
        const float mod = i * kPhaseCosLut[phase][0] + q * kPhaseSinLut[phase][0];
        const float luma_ire = levels.black_ire + y * luma_gain;
        line_active[x] = Clamp(luma_ire + chroma_gain * mod, levels.sync_ire, 120.0f);
        phase = (phase + 1) & 0x3;
      }
#endif

    }

    OutputT* dst = output->data() + static_cast<size_t>(line) * t.samples_per_line;
    if constexpr (std::is_same_v<OutputT, int16_t>) {
      ConvertIreLineToSamples(line_buf.data(), t.samples_per_line, dst);
    } else {
      std::copy_n(line_buf.data(), t.samples_per_line, dst);
    }
  }
}

void EncodeNtscCompositeFrame(const uint8_t* rgb720x480,
                              uint32_t frame_index,
                              const NtscSignalConfig& config,
                              std::vector<int16_t>* out_samples) {
  EncodeCompositeImpl(rgb720x480, frame_index, config, out_samples);
}

void EncodeNtscCompositeFrameFloat(const uint8_t* rgb720x480,
                                    uint32_t frame_index,
                                    const NtscSignalConfig& config,
                                    std::vector<float>* out_ire) {
  EncodeCompositeImpl(rgb720x480, frame_index, config, out_ire);
}

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
                           uint32_t random_seed) {
  // Delegate to the struct-based implementation in ntsc_effects.cpp.
  NtscEffectControls controls;
  controls.noise_stddev_ire = noise_stddev_ire;
  controls.multipath_gain = multipath_gain;
  controls.multipath_delay_samples = multipath_delay_samples;
  controls.line_time_jitter_samples = line_time_jitter_samples;
  controls.rf_drift = rf_drift;
  controls.am_nonlinearity = am_nonlinearity;
  controls.impulse_noise = impulse_noise;
  controls.afc_hunt = afc_hunt;
  controls.chroma_flutter = chroma_flutter;
  controls.agc_pump = agc_pump;
  controls.multipath_ensemble = multipath_ensemble;
  controls.hum = hum;
  controls.yc_crosstalk = yc_crosstalk;
  controls.h_sync_noise_ire = h_lock_noise_ire;
  controls.v_sync_noise_ire = v_hold_noise_ire;
  controls.burst_noise_ire = burst_lock_noise_ire;
  controls.vhs_tracking = vhs_tracking;
  controls.vhs_wrinkle = vhs_wrinkle;
  controls.vhs_head_switch = vhs_head_switch;
  controls.vhs_dropouts = vhs_dropouts;
  controls.random_seed = random_seed;

  NtscSignalConfig config;
  config.timing.lines_per_frame = lines_per_frame;
  config.timing.samples_per_line = samples_per_line;
  config.timing.hsync_samples = hsync_samples;
  config.timing.burst_start = burst_start;
  config.timing.burst_samples = burst_samples;

  ApplyCompositeEffectsInternal(frame_ire, controls, config);
}

}  // namespace rf2
