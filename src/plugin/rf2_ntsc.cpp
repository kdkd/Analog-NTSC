// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#include "rf2_ntsc.h"
#include "rf2/ntsc_processor.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>

namespace {

constexpr int kNtscW = 720;
constexpr int kNtscH = 480;

// ---------------------------------------------------------------------------
// Bilinear scale: BGRA input (any size) → RGB 720×480.
// - letterbox_to_43=true: aspect-preserve full source into 4:3 frame (bars)
// - letterbox_to_43=false: center-crop source to fill 4:3 frame (no bars)
// ---------------------------------------------------------------------------
inline float PixelAspect(const PF_RationalScale& par) {
  if (par.num <= 0 || par.den == 0) {
    return 1.0f;
  }
  return static_cast<float>(par.num) / static_cast<float>(par.den);
}

inline bool IsSdRaster43Candidate(int w, int h) {
  const bool sd_h = (h == 480 || h == 486);
  const bool sd_w = (w == 640 || w == 704 || w == 720);
  return sd_w && sd_h;
}

void ScaleInputToNtsc(const uint8_t* bgra, int srcW, int srcH, int rowBytes,
                      bool letterbox_to_43,
                      float src_display_aspect,
                      uint8_t* rgb) {
  if (srcW <= 0 || srcH <= 0) return;

  std::fill(rgb, rgb + (kNtscW * kNtscH * 3), 0);

  constexpr float kTargetAspect = 4.0f / 3.0f;
  float src_aspect = std::clamp(src_display_aspect, 0.25f, 4.0f);
  const float src_aspect_raw = static_cast<float>(srcW) / static_cast<float>(srcH);
  float src_par_eff = src_aspect / std::max(0.01f, src_aspect_raw);
  src_par_eff = std::clamp(src_par_eff, 0.1f, 4.0f);

  // If metadata is clearly unreliable, fall back to 4:3 for classic SD raster.
  if (IsSdRaster43Candidate(srcW, srcH) &&
      (src_aspect < 1.05f || src_aspect > 1.60f)) {
    src_aspect = kTargetAspect;
    src_par_eff = src_aspect / std::max(0.01f, src_aspect_raw);
  }

  float crop_x0 = 0.0f;
  float crop_y0 = 0.0f;
  float crop_w = static_cast<float>(srcW);
  float crop_h = static_cast<float>(srcH);
  if (!letterbox_to_43) {
    // Fill 4:3 frame by center-cropping source display aspect.
    if (src_aspect > kTargetAspect + 0.001f) {
      crop_w = (crop_h * kTargetAspect) / src_par_eff;
      crop_x0 = 0.5f * (static_cast<float>(srcW) - crop_w);
    } else if (src_aspect < kTargetAspect - 0.001f) {
      crop_h = (crop_w * src_par_eff) / kTargetAspect;
      crop_y0 = 0.5f * (static_cast<float>(srcH) - crop_h);
    }
  }
  const float crop_aspect =
      (crop_w * src_par_eff) / std::max(1.0f, crop_h);

  int drawW = kNtscW;
  int drawH = kNtscH;
  if (letterbox_to_43) {
    if (crop_aspect > kTargetAspect) {
      drawH = std::max(
          1, static_cast<int>(std::lrintf(static_cast<float>(kNtscH) * (kTargetAspect / crop_aspect))));
    } else {
      drawW = std::max(
          1, static_cast<int>(std::lrintf(static_cast<float>(kNtscW) * (crop_aspect / kTargetAspect))));
    }
  }
  drawW = std::min(kNtscW, drawW);
  drawH = std::min(kNtscH, drawH);
  const int offX = (kNtscW - drawW) / 2;
  const int offY = (kNtscH - drawH) / 2;

  const float sx = (drawW > 1) ? (crop_w - 1.0f) / (drawW - 1.0f) : 0.0f;
  const float sy = (drawH > 1) ? (crop_h - 1.0f) / (drawH - 1.0f) : 0.0f;

  for (int y = 0; y < drawH; ++y) {
    const float fy = crop_y0 + y * sy;
    const int y0 = static_cast<int>(fy);
    const int y1 = std::min(y0 + 1, srcH - 1);
    const float wy = fy - y0;
    const float wy1 = 1.0f - wy;

    const uint8_t* r0 = bgra + y0 * rowBytes;
    const uint8_t* r1 = bgra + y1 * rowBytes;
    uint8_t* dst = rgb + (offY + y) * kNtscW * 3 + offX * 3;

    for (int x = 0; x < drawW; ++x) {
      const float fx = crop_x0 + x * sx;
      const int x0 = static_cast<int>(fx);
      const int x1 = std::min(x0 + 1, srcW - 1);
      const float wx = fx - x0;
      const float wx1 = 1.0f - wx;

      const float w00 = wx1 * wy1, w10 = wx * wy1;
      const float w01 = wx1 * wy,  w11 = wx * wy;

      // BGRA → RGB  (unrolled for auto-vectorisation)
      const uint8_t* p00 = r0 + x0 * 4;
      const uint8_t* p10 = r0 + x1 * 4;
      const uint8_t* p01 = r1 + x0 * 4;
      const uint8_t* p11 = r1 + x1 * 4;

      const float rv = p00[2]*w00 + p10[2]*w10 + p01[2]*w01 + p11[2]*w11;
      const float gv = p00[1]*w00 + p10[1]*w10 + p01[1]*w01 + p11[1]*w11;
      const float bv = p00[0]*w00 + p10[0]*w10 + p01[0]*w01 + p11[0]*w11;
      dst[x * 3 + 0] = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, rv + 0.5f)));
      dst[x * 3 + 1] = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, gv + 0.5f)));
      dst[x * 3 + 2] = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, bv + 0.5f)));
    }
  }
}

// ---------------------------------------------------------------------------
// Bilinear scale: RGB 720×480 → BGRA output (any size)
// ---------------------------------------------------------------------------
void ScaleNtscToOutput(const uint8_t* rgb, uint8_t* bgra, int dstW, int dstH,
                       int rowBytes) {
  if (dstW <= 0 || dstH <= 0) return;

  const float sx = (dstW > 1) ? (kNtscW - 1.0f) / (dstW - 1.0f) : 0.0f;
  const float sy = (dstH > 1) ? (kNtscH - 1.0f) / (dstH - 1.0f) : 0.0f;

  for (int y = 0; y < dstH; ++y) {
    const float fy = y * sy;
    const int y0 = static_cast<int>(fy);
    const int y1 = std::min(y0 + 1, kNtscH - 1);
    const float wy = fy - y0;
    const float wy1 = 1.0f - wy;

    const uint8_t* r0 = rgb + y0 * kNtscW * 3;
    const uint8_t* r1 = rgb + y1 * kNtscW * 3;
    uint8_t* dst = bgra + y * rowBytes;

    for (int x = 0; x < dstW; ++x) {
      const float fx = x * sx;
      const int x0 = static_cast<int>(fx);
      const int x1 = std::min(x0 + 1, kNtscW - 1);
      const float wx = fx - x0;
      const float wx1 = 1.0f - wx;

      const float w00 = wx1 * wy1, w10 = wx * wy1;
      const float w01 = wx1 * wy,  w11 = wx * wy;

      // RGB → BGRA  (unrolled for auto-vectorisation)
      const float bv = r0[x0*3+2]*w00 + r0[x1*3+2]*w10 + r1[x0*3+2]*w01 + r1[x1*3+2]*w11;
      const float gv = r0[x0*3+1]*w00 + r0[x1*3+1]*w10 + r1[x0*3+1]*w01 + r1[x1*3+1]*w11;
      const float rv = r0[x0*3+0]*w00 + r0[x1*3+0]*w10 + r1[x0*3+0]*w01 + r1[x1*3+0]*w11;
      dst[x*4+0] = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, bv + 0.5f)));
      dst[x*4+1] = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, gv + 0.5f)));
      dst[x*4+2] = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, rv + 0.5f)));
      dst[x * 4 + 3] = 255;  // opaque alpha
    }
  }
}

// ---------------------------------------------------------------------------
// Preset application: writes values into params[] and sets change flags.
// ---------------------------------------------------------------------------
void SetF(PF_ParamDef* params[], int idx, float v) {
  params[idx]->u.fs_d.value = v;
  params[idx]->uu.change_flags |= PF_ChangeFlag_CHANGED_VALUE;
}
void SetB(PF_ParamDef* params[], int idx, bool v) {
  params[idx]->u.bd.value = v ? 1 : 0;
  params[idx]->uu.change_flags |= PF_ChangeFlag_CHANGED_VALUE;
}

void ApplyPreset(PF_ParamDef* params[], int preset) {
  if (preset == RF2_PRESET_CUSTOM) return;

  // Start from clean defaults, then override per-preset.
  // -- Decode --
  SetF(params, RF2_BRIGHTNESS, 0.0f);
  SetF(params, RF2_CONTRAST,   1.0f);
  SetF(params, RF2_SATURATION, 1.0f);
  SetF(params, RF2_TINT,       0.0f);
  SetF(params, RF2_SHARPNESS,  0.0f);
  SetB(params, RF2_COMB_FILTER, true);
  SetB(params, RF2_DOT_CRAWL,   false);
  SetF(params, RF2_CHROMA_DELAY,    0.0f);
  SetF(params, RF2_OVERSCAN_REVEAL, 0.0f);
  SetF(params, RF2_H_LOCK,     0.0f);
  SetF(params, RF2_V_HOLD,     0.0f);
  SetF(params, RF2_BURST_LOCK, 0.0f);
  // -- Effects --
  SetF(params, RF2_NOISE,          0.0f);
  SetF(params, RF2_GHOST_GAIN,     0.0f);
  SetF(params, RF2_GHOST_DELAY,   24.0f);
  SetF(params, RF2_GHOST_ENSEMBLE, 0.0f);
  SetF(params, RF2_LINE_JITTER,    0.0f);
  SetF(params, RF2_AFC_HUNT,       0.0f);
  SetF(params, RF2_RF_DRIFT,       0.0f);
  SetF(params, RF2_AM_NONLINEARITY,0.0f);
  SetF(params, RF2_IMPULSE_NOISE,  0.0f);
  SetF(params, RF2_AGC_PUMP,       0.0f);
  SetF(params, RF2_HUM,            0.0f);
  SetF(params, RF2_CHROMA_FLUTTER, 0.0f);
  SetF(params, RF2_YC_CROSSTALK,   0.0f);
  SetF(params, RF2_H_SYNC_NOISE,   0.0f);
  SetF(params, RF2_V_SYNC_NOISE,   0.0f);
  SetF(params, RF2_BURST_NOISE,    0.0f);
  SetF(params, RF2_VHS_TRACKING,   0.0f);
  SetF(params, RF2_VHS_WRINKLE,    0.0f);
  SetF(params, RF2_VHS_HEAD_SWITCH,0.0f);
  SetF(params, RF2_VHS_DROPOUTS,   0.0f);
  SetF(params, RF2_NOISE_COLOR,    0.0f);
  SetF(params, RF2_GROUP_DELAY,    0.0f);
  // -- Encoder --
  SetF(params, RF2_LUMA_CUTOFF,     4.2f);
  SetF(params, RF2_I_CUTOFF,        1.1f);
  SetF(params, RF2_Q_CUTOFF,        0.45f);
  SetF(params, RF2_CHROMA_MOD_SCALE,1.0f);
  SetF(params, RF2_FILTER_PASSES,   3.0f);
  SetB(params, RF2_LETTERBOX_TO_43, true);

  switch (preset) {
    case RF2_PRESET_CLEAN:
      // Pure NTSC encode/decode, no effects.
      // Shows only NTSC's inherent bandwidth and color resolution limits.
      break;

    case RF2_PRESET_GOOD_RF:
      // Close station, rooftop antenna. Barely-visible degradation.
      SetF(params, RF2_CHROMA_MOD_SCALE,0.88f);
      SetF(params, RF2_NOISE,           1.0f);
      SetF(params, RF2_NOISE_COLOR,     0.25f);
      SetF(params, RF2_GHOST_GAIN,      0.05f);
      SetF(params, RF2_GHOST_DELAY,    24.0f);
      SetF(params, RF2_AM_NONLINEARITY, 0.02f);
      SetF(params, RF2_YC_CROSSTALK,    0.03f);
      SetF(params, RF2_H_SYNC_NOISE,    0.2f);
      SetF(params, RF2_V_SYNC_NOISE,    0.2f);
      SetF(params, RF2_BURST_NOISE,     0.3f);
      SetF(params, RF2_GROUP_DELAY,     0.05f);
      break;

    case RF2_PRESET_AVERAGE_RF:
      // Indoor antenna, suburban area. Visible noise, some ghosting.
      SetF(params, RF2_CHROMA_MOD_SCALE,0.88f);
      SetF(params, RF2_NOISE,           2.5f);
      SetF(params, RF2_NOISE_COLOR,     0.35f);
      SetF(params, RF2_GHOST_GAIN,      0.12f);
      SetF(params, RF2_GHOST_DELAY,    28.0f);
      SetF(params, RF2_GHOST_ENSEMBLE,  0.15f);
      SetF(params, RF2_LINE_JITTER,     0.15f);
      SetF(params, RF2_AFC_HUNT,        0.08f);
      SetF(params, RF2_RF_DRIFT,        0.05f);
      SetF(params, RF2_AM_NONLINEARITY, 0.06f);
      SetF(params, RF2_AGC_PUMP,        0.03f);
      SetF(params, RF2_HUM,             0.04f);
      SetF(params, RF2_CHROMA_FLUTTER,  0.04f);
      SetF(params, RF2_YC_CROSSTALK,    0.08f);
      SetF(params, RF2_H_SYNC_NOISE,    1.0f);
      SetF(params, RF2_V_SYNC_NOISE,    0.6f);
      SetF(params, RF2_BURST_NOISE,     1.0f);
      SetF(params, RF2_GROUP_DELAY,     0.15f);
      break;

    case RF2_PRESET_BAD_RF:
      // Rabbit ears, fringe area. Heavy snow, still watchable.
      SetF(params, RF2_CHROMA_MOD_SCALE,0.88f);
      SetF(params, RF2_NOISE,           5.0f);
      SetF(params, RF2_NOISE_COLOR,     0.50f);
      SetF(params, RF2_GHOST_GAIN,      0.25f);
      SetF(params, RF2_GHOST_DELAY,    32.0f);
      SetF(params, RF2_GHOST_ENSEMBLE,  0.30f);
      SetF(params, RF2_LINE_JITTER,     0.45f);
      SetF(params, RF2_AFC_HUNT,        0.15f);
      SetF(params, RF2_RF_DRIFT,        0.15f);
      SetF(params, RF2_AM_NONLINEARITY, 0.12f);
      SetF(params, RF2_IMPULSE_NOISE,   0.10f);
      SetF(params, RF2_AGC_PUMP,        0.08f);
      SetF(params, RF2_HUM,             0.08f);
      SetF(params, RF2_CHROMA_FLUTTER,  0.08f);
      SetF(params, RF2_YC_CROSSTALK,    0.12f);
      SetF(params, RF2_H_SYNC_NOISE,    3.0f);
      SetF(params, RF2_V_SYNC_NOISE,    2.0f);
      SetF(params, RF2_BURST_NOISE,     3.5f);
      SetF(params, RF2_H_LOCK,          0.10f);
      SetF(params, RF2_V_HOLD,          0.06f);
      SetF(params, RF2_BURST_LOCK,      0.10f);
      SetF(params, RF2_GROUP_DELAY,     0.25f);
      break;

    case RF2_PRESET_GOOD_VHS:
      // Fresh tape, good deck. VHS feel, no visible noise.
      SetF(params, RF2_LUMA_CUTOFF,     3.0f);
      SetF(params, RF2_I_CUTOFF,        0.5f);
      SetF(params, RF2_Q_CUTOFF,        0.3f);
      SetF(params, RF2_FILTER_PASSES,   2.0f);
      SetF(params, RF2_CHROMA_MOD_SCALE,0.80f);
      SetF(params, RF2_SATURATION,      0.95f);
      SetF(params, RF2_TINT,            0.5f);
      SetF(params, RF2_CHROMA_DELAY,    0.3f);
      SetF(params, RF2_NOISE,           0.8f);
      SetF(params, RF2_NOISE_COLOR,     0.35f);
      SetF(params, RF2_LINE_JITTER,     0.05f);
      SetF(params, RF2_AM_NONLINEARITY, 0.02f);
      SetF(params, RF2_CHROMA_FLUTTER,  0.03f);
      SetF(params, RF2_YC_CROSSTALK,    0.05f);
      SetF(params, RF2_H_SYNC_NOISE,    0.3f);
      SetF(params, RF2_BURST_NOISE,     0.4f);
      SetF(params, RF2_BURST_LOCK,      0.02f);
      SetF(params, RF2_VHS_HEAD_SWITCH, 0.30f);
      SetF(params, RF2_GROUP_DELAY,     0.08f);
      break;

    case RF2_PRESET_DIRTY_HEAD_VHS:
      // Oxide deposits on heads. Dropouts and noise, tape undamaged.
      SetF(params, RF2_LUMA_CUTOFF,     3.0f);
      SetF(params, RF2_I_CUTOFF,        0.5f);
      SetF(params, RF2_Q_CUTOFF,        0.3f);
      SetF(params, RF2_FILTER_PASSES,   2.0f);
      SetF(params, RF2_CHROMA_MOD_SCALE,0.80f);
      SetF(params, RF2_BRIGHTNESS,      0.01f);
      SetF(params, RF2_CONTRAST,        0.97f);
      SetF(params, RF2_SATURATION,      0.88f);
      SetF(params, RF2_TINT,            1.0f);
      SetF(params, RF2_CHROMA_DELAY,    0.5f);
      SetF(params, RF2_NOISE,           2.0f);
      SetF(params, RF2_NOISE_COLOR,     0.45f);
      SetF(params, RF2_LINE_JITTER,     0.10f);
      SetF(params, RF2_AM_NONLINEARITY, 0.03f);
      SetF(params, RF2_AGC_PUMP,        0.02f);
      SetF(params, RF2_HUM,             0.02f);
      SetF(params, RF2_CHROMA_FLUTTER,  0.08f);
      SetF(params, RF2_YC_CROSSTALK,    0.10f);
      SetF(params, RF2_H_SYNC_NOISE,    0.8f);
      SetF(params, RF2_V_SYNC_NOISE,    0.4f);
      SetF(params, RF2_BURST_NOISE,     1.5f);
      SetF(params, RF2_BURST_LOCK,      0.05f);
      SetF(params, RF2_VHS_HEAD_SWITCH, 0.50f);
      SetF(params, RF2_VHS_DROPOUTS,    0.15f);
      SetF(params, RF2_GROUP_DELAY,     0.15f);
      break;

    case RF2_PRESET_DAMAGED_TAPE_VHS:
      // Old tape, physical damage, worn edges, oxide deterioration.
      SetF(params, RF2_LUMA_CUTOFF,     2.8f);
      SetF(params, RF2_I_CUTOFF,        0.45f);
      SetF(params, RF2_Q_CUTOFF,        0.28f);
      SetF(params, RF2_FILTER_PASSES,   2.0f);
      SetF(params, RF2_CHROMA_MOD_SCALE,0.78f);
      SetF(params, RF2_BRIGHTNESS,      0.02f);
      SetF(params, RF2_CONTRAST,        0.92f);
      SetF(params, RF2_SATURATION,      0.75f);
      SetF(params, RF2_TINT,            2.0f);
      SetF(params, RF2_CHROMA_DELAY,    1.0f);
      SetF(params, RF2_H_LOCK,          0.08f);
      SetF(params, RF2_V_HOLD,          0.06f);
      SetF(params, RF2_BURST_LOCK,      0.15f);
      SetF(params, RF2_NOISE,           4.0f);
      SetF(params, RF2_NOISE_COLOR,     0.55f);
      SetF(params, RF2_LINE_JITTER,     0.30f);
      SetF(params, RF2_AM_NONLINEARITY, 0.08f);
      SetF(params, RF2_IMPULSE_NOISE,   0.04f);
      SetF(params, RF2_AGC_PUMP,        0.06f);
      SetF(params, RF2_HUM,             0.04f);
      SetF(params, RF2_CHROMA_FLUTTER,  0.20f);
      SetF(params, RF2_YC_CROSSTALK,    0.18f);
      SetF(params, RF2_H_SYNC_NOISE,    2.5f);
      SetF(params, RF2_V_SYNC_NOISE,    1.5f);
      SetF(params, RF2_BURST_NOISE,     3.5f);
      SetF(params, RF2_VHS_TRACKING,    0.20f);
      SetF(params, RF2_VHS_WRINKLE,     0.20f);
      SetF(params, RF2_VHS_HEAD_SWITCH, 0.60f);
      SetF(params, RF2_VHS_DROPOUTS,    0.25f);
      SetF(params, RF2_GROUP_DELAY,     0.25f);
      break;
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// PF_Cmd_GLOBAL_SETUP
// ---------------------------------------------------------------------------
static PF_Err GlobalSetup(PF_InData* in_data, PF_OutData* out_data,
                          PF_ParamDef* params[], PF_LayerDef* output) {
  out_data->my_version =
      PF_VERSION(RF2_MAJOR_VERSION, RF2_MINOR_VERSION, RF2_BUG_VERSION,
                 RF2_STAGE_VERSION, RF2_BUILD_VERSION);

  if (in_data->appl_id == 'PrMr') {
    AEFX_SuiteScoper<PF_PixelFormatSuite1> pfs(
        in_data, kPFPixelFormatSuite, kPFPixelFormatSuiteVersion1, out_data);
    pfs->ClearSupportedPixelFormats(in_data->effect_ref);
    pfs->AddSupportedPixelFormat(in_data->effect_ref,
                                 PrPixelFormat_BGRA_4444_8u);
  }

  out_data->out_flags2 |= PF_OutFlag2_PRESERVES_FULLY_OPAQUE_PIXELS;

  return PF_Err_NONE;
}

// ---------------------------------------------------------------------------
// Helper: add a float slider with our standard flags
// ---------------------------------------------------------------------------
#define ADD_SLIDER(NAME, VMIN, VMAX, SMIN, SMAX, DFLT, PREC, ID) \
  do {                                                            \
    AEFX_CLR_STRUCT(def);                                         \
    PF_ADD_FLOAT_SLIDERX(NAME, VMIN, VMAX, SMIN, SMAX, DFLT,    \
                          PREC, PF_ValueDisplayFlag_NONE, 0, ID); \
  } while (0)

#define ADD_CHECK(NAME, COMMENT, DFLT, ID) \
  do {                                     \
    AEFX_CLR_STRUCT(def);                  \
    PF_ADD_CHECKBOX(NAME, COMMENT, DFLT, 0, ID); \
  } while (0)

// ---------------------------------------------------------------------------
// PF_Cmd_PARAMS_SETUP
// ---------------------------------------------------------------------------
static PF_Err ParamsSetup(PF_InData* in_data, PF_OutData* out_data,
                          PF_ParamDef* params[], PF_LayerDef* output) {
  PF_ParamDef def;

  // -- Preset selector -------------------------------------------------------
  AEFX_CLR_STRUCT(def);
  def.flags = PF_ParamFlag_SUPERVISE;
  PF_ADD_POPUP("Preset", RF2_NUM_PRESETS, RF2_PRESET_CUSTOM,
               "Custom"
               "|Clean"
               "|Good RF"
               "|Average RF"
               "|Bad RF"
               "|Good VHS"
               "|Dirty Head VHS"
               "|Damaged Tape VHS",
               RF2_PRESET);
  // -- Decode controls -------------------------------------------------------
  ADD_SLIDER("Brightness",  -0.5,  0.5, -0.5,  0.5,  0.0,
             PF_Precision_HUNDREDTHS, RF2_BRIGHTNESS);
  ADD_SLIDER("Contrast",     0.2,  2.0,  0.2,  2.0,  1.0,
             PF_Precision_HUNDREDTHS, RF2_CONTRAST);
  ADD_SLIDER("Saturation",   0.0,  2.0,  0.0,  2.0,  0.9,
             PF_Precision_HUNDREDTHS, RF2_SATURATION);
  ADD_SLIDER("Tint",       -45.0, 45.0,-45.0, 45.0,  0.0,
             PF_Precision_TENTHS, RF2_TINT);
  ADD_SLIDER("Sharpness",    0.0,  1.0,  0.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_SHARPNESS);
  ADD_CHECK("Comb Filter", "", TRUE,  RF2_COMB_FILTER);
  ADD_CHECK("Dot Crawl",   "", FALSE, RF2_DOT_CRAWL);
  ADD_SLIDER("Chroma Delay", -6.0, 6.0, -6.0, 6.0, 0.0,
             PF_Precision_TENTHS, RF2_CHROMA_DELAY);
  ADD_SLIDER("Overscan Reveal", 0.0, 0.25, 0.0, 0.25, 0.0,
             PF_Precision_HUNDREDTHS, RF2_OVERSCAN_REVEAL);
  ADD_SLIDER("H Lock Instability", 0.0, 1.0, 0.0, 1.0, 0.0,
             PF_Precision_HUNDREDTHS, RF2_H_LOCK);
  ADD_SLIDER("V Hold Instability", 0.0, 1.0, 0.0, 1.0, 0.0,
             PF_Precision_HUNDREDTHS, RF2_V_HOLD);
  ADD_SLIDER("Burst Lock Instability", 0.0, 1.0, 0.0, 1.0, 0.0,
             PF_Precision_HUNDREDTHS, RF2_BURST_LOCK);

  // -- Signal effects --------------------------------------------------------
  ADD_SLIDER("Noise",           0.0, 10.0,  0.0, 10.0,  0.0,
             PF_Precision_TENTHS, RF2_NOISE);
  ADD_SLIDER("Ghost Gain",     0.0,  1.0,  0.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_GHOST_GAIN);
  ADD_SLIDER("Ghost Delay",    1.0, 200.0, 1.0, 200.0, 24.0,
             PF_Precision_INTEGER, RF2_GHOST_DELAY);
  ADD_SLIDER("Ghost Ensemble",  0.0,  1.0,  0.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_GHOST_ENSEMBLE);
  ADD_SLIDER("Line Jitter",    0.0,  2.0,  0.0,  2.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_LINE_JITTER);
  ADD_SLIDER("AFC Hunt",       0.0,  1.0,  0.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_AFC_HUNT);
  ADD_SLIDER("RF Drift",       0.0,  1.0,  0.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_RF_DRIFT);
  ADD_SLIDER("AM Nonlinearity", 0.0, 1.0,  0.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_AM_NONLINEARITY);
  ADD_SLIDER("Impulse Noise",  0.0,  1.0,  0.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_IMPULSE_NOISE);
  ADD_SLIDER("AGC Pump",       0.0,  1.0,  0.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_AGC_PUMP);
  ADD_SLIDER("Hum",            0.0,  1.0,  0.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_HUM);
  ADD_SLIDER("Chroma Flutter", 0.0,  1.0,  0.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_CHROMA_FLUTTER);
  ADD_SLIDER("Y/C Crosstalk",  0.0,  1.0,  0.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_YC_CROSSTALK);
  ADD_SLIDER("H Sync Noise",   0.0, 16.0,  0.0, 16.0,  0.0,
             PF_Precision_TENTHS, RF2_H_SYNC_NOISE);
  ADD_SLIDER("V Sync Noise",   0.0, 16.0,  0.0, 16.0,  0.0,
             PF_Precision_TENTHS, RF2_V_SYNC_NOISE);
  ADD_SLIDER("Burst Noise",    0.0, 16.0,  0.0, 16.0,  0.0,
             PF_Precision_TENTHS, RF2_BURST_NOISE);
  ADD_SLIDER("VHS Tracking",  -1.0,  1.0, -1.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_VHS_TRACKING);
  ADD_SLIDER("VHS Wrinkle",    0.0,  1.0,  0.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_VHS_WRINKLE);
  ADD_SLIDER("VHS Head Switch", 0.0, 1.0,  0.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_VHS_HEAD_SWITCH);
  ADD_SLIDER("VHS Dropouts",   0.0,  1.0,  0.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_VHS_DROPOUTS);
  ADD_SLIDER("Noise Color",    0.0,  1.0,  0.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_NOISE_COLOR);
  ADD_SLIDER("Group Delay",    0.0,  1.0,  0.0,  1.0,  0.0,
             PF_Precision_HUNDREDTHS, RF2_GROUP_DELAY);

  // -- Encoder controls ------------------------------------------------------
  ADD_SLIDER("Luma Cutoff MHz",     0.5, 6.0, 0.5, 6.0, 4.2,
             PF_Precision_TENTHS, RF2_LUMA_CUTOFF);
  ADD_SLIDER("I Cutoff MHz",        0.2, 1.5, 0.2, 1.5, 1.1,
             PF_Precision_TENTHS, RF2_I_CUTOFF);
  ADD_SLIDER("Q Cutoff MHz",        0.1, 0.6, 0.1, 0.6, 0.45,
             PF_Precision_HUNDREDTHS, RF2_Q_CUTOFF);
  ADD_SLIDER("Chroma Mod Scale",    0.5, 1.5, 0.5, 1.5, 0.88,
             PF_Precision_HUNDREDTHS, RF2_CHROMA_MOD_SCALE);
  ADD_SLIDER("Filter Passes",       1.0, 6.0, 1.0, 6.0, 3.0,
             PF_Precision_INTEGER, RF2_FILTER_PASSES);
  ADD_CHECK("Letterbox to 4:3", "", TRUE, RF2_LETTERBOX_TO_43);

  out_data->num_params = RF2_NUM_PARAMS;
  return PF_Err_NONE;
}

#undef ADD_SLIDER
#undef ADD_CHECK

// ---------------------------------------------------------------------------
// PF_Cmd_RENDER  (Premiere-only, BGRA_4444_8u)
// ---------------------------------------------------------------------------
static PF_Err Render(PF_InData* in_data, PF_OutData* out_data,
                     PF_ParamDef* params[], PF_LayerDef* output) {
  if (in_data->appl_id != 'PrMr') return PF_Err_NONE;

  PF_LayerDef* src = &params[RF2_INPUT]->u.ld;
  PF_LayerDef* dst = output;

  // ------- frame number for temporal stability ------------------------------
  A_long frameNumber = 0;
  // Compute from in_data time fields (always available, no suite needed).
  if (in_data->time_step > 0) {
    frameNumber = static_cast<A_long>(in_data->current_time / in_data->time_step);
  }
  const uint32_t seed =
      static_cast<uint32_t>(std::abs(frameNumber)) + 1;

  // ------- reuse NTSC processor across frames (avoid per-frame heap alloc) --
  thread_local rf2::NtscProcessor proc;

  proc.encoder().set_frame_index(
      static_cast<uint32_t>(std::abs(frameNumber)));

  // Deterministic seeds derived from frame number.
  proc.effects().controls().random_seed = seed;
  proc.decoder().controls().random_seed = seed;

  // ------- read decode sliders ----------------------------------------------
  auto& dec = proc.decoder().controls();
  dec.brightness          = params[RF2_BRIGHTNESS]->u.fs_d.value;
  dec.contrast            = params[RF2_CONTRAST]->u.fs_d.value;
  dec.saturation          = params[RF2_SATURATION]->u.fs_d.value;
  dec.tint_degrees        = params[RF2_TINT]->u.fs_d.value;
  dec.sharpness           = params[RF2_SHARPNESS]->u.fs_d.value;
  dec.comb_filter         = params[RF2_COMB_FILTER]->u.bd.value != 0;
  dec.dot_crawl           = params[RF2_DOT_CRAWL]->u.bd.value != 0;
  dec.chroma_delay_pixels = params[RF2_CHROMA_DELAY]->u.fs_d.value;
  dec.overscan_reveal     = params[RF2_OVERSCAN_REVEAL]->u.fs_d.value;
  dec.h_lock_instability  = params[RF2_H_LOCK]->u.fs_d.value;
  dec.v_hold_instability  = params[RF2_V_HOLD]->u.fs_d.value;
  dec.burst_lock_instability = params[RF2_BURST_LOCK]->u.fs_d.value;

  // ------- read effects sliders ---------------------------------------------
  auto& fx = proc.effects().controls();
  fx.noise_stddev_ire        = params[RF2_NOISE]->u.fs_d.value;
  fx.multipath_gain          = params[RF2_GHOST_GAIN]->u.fs_d.value;
  fx.multipath_delay_samples =
      static_cast<uint32_t>(params[RF2_GHOST_DELAY]->u.fs_d.value + 0.5f);
  fx.multipath_ensemble      = params[RF2_GHOST_ENSEMBLE]->u.fs_d.value;
  fx.line_time_jitter_samples = params[RF2_LINE_JITTER]->u.fs_d.value;
  fx.afc_hunt                = params[RF2_AFC_HUNT]->u.fs_d.value;
  fx.rf_drift                = params[RF2_RF_DRIFT]->u.fs_d.value;
  fx.am_nonlinearity         = params[RF2_AM_NONLINEARITY]->u.fs_d.value;
  fx.impulse_noise           = params[RF2_IMPULSE_NOISE]->u.fs_d.value;
  fx.agc_pump                = params[RF2_AGC_PUMP]->u.fs_d.value;
  fx.hum                     = params[RF2_HUM]->u.fs_d.value;
  fx.chroma_flutter          = params[RF2_CHROMA_FLUTTER]->u.fs_d.value;
  fx.yc_crosstalk            = params[RF2_YC_CROSSTALK]->u.fs_d.value;
  fx.h_sync_noise_ire        = params[RF2_H_SYNC_NOISE]->u.fs_d.value;
  fx.v_sync_noise_ire        = params[RF2_V_SYNC_NOISE]->u.fs_d.value;
  fx.burst_noise_ire         = params[RF2_BURST_NOISE]->u.fs_d.value;
  fx.vhs_tracking            = params[RF2_VHS_TRACKING]->u.fs_d.value;
  fx.vhs_wrinkle             = params[RF2_VHS_WRINKLE]->u.fs_d.value;
  fx.vhs_head_switch         = params[RF2_VHS_HEAD_SWITCH]->u.fs_d.value;
  fx.vhs_dropouts            = params[RF2_VHS_DROPOUTS]->u.fs_d.value;
  fx.noise_color             = params[RF2_NOISE_COLOR]->u.fs_d.value;
  fx.group_delay             = params[RF2_GROUP_DELAY]->u.fs_d.value;

  // ------- read encoder sliders (MHz → Hz) ----------------------------------
  auto& enc = proc.encoder().config();
  enc.luma_cutoff_hz  = params[RF2_LUMA_CUTOFF]->u.fs_d.value * 1e6f;
  enc.i_cutoff_hz     = params[RF2_I_CUTOFF]->u.fs_d.value * 1e6f;
  enc.q_cutoff_hz     = params[RF2_Q_CUTOFF]->u.fs_d.value * 1e6f;
  enc.chroma_mod_scale = params[RF2_CHROMA_MOD_SCALE]->u.fs_d.value;
  enc.filter_passes   =
      static_cast<uint32_t>(params[RF2_FILTER_PASSES]->u.fs_d.value + 0.5f);

  // ------- scale input to NTSC 720×480 RGB ----------------------------------
  thread_local std::vector<uint8_t> rgb_in;
  thread_local std::vector<uint8_t> rgb_out;
  rgb_in.resize(kNtscW * kNtscH * 3);  // no-op after first frame
  const bool letterbox_to_43 = params[RF2_LETTERBOX_TO_43]->u.bd.value != 0;
  const float in_par = PixelAspect(in_data->pixel_aspect_ratio);
  float src_aspect =
      (in_data->width > 0 && in_data->height > 0)
          ? (static_cast<float>(in_data->width) * in_par /
             static_cast<float>(in_data->height))
          : 0.0f;
  if (!(src_aspect > 0.25f && src_aspect < 4.0f)) {
    const float src_par = PixelAspect(src->pix_aspect_ratio);
    src_aspect = (src->width > 0 && src->height > 0)
                     ? (static_cast<float>(src->width) * src_par /
                        static_cast<float>(src->height))
                     : (4.0f / 3.0f);
  }
  ScaleInputToNtsc(reinterpret_cast<const uint8_t*>(src->data), src->width,
                   src->height, src->rowbytes, letterbox_to_43, src_aspect,
                   rgb_in.data());

  // ------- run NTSC encode → effects → decode pipeline ----------------------
  proc.ProcessFrame(rgb_in.data(), &rgb_out);

  // ------- scale output back to destination resolution ----------------------
  ScaleNtscToOutput(rgb_out.data(), reinterpret_cast<uint8_t*>(dst->data),
                    dst->width, dst->height, dst->rowbytes);

  return PF_Err_NONE;
}

// ---------------------------------------------------------------------------
// PF_Cmd_USER_CHANGED_PARAM  —  apply preset when dropdown changes
// ---------------------------------------------------------------------------
static PF_Err UserChangedParam(PF_InData* in_data, PF_OutData* out_data,
                                PF_ParamDef* params[], PF_LayerDef* output,
                                void* extra) {
  auto* ucp = reinterpret_cast<PF_UserChangedParamExtra*>(extra);
  if (ucp && ucp->param_index == RF2_PRESET) {
    const int preset = static_cast<int>(params[RF2_PRESET]->u.pd.value);
    ApplyPreset(params, preset);
    out_data->out_flags |= PF_OutFlag_FORCE_RERENDER;
  }
  return PF_Err_NONE;
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
#if defined(MSWindows) || defined(_WIN32)
#define DllExport __declspec(dllexport)
#else
#define DllExport __attribute__((visibility("default")))
#endif

extern "C" DllExport PF_Err EffectMain(PF_Cmd cmd, PF_InData* in_data,
                                       PF_OutData* out_data,
                                       PF_ParamDef* params[],
                                       PF_LayerDef* output, void* extra) {
  PF_Err err = PF_Err_NONE;
  try {
    switch (cmd) {
      case PF_Cmd_GLOBAL_SETUP:
        err = GlobalSetup(in_data, out_data, params, output);
        break;
      case PF_Cmd_PARAMS_SETUP:
        err = ParamsSetup(in_data, out_data, params, output);
        break;
      case PF_Cmd_RENDER:
        err = Render(in_data, out_data, params, output);
        break;
      case PF_Cmd_USER_CHANGED_PARAM:
        err = UserChangedParam(in_data, out_data, params, output, extra);
        break;
    }
  } catch (PF_Err& thrown_err) {
    err = thrown_err;
  }
  return err;
}
