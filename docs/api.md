# RF2 NTSC Library — Developer Guide

RF2 is a C++20 library that encodes RGB video into a realistic NTSC composite signal, optionally degrades it with analog artifacts, and decodes it back to RGB. The pipeline is split into independent stages so you can embed any combination of them in your own host application, plugin, or tool.

## Headers at a Glance

| Header | What it provides |
|--------|-----------------|
| `rf2/ntsc_signal.h` | Core types (`NtscSignalConfig`, `NtscTiming`, etc.), `IreToSample`/`SampleToIre`, `EncodeNtscCompositeFrame`, `EncodeNtscCompositeFrameFloat` |
| `rf2/ntsc_encoder.h` | `NtscEncoder` — stateful encoder class |
| `rf2/ntsc_effects.h` | `NtscEffectControls`, `NtscEffects` — signal degradation |
| `rf2/ntsc_decoder.h` | `NtscDecodeControls`, `NtscDecoder` — composite-to-RGB decoder |
| `rf2/ntsc_processor.h` | `NtscProcessor` — all-in-one encode→effects→decode |
| `rf2/ntsc_params.h` | `ParamId` enum, `ParamMeta`, parameter registry |
| `rf2/ntrf_container.h` | `NtrfWriter`/`NtrfReader` — NTRF container I/O |

Include only what you need. Every header is self-contained.

---

## Quick Start

### Simplest case: RGB in, degraded RGB out

```cpp
#include "rf2/ntsc_processor.h"

// Input: 720x480 packed RGB888 (1,036,800 bytes).
const uint8_t* rgb_in = /* your frame */;
std::vector<uint8_t> rgb_out;

rf2::NtscProcessor proc;

// Dial in some analog ugliness.
proc.SetParam(rf2::ParamId::kNoiseStddevIre, 3.0f);
proc.SetParam(rf2::ParamId::kMultipathGain, 0.15f);
proc.SetParam(rf2::ParamId::kVhsTracking, 0.4f);

// Process frame. Call once per frame in sequence.
proc.ProcessFrame(rgb_in, &rgb_out);
// rgb_out is 720x480 packed RGB888.
```

`NtscProcessor` manages the full pipeline internally: encode → apply effects → decode. It auto-increments the frame counter so subcarrier phase is correct across frames.

### Manual pipeline (decode-only, or effects-only)

When you already have composite data (e.g. from an NTRF file) or need finer control, use the individual stages directly.

```cpp
#include "rf2/ntsc_effects.h"
#include "rf2/ntsc_decoder.h"
#include "rf2/ntsc_signal.h"  // for SampleToIre

rf2::NtscSignalConfig signal;  // default NTSC timing
rf2::NtscEffects effects(signal);
rf2::NtscDecoder decoder(signal);

// Configure effects.
effects.controls().noise_stddev_ire = 2.0f;
effects.controls().h_sync_noise_ire = 4.0f;

// Configure decoder.
decoder.controls().saturation = 1.1f;
decoder.controls().h_lock_instability = 0.3f;

// Given a composite signal as int16 samples (e.g. from an NTRF frame):
const std::vector<int16_t>& composite = frame.composite;

// Step 1: Convert int16 samples to IRE floats.
std::vector<float> ire(composite.size());
for (size_t i = 0; i < composite.size(); ++i) {
    ire[i] = rf2::SampleToIre(composite[i]);
}

// Step 2: Apply signal degradation (mutates ire in-place).
effects.controls().random_seed = frame_index + 1;
effects.Apply(&ire);

// Step 3: Decode to RGB.
std::vector<uint8_t> rgb;
decoder.controls().random_seed = frame_index + 1;
decoder.Decode(ire, &rgb);
// rgb is 720x480 packed RGB888.
```

### Encode-only

```cpp
#include "rf2/ntsc_encoder.h"

rf2::NtscEncoder encoder;
std::vector<int16_t> composite;

// Encode sequential frames — frame counter advances automatically.
encoder.EncodeFrame(rgb_frame_0, &composite);
encoder.EncodeFrame(rgb_frame_1, &composite);

// Or encode at a specific frame index (for random access):
encoder.EncodeFrameAt(rgb, 42, &composite);
```

---

## Signal Flow

```
RGB 720x480        int16 composite        float IRE            float IRE           RGB 720x480
     │                   │                    │                    │                    │
     ▼                   │                    │                    │                    ▼
 NtscEncoder ──────►  samples ──► SampleToIre ──► NtscEffects ──► NtscDecoder ──►  pixels
                    (525 × 910)              (corrupt signal)    (recover picture)
```

The composite signal is 525 lines × 910 samples per line = 477,750 int16 samples per frame, sampled at 4×fsc (≈14.318 MHz).

IRE floats use the same scale: blanking = 0, sync tip = −40, white = 100. The int16 encoding is `sample = round(ire × 256)`.

---

## Core Types

### `NtscSignalConfig`

Top-level configuration bundle passed to encoder, effects, and decoder. Default-constructed values produce standard NTSC timing.

```cpp
struct NtscSignalConfig {
    NtscTiming timing;       // line/sample counts
    NtscLevels levels;       // IRE voltage levels
    NtscLineMap line_map;    // visible line mapping
    float luma_cutoff_hz;    // encoder luma bandwidth  (default 4.2 MHz)
    float i_cutoff_hz;       // encoder I bandwidth     (default 1.1 MHz)
    float q_cutoff_hz;       // encoder Q bandwidth     (default 450 kHz)
    float chroma_mod_scale;  // chroma modulation gain  (default 0.88)
    uint32_t filter_passes;  // encoder filter steepness (default 3)
};
```

**`NtscTiming`** — sample-level timing at 4×fsc:

| Field | Default | Description |
|-------|---------|-------------|
| `lines_per_frame` | 525 | Total scanlines per frame |
| `samples_per_line` | 910 | Samples per scanline |
| `hsync_samples` | 67 | Horizontal sync pulse width |
| `back_porch_samples` | 75 | Back porch duration |
| `active_start` | 142 | First active picture sample |
| `active_samples` | 754 | Active picture width |
| `burst_start` | 80 | Color burst start offset |
| `burst_samples` | 36 | Color burst duration (9 cycles) |

**`NtscLevels`** — signal voltages in IRE:

| Field | Default | Description |
|-------|---------|-------------|
| `sync_ire` | −40.0 | Sync tip |
| `blank_ire` | 0.0 | Blanking level |
| `black_ire` | 7.5 | Setup/pedestal |
| `white_ire` | 100.0 | Peak white |
| `burst_peak_ire` | 20.0 | Burst amplitude (±20 IRE) |
| `chroma_gain_ire` | 40.0 | Chroma subcarrier amplitude |

**`NtscLineMap`** — field-to-row mapping:

| Field | Default | Description |
|-------|---------|-------------|
| `top_field_start` | 20 | First visible line, top field |
| `bottom_field_start` | 283 | First visible line, bottom field |
| `lines_per_field_visible` | 240 | Visible lines per field |

All three stages (encoder, effects, decoder) should share the same `NtscSignalConfig` to keep timing assumptions consistent.

---

## API Reference

### `NtscEncoder`

Wraps `EncodeNtscCompositeFrame` with automatic frame counting.

```cpp
class NtscEncoder {
    explicit NtscEncoder(const NtscSignalConfig& config = {});

    void EncodeFrame(const uint8_t* rgb720x480, std::vector<int16_t>* out);
    void EncodeFrameAt(const uint8_t* rgb720x480, uint32_t frame_index,
                       std::vector<int16_t>* out);

    // Float variants — output IRE directly, skipping int16 quantisation.
    void EncodeFrameFloat(const uint8_t* rgb720x480, std::vector<float>* out);
    void EncodeFrameAtFloat(const uint8_t* rgb720x480, uint32_t frame_index,
                            std::vector<float>* out);

    void Reset();                          // reset frame counter to 0

    uint32_t frame_index() const;
    void set_frame_index(uint32_t idx);
    const NtscSignalConfig& config() const;
    NtscSignalConfig& config();
};
```

- **Input:** 720×480 packed RGB888 (1,036,800 bytes).
- **Output:** 477,750 int16 composite samples (or float IRE for the float variants).
- `EncodeFrame` uses and then increments the internal frame counter. The frame index matters because NTSC subcarrier phase rotates across frames; sequential calls produce correct phase continuity.
- `EncodeFrameAt` sets the counter to the given index, encodes, then increments. Use this for random-access encoding.
- `EncodeFrameFloat`/`EncodeFrameAtFloat` output IRE floats directly, avoiding the lossy int16 quantisation round-trip. Use these when feeding directly into `NtscEffects::Apply`.

### `NtscEffects`

Applies signal-domain degradations to a composite frame in IRE float format.

```cpp
class NtscEffects {
    explicit NtscEffects(const NtscSignalConfig& config = {});

    void Apply(std::vector<float>* frame_ire) const;
    bool HasActiveEffects() const;

    bool GetParam(ParamId id, float* value) const;
    bool SetParam(ParamId id, float value);

    NtscEffectControls& controls();
    const NtscEffectControls& controls() const;
    NtscSignalConfig& config();
    const NtscSignalConfig& config() const;
};
```

`Apply` mutates the signal in-place. Call it **before** decoding.

**`NtscEffectControls`** — all fields default to 0 (no effect):

| Field | Range | Unit | Description |
|-------|-------|------|-------------|
| `noise_stddev_ire` | 0–10 | IRE | Gaussian noise |
| `multipath_gain` | 0–1 | | Ghost image strength |
| `multipath_delay_samples` | 1–200 | samples | Ghost delay |
| `multipath_ensemble` | 0–1 | | Extra drifting ghost paths |
| `line_time_jitter_samples` | 0–2 | samples | Horizontal timing jitter |
| `afc_hunt` | 0–1 | | AFC lock hunting |
| `rf_drift` | 0–1 | | Slow RF gain/bias drift |
| `am_nonlinearity` | 0–1 | | AM detection compression |
| `impulse_noise` | 0–1 | | Spark/pop transients |
| `agc_pump` | 0–1 | | Gain breathing/pumping |
| `hum` | 0–1 | | Mains hum bars |
| `chroma_flutter` | 0–1 | | Chroma phase noise |
| `yc_crosstalk` | 0–1 | | Luma/chroma leakage |
| `h_sync_noise_ire` | 0–16 | IRE | Noise injected into hsync region |
| `v_sync_noise_ire` | 0–16 | IRE | Noise injected into vsync region |
| `burst_noise_ire` | 0–16 | IRE | Noise injected into colorburst |
| `vhs_tracking` | −1–1 | | VHS mistracking offset |
| `vhs_wrinkle` | 0–1 | | VHS tape wrinkle |
| `vhs_head_switch` | 0–1 | | VHS head-switch noise |
| `vhs_dropouts` | 0–1 | | VHS signal dropout rate |
| `noise_color` | 0–1 | | Noise spectrum: 0 = white, 1 = pink (1/f) |
| `group_delay` | 0–1 | | Frequency-dependent group delay (chroma smearing) |
| `random_seed` | 1+ | | RNG seed (set per-frame for deterministic results) |

### `NtscDecoder`

Pure signal decoder — no effects, no signal corruption. Takes a (possibly degraded) composite signal in IRE format and recovers an RGB picture.

```cpp
class NtscDecoder {
    explicit NtscDecoder(const NtscSignalConfig& config = {});

    void Decode(const std::vector<float>& frame_ire,
                std::vector<uint8_t>* rgb_out) const;

    bool GetParam(ParamId id, float* value) const;
    bool SetParam(ParamId id, float value);

    NtscDecodeControls& controls();
    const NtscDecodeControls& controls() const;
    NtscSignalConfig& config();
    const NtscSignalConfig& config() const;
};
```

- **Input:** 477,750 float samples in IRE.
- **Output:** 720×480 packed RGB888.

**`NtscDecodeControls`**:

| Field | Default | Range | Description |
|-------|---------|-------|-------------|
| `brightness` | 0.0 | −0.5–0.5 | Brightness offset |
| `contrast` | 1.0 | 0.2–2.0 | Contrast gain |
| `saturation` | 0.9 | 0–2.0 | Color saturation |
| `tint_degrees` | 0.0 | −45–45 | Hue rotation (degrees) |
| `sharpness` | 0.0 | 0–1 | Luma sharpening |
| `comb_filter` | true | | Y/C separation comb filter |
| `dot_crawl` | false | | Simulate dot-crawl artifact |
| `chroma_delay_pixels` | 0.0 | −6–6 | Chroma horizontal shift |
| `overscan_reveal` | 0.0 | 0–0.25 | Reveal VBI/edge area |
| `h_lock_instability` | 0.0 | 0–1 | Horizontal PLL weakness |
| `v_hold_instability` | 0.0 | 0–1 | Vertical hold weakness |
| `burst_lock_instability` | 0.0 | 0–1 | Color PLL weakness |
| `random_seed` | 1 | 1+ | RNG seed for PLL randomness |

#### Signal noise vs. decoder instability

The sync noise parameters (`h_sync_noise_ire`, `v_sync_noise_ire`, `burst_noise_ire`) in `NtscEffectControls` add noise to specific signal regions. This naturally degrades sync recovery because the decoder reads those regions.

The instability parameters (`h_lock_instability`, `v_hold_instability`, `burst_lock_instability`) in `NtscDecodeControls` independently weaken the decoder's PLL loops. A value of 0 means a perfect lock; 1 means maximum instability.

These are independent controls:
- **Clean signal + weak PLL:** sync noise = 0, instability > 0 → the decoder struggles even though the signal is perfect.
- **Noisy signal + strong PLL:** sync noise > 0, instability = 0 → the signal is corrupted but the decoder tracks through it.
- **Both:** sync noise > 0, instability > 0 → compounding degradation.

### `NtscProcessor`

Convenience class that composes all three stages. Useful when you want the full encode→effects→decode pipeline with minimal setup.

```cpp
class NtscProcessor {
    NtscProcessor();
    explicit NtscProcessor(const NtscSignalConfig& config);

    void ProcessFrame(const uint8_t* rgb_in, std::vector<uint8_t>* rgb_out);
    void Reset();

    bool GetParam(ParamId id, float* value) const;
    bool SetParam(ParamId id, float value);
    static const ParamMeta* GetParamTable(size_t* count);

    NtscEncoder& encoder();
    NtscEffects& effects();
    NtscDecoder& decoder();
};
```

`SetParam`/`GetParam` automatically route to the correct stage based on `ParamId` range (100–199 → decoder, 200–299 → effects, 300–399 → encoder).

---

## Parameter Metadata System

Every tunable parameter has a stable integer ID (`ParamId`), metadata (name, range, default, unit), and get/set support through the classes. This is designed for plugin hosts that need to auto-generate UI.

### Iterating all parameters

```cpp
size_t count = 0;
const rf2::ParamMeta* table = rf2::GetParamTable(&count);
for (size_t i = 0; i < count; ++i) {
    printf("%-24s  key=%-28s  range=[%.1f, %.1f]  default=%.2f  %s\n",
           table[i].name, table[i].key,
           table[i].min_val, table[i].max_val, table[i].default_val,
           table[i].unit);
}
```

### Looking up a single parameter

```cpp
const rf2::ParamMeta* meta = rf2::FindParamMeta(rf2::ParamId::kNoiseStddevIre);
if (meta) {
    printf("%s: %.1f to %.1f %s\n", meta->name, meta->min_val, meta->max_val, meta->unit);
}
```

### ParamId ranges

| Range | Category | Examples |
|-------|----------|---------|
| 100–199 | Decode controls | `kBrightness`, `kContrast`, `kHLockInstability` |
| 200–299 | Signal effects | `kNoiseStddevIre`, `kMultipathGain`, `kVhsTracking`, `kNoiseColor`, `kGroupDelay` |
| 300–399 | Encoder config | `kLumaCutoffHz`, `kChromaModScale`, `kFilterPasses` |

Gaps between IDs are intentional to allow future additions without breaking existing code.

### `ParamMeta` struct

```cpp
struct ParamMeta {
    ParamId id;
    ParamType type;         // kFloat, kBool, or kUint32
    ParamCategory category; // kDecode, kEffects, or kEncoder
    const char* name;       // display name, e.g. "Noise"
    const char* key;        // serialization key, e.g. "noise_stddev_ire"
    float min_val;
    float max_val;
    float default_val;
    const char* unit;       // e.g. "IRE", "Hz", "deg", "px", or ""
};
```

---

## NTRF Container Format

NTRF (.ntrf) is a simple sequential container for composite video frames with interleaved mono PCM audio.

### Writing

```cpp
#include "rf2/ntrf_container.h"

rf2::NtrfHeader header;
header.fps_num = 30000;
header.fps_den = 1001;
header.audio_rate = 48000;

rf2::NtrfWriter writer;
std::string error;
if (!writer.Open("output.ntrf", header, &error)) {
    // handle error
}

for (uint32_t i = 0; i < frame_count; ++i) {
    rf2::NtrfFrame frame;
    frame.frame_index = i;
    frame.sc_phase = static_cast<uint8_t>(i & 3);

    encoder.EncodeFrame(rgb, &frame.composite);
    frame.audio_pcm = /* your audio samples for this frame */;

    if (!writer.WriteFrame(frame, &error)) {
        // handle error
    }
}
writer.Close();
```

Frame compression (LZFSE) is enabled by default. Disable with `writer.SetFrameCompression(false)`.

### Reading (sequential)

```cpp
rf2::NtrfReader reader;
std::string error;
if (!reader.Open("input.ntrf", &error)) {
    // handle error
}

rf2::NtrfFrame frame;
while (reader.ReadNextFrame(&frame, &error)) {
    // process frame.composite, frame.audio_pcm
}
```

### Reading (random access)

```cpp
reader.Open("input.ntrf", &error);
reader.BuildIndex(&error);  // required for random access

rf2::NtrfFrame frame;
reader.ReadFrameAt(42, &frame, &error);  // jump to frame 42

size_t total = reader.FrameCount();
const auto& header = reader.header();
```

---

## Integration Patterns

### OFX / Plugin Host

Use `NtscProcessor` with the parameter metadata system. The host iterates `GetParamTable` to build UI, and routes values through `SetParam`/`GetParam` by `ParamId`.

```cpp
// Plugin init: register parameters from the table.
size_t count;
const rf2::ParamMeta* table = rf2::NtscProcessor::GetParamTable(&count);
for (size_t i = 0; i < count; ++i) {
    host->DefineParam(table[i].key, table[i].name,
                      table[i].min_val, table[i].max_val, table[i].default_val);
}

// Per-frame render:
rf2::NtscProcessor proc;
for (size_t i = 0; i < count; ++i) {
    float value = host->GetParamValue(table[i].key);
    proc.SetParam(table[i].id, value);
}
proc.ProcessFrame(input_rgb, &output_rgb);
```

### Decode-only player (reading NTRF files)

When you have pre-encoded composite data and want to apply effects and decode on playback:

```cpp
rf2::NtscSignalConfig signal;
rf2::NtscEffects effects(signal);
rf2::NtscDecoder decoder(signal);

// Adjust decoder picture controls.
decoder.controls().contrast = 1.2f;
decoder.controls().saturation = 1.0f;

// Add some analog warmth.
effects.controls().noise_stddev_ire = 1.5f;

std::vector<float> ire;
std::vector<uint8_t> rgb;

for (each frame from NTRF reader) {
    // Convert int16 composite to IRE.
    ire.resize(frame.composite.size());
    for (size_t i = 0; i < ire.size(); ++i) {
        ire[i] = rf2::SampleToIre(frame.composite[i]);
    }

    // Set per-frame seed for deterministic noise.
    effects.controls().random_seed = frame.frame_index + 1;
    decoder.controls().random_seed = frame.frame_index + 1;

    effects.Apply(&ire);
    decoder.Decode(ire, &rgb);
    // display rgb...
}
```

### Encode-only (generating NTRF from video)

```cpp
rf2::NtscEncoder encoder;
rf2::NtrfWriter writer;
// ... open writer ...

for (uint32_t i = 0; i < frame_count; ++i) {
    rf2::NtrfFrame frame;
    frame.frame_index = i;
    frame.sc_phase = static_cast<uint8_t>(i & 3);
    encoder.EncodeFrame(rgb_frames[i], &frame.composite);
    writer.WriteFrame(frame, &error);
}
```

---

## Building

RF2 requires C++20. Add all source files under `src/common/` to your build:

```
src/common/ntsc_signal.cpp
src/common/ntsc_params.cpp
src/common/ntsc_effects.cpp
src/common/ntsc_encoder.cpp
src/common/ntsc_decoder.cpp
src/common/ntsc_processor.cpp
src/common/ntrf_container.cpp
```

Add `include/` to your header search path. Link against `compression` (Apple platforms, for NTRF LZFSE support).

On ARM64 (Apple Silicon), the encoder uses NEON intrinsics automatically for faster composite signal generation.

### Minimal CMake example

```cmake
add_library(rf2 STATIC
    src/common/ntsc_signal.cpp
    src/common/ntsc_params.cpp
    src/common/ntsc_effects.cpp
    src/common/ntsc_encoder.cpp
    src/common/ntsc_decoder.cpp
    src/common/ntsc_processor.cpp
    src/common/ntrf_container.cpp
)
target_include_directories(rf2 PUBLIC include)
target_compile_features(rf2 PUBLIC cxx_std_20)

# Apple platforms: NTRF uses LZFSE compression.
if(APPLE)
    target_link_libraries(rf2 PRIVATE compression)
endif()
```

---

## Frame Format Reference

| Property | Value |
|----------|-------|
| Picture size | 720 × 480 pixels |
| Pixel format | Packed RGB888 (3 bytes/pixel, 1,036,800 bytes/frame) |
| Composite size | 525 lines × 910 samples = 477,750 int16 samples/frame |
| Sample rate | 4×fsc = 315/22 MHz ≈ 14.318 MHz |
| Frame rate | 29.97 fps (30000/1001) |
| IRE encoding | `int16 = round(ire × 256)` |
