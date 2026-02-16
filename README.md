# Analog NTSC

A video effect plugin for Adobe Premiere Pro and After Effects that simulates
NTSC composite video. Rather than applying lookup tables or blending pre-rendered
distortion overlays onto your footage, this plugin actually encodes each frame
into a synthetic NTSC composite signal, applies signal-domain effects to it, then
decodes it back to RGB.

https://github.com/user-attachments/assets/3f0d19be-d2ae-4d8b-bdac-718a49deb01f

This matters because NTSC's visual characteristics — the softened luma, the
limited chroma bandwidth, the dot-crawl patterns, the color bleeding on sharp
transitions — are all consequences of how the signal is encoded and decoded. A
real NTSC encoder bandlimits luma to ~4.2 MHz and chroma (I/Q) to 1.3/0.6 MHz
within a 14.3 MHz composite sample stream. When you modulate and demodulate
through that, the resolution loss and color artifacts emerge naturally. You don't
need to fake them with blur filters or color grading.

The same applies to the distortion controls. Noise is injected into the composite
waveform in IRE units, so it interacts with the chroma subcarrier the way real RF
noise does — degrading color lock before it wipes out the picture. Multipath
ghosting delays the composite signal by a sample count, so the ghost carries its
own chroma phase. Sync noise displaces horizontal and vertical timing the way
real interference does, producing line-by-line horizontal jitter and vertical
rolling rather than a uniform shake.

The VHS controls (tracking error, tape wrinkle, head switching, dropouts) are the
one area where the simulation is approximate. Real VHS artifacts come from
magnetic tape mechanics — helical scan geometry, FM luminance recording, and
under-carrier chroma — none of which are modeled here. Instead, the VHS effects
are applied to the composite signal in a way that produces visually similar
results. They're convincing enough for most uses, but they aren't derived from a
tape transport model.

## Installation

Download the latest release for your platform from the
[Releases](../../releases/) page:

- **macOS**: Open the `.dmg` and run the installer package.
- **Windows**: Run the `.exe` installer.

Both installers copy the plugin to the standard Adobe shared plugin directory.
After installing, restart Premiere Pro or After Effects. The plugin appears under
**Video Effects > Stylize > Analog NTSC**.

## Presets

The preset dropdown at the top of the effect controls sets all parameters at once
to match a specific signal scenario. Selecting a preset overwrites every
parameter. After selecting one, you can tweak individual values — the preset
switches to "Custom" implicitly.

| Preset | Description |
|---|---|
| Custom | No automatic values. All parameters are manually controlled. |
| Studio Monitor | Clean composite video with no signal degradation. Shows only NTSC's inherent bandwidth and color limitations — the picture is softer than the source but free of noise or artifacts. |
| Broadcast RF | Typical over-the-air reception with a decent antenna. Mild noise, faint ghosting, slight color instability. |
| Weak RF | Poor reception — distant transmitter, rabbit ears, bad weather. Heavy noise, visible ghosts, horizontal and vertical sync instability, color phase drift. |
| Good VHS | A fresh tape on a well-maintained deck. Slightly reduced bandwidth, minor head-switch bar at the bottom, subtle color shift and noise. |
| Worn VHS | A tape that's been played many times. More noise, visible head switching, occasional dropouts, washed-out color. |
| Damaged VHS | An old tape with significant wear. Heavy tracking noise, tape wrinkle artifacts, frequent dropouts, unstable sync, poor color fidelity. |

## Parameters

### Decoder

These control how the composite signal is decoded back to RGB.

| Parameter | Range | Default | Description |
|---|---|---|---|
| Brightness | -0.5 – 0.5 | 0.0 | DC offset applied to the decoded luma. |
| Contrast | 0.2 – 2.0 | 1.0 | Gain applied to the decoded luma. |
| Saturation | 0.0 – 2.0 | 0.9 | Chroma gain multiplier. Default is slightly below 1.0 to compensate for the decoder's chroma bandwidth overshoot. |
| Tint | -45 – 45 | 0.0 | Rotates the decoded chroma phase in degrees. Simulates a misadjusted tint knob. |
| Sharpness | 0.0 – 1.0 | 0.0 | Post-decode luma sharpening. |
| Comb Filter | on/off | on | Enables 2-line comb filtering for Y/C separation. When off, the decoder uses a simple bandpass, which increases dot-crawl and cross-color artifacts. |
| Dot Crawl | on/off | off | When enabled, the decoder preserves the frame-to-frame phase alternation that causes the characteristic crawling dot pattern along sharp edges. |
| Chroma Delay | -6.0 – 6.0 | 0.0 | Shifts decoded chroma left or right in pixels. Simulates the group delay mismatch between Y and C paths in real decoders. |
| Overscan Reveal | 0.0 – 0.25 | 0.0 | Shows the normally hidden blanking and sync regions around the active picture. At 0 the output is cropped to active video only. |
| Letterbox to 4:3 | on/off | on | Plugin input mapping mode for NTSC's 4:3 frame. On preserves source display aspect inside 4:3 (letterbox/pillarbox as needed). Off center-crops to fill 4:3 without stretching. |
| H Lock Instability | 0.0 – 1.0 | 0.0 | Per-line horizontal position jitter from noisy sync detection. |
| V Hold Instability | 0.0 – 1.0 | 0.0 | Frame-to-frame vertical position drift, like a misadjusted V-Hold knob. |
| Burst Lock Instability | 0.0 – 1.0 | 0.0 | Per-line chroma phase noise from imperfect burst locking. Causes random color shifts line by line. |

### Signal Effects

These operate on the composite signal between encoding and decoding.

| Parameter | Range | Default | Description |
|---|---|---|---|
| Noise | 0.0 – 20.0 | 0.0 | Additive Gaussian noise in IRE. Because it's added to the composite signal, it degrades chroma (which occupies a narrow band around 3.58 MHz) disproportionately more than luma. |
| Noise Color | 0.0 – 1.0 | 0.0 | Blends between white noise (0) and pink/1/f noise (1). Pink noise has more low-frequency energy and looks more like real RF/amplifier noise. |
| Ghost Gain | 0.0 – 1.0 | 0.0 | Amplitude of the multipath reflection. |
| Ghost Delay | 1 – 200 | 24 | Delay of the ghost in composite samples. Higher values push the ghost further to the right. |
| Ghost Ensemble | 0.0 – 1.0 | 0.0 | Randomizes the ghost delay slightly per-line, simulating multiple scattered reflections rather than a single clean echo. |
| Line Jitter | 0.0 – 2.0 | 0.0 | Random horizontal displacement per scanline, in samples. Simulates timing instability in the signal path. |
| AFC Hunt | 0.0 – 1.0 | 0.0 | Slow frequency drift of the receiver's automatic frequency control. Causes the picture to gently shift horizontally over time. |
| RF Drift | 0.0 – 1.0 | 0.0 | Carrier frequency drift. Introduces slow chroma phase rotation and brightness variation. |
| AM Nonlinearity | 0.0 – 1.0 | 0.0 | Soft-knee compression applied to the composite signal, simulating transmitter or amplifier nonlinearity. Creates intermodulation between luma and chroma. |
| Impulse Noise | 0.0 – 1.0 | 0.0 | Random high-amplitude spikes in the signal — ignition noise, electrical interference. |
| AGC Pump | 0.0 – 1.0 | 0.0 | Automatic gain control overshoot. Bright scenes momentarily compress and then recover, simulating a slow AGC loop. |
| Hum | 0.0 – 1.0 | 0.0 | 60 Hz power supply interference. Produces slow-moving horizontal brightness bars. |
| Chroma Flutter | 0.0 – 1.0 | 0.0 | Random chroma amplitude variation per line. Colors shimmer or shift in saturation. |
| Y/C Crosstalk | 0.0 – 1.0 | 0.0 | Luma-chroma crosstalk in the signal path. High-frequency luma detail bleeds into the chroma channel and vice versa. |
| H Sync Noise | 0.0 – 16.0 | 0.0 | Noise added to the horizontal sync pulses in IRE. Causes the decoder's sync separator to mistrack, displacing lines horizontally. |
| V Sync Noise | 0.0 – 16.0 | 0.0 | Noise added to the vertical sync pulses. Causes field-to-field vertical displacement. |
| Burst Noise | 0.0 – 16.0 | 0.0 | Noise added to the colorburst reference. Degrades the decoder's chroma phase lock, causing color instability. |
| Group Delay | 0.0 – 1.0 | 0.0 | Frequency-dependent delay from causal analog filtering. Higher frequencies arrive later, causing chroma to smear to the right of luma transitions. |
| VHS Tracking | -1.0 – 1.0 | 0.0 | Simulates a misadjusted tracking control. Positive and negative values shift in opposite directions. |
| VHS Wrinkle | 0.0 – 1.0 | 0.0 | Tape crease/wrinkle distortion — localized horizontal displacement bands. |
| VHS Head Switch | 0.0 – 1.0 | 0.0 | Noise bar at the bottom of the frame from the head drum switching between record heads. |
| VHS Dropouts | 0.0 – 1.0 | 0.0 | Random signal loss on individual scanlines, replaced with noise or repeated previous-line data. |

### Encoder

These control the properties of the synthetic NTSC encoder that processes each
frame before effects and decoding. Most users won't need to change these from
their defaults.

| Parameter | Range | Default | Description |
|---|---|---|---|
| Luma Cutoff MHz | 0.5 – 6.0 | 4.2 | Low-pass cutoff for the luma (Y) channel. The NTSC standard limits luma to ~4.2 MHz. Lower values produce a softer picture. |
| I Cutoff MHz | 0.2 – 1.5 | 1.1 | Low-pass cutoff for the I chroma channel. |
| Q Cutoff MHz | 0.1 – 0.6 | 0.45 | Low-pass cutoff for the Q chroma channel. Q is more aggressively filtered than I in real NTSC. |
| Chroma Mod Scale | 0.5 – 1.5 | 0.88 | Amplitude of the chroma subcarrier modulation relative to the luma signal. |
| Filter Passes | 1 – 6 | 3 | Number of bidirectional filter passes in the encoder. More passes produce a steeper rolloff closer to a real transmitter filter (~12 dB/octave per pass). |

## Building from source

The plugin can be built from source on macOS and Windows. The standalone
encoder/player tools (not covered here) are macOS-only.

### Prerequisites

You need the Premiere Pro C++ SDK and the After Effects Plug-in SDK from Adobe.
Both are free downloads but require an Adobe account.

1. Download the **Premiere Pro C++ SDK** from
   <https://developer.adobe.com/premiere-pro/#>
2. Download the **After Effects Plug-in SDK** from
   <https://developer.adobe.com/after-effects/>
3. Extract both into the appropriate platform directory:
   - macOS: `3rdparty/macos/`
   - Windows: `3rdparty/windows/`

The After Effects SDK ships as a zstd-compressed archive. After placing it in the
platform directory, run the included extraction script once:

- macOS: `cd 3rdparty/macos/AfterEffectsSDK_*/ && sh extractzstd.sh`
- Windows: `cd 3rdparty\windows\AfterEffectsSDK_*\ && extractzstd.bat`

The resulting directory layout should look like:

```
3rdparty/
  macos/   (or windows/)
    AfterEffectsSDK_.../
      ae25.6_61.64bit.AfterEffectsSDK/
        Examples/
          Headers/
          Resources/
    Premiere Pro 26.0 C++ SDK/
      Examples/
        Headers/
```

The exact SDK version numbers in the directory names may differ. If they do,
update the `AE_SDK_DIR` and `PR_SDK_DIR` paths in the Makefile to match.

### macOS

Install Xcode Command Line Tools if you haven't already:

```
xcode-select --install
```

Build the plugin:

```
make plugin
```

The output is `build/plugin/Analog NTSC.plugin/`. To create an installer DMG:

```
make plugin-release
```

This produces `build/Analog-NTSC.dmg` containing a `.pkg` installer that copies
the plugin to `/Library/Application Support/Adobe/Common/Plug-ins/7.0/MediaCore`.

### Windows

Install [MSYS2](https://www.msys2.org/). Open an MSYS2 UCRT64 or MINGW64
terminal and install the toolchain:

```
pacman -S mingw-w64-x86_64-gcc make
```

Build the plugin:

```
make plugin
```

The output is `build/plugin/Analog_NTSC.prm`. To create a self-extracting
installer, install NSIS and build the release target:

```
pacman -S mingw-w64-x86_64-nsis
make plugin-release
```

This produces `build/Analog-NTSC-Setup.exe`.

### Standalone tools (macOS only)

The repository also includes two command-line utilities that are separate from the
plugin. An encoder (`ntsc-encode`) takes existing media files via
libavcodec/libavformat, encodes them into a synthetic NTSC composite stream, and
writes the result to a file or pushes it over a TCP socket. A player
(`ntsc-player`) reads a pre-encoded file or receives a stream over a socket and
displays it in a macOS window with sliders for all the signal and decode
parameters. Together they're useful for developing and testing the encoder/decoder
pipeline without needing Premiere Pro or After Effects installed. They are not
required for the plugin.

To build them, you need Homebrew ffmpeg with libx264:

```
brew install ffmpeg
make
```

This produces `build/bin/ntsc-encode` and `build/bin/ntsc-player`. The `make`
default target builds only these two tools, not the plugin.

## License

BSD 2-Clause. See [LICENSE.txt](LICENSE.txt).
