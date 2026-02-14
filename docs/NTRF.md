# NTRF Container (v1)

`NTRF` is a little-endian frame-chunk format designed for NTSC composite + synced audio.

## File Header

Binary layout:

1. Magic: `NTRF` (4 bytes)
2. `uint32 version`
3. `uint32 width` (`720`)
4. `uint32 height` (`480`)
5. `uint32 lines_per_frame` (`525`)
6. `uint32 samples_per_line` (`910` for 4*fsc)
7. `uint32 active_start`
8. `uint32 active_samples`
9. `uint32 fps_num` (`30000`)
10. `uint32 fps_den` (`1001`)
11. `uint32 composite_sample_rate_num` (`315000000`)
12. `uint32 composite_sample_rate_den` (`22`) -> `4*fsc`
13. `uint32 audio_rate` (`48000`)
14. `uint16 audio_channels` (`1`)
15. `uint16 reserved`

## Frame Chunk

Each frame starts with tag `FRM0`:

1. Tag: `FRM0` (4 bytes)
2. `uint32 payload_size`
3. `uint32 frame_index`
4. `uint8 sc_phase` (subcarrier phase sequence index)
5. `uint8 flags`
6. `uint16 reserved`
7. `uint64 audio_start_sample`
8. `uint32 composite_samples`
9. `uint32 audio_samples`
10. `int16 composite[composite_samples]` (IRE-scaled composite waveform)
11. `int16 audio_pcm[audio_samples]` (mono PCM16)

## Streaming

The format is streamable: chunks are self-delimiting, so `ntsc-encode` output can be piped.

For random access and GUI playback, a seekable file is recommended so the player can build a frame index.
