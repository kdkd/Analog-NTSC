# NTSC Notes Used by RF2

RF2 models composite NTSC-M behavior at a level intended to be visually faithful:

- Horizontal line rate near `15734.2657 Hz`
- `227.5` color subcarrier cycles per line
- Composite sampled at `4*fsc` -> `910` samples per line
- `525` lines/frame, interlaced field mapping
- Color burst inserted on non-vertical-interval lines
- IRE-style levels for sync/blank/black/white handling

Reference used for quick practical timing sanity checks:

- https://www.nesdev.org/wiki/NTSC_video

This implementation is intentionally pragmatic: it preserves visible behavior (burst phase, luma/chroma interaction, line timing, sync structure) while avoiding non-visible complexity.
