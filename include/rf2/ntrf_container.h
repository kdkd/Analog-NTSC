// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#pragma once

#include <cstddef>
#include <cstdint>
#include <istream>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

namespace rf2 {

constexpr uint32_t kNtrfVersion = 1;
constexpr uint32_t kDefaultCompositeSampleRateNum = 315000000;  // 4 * fsc
constexpr uint32_t kDefaultCompositeSampleRateDen = 22;
constexpr uint8_t kNtrfFrameFlagLzfseCompressed = 1u << 0;

struct NtrfHeader {
  uint32_t version = kNtrfVersion;
  uint32_t width = 720;
  uint32_t height = 480;
  uint32_t lines_per_frame = 525;
  uint32_t samples_per_line = 910;
  uint32_t active_start = 142;
  uint32_t active_samples = 754;
  uint32_t fps_num = 30000;
  uint32_t fps_den = 1001;
  uint32_t composite_sample_rate_num = kDefaultCompositeSampleRateNum;
  uint32_t composite_sample_rate_den = kDefaultCompositeSampleRateDen;
  uint32_t audio_rate = 48000;
  uint16_t audio_channels = 1;
  uint16_t reserved = 0;
};

struct NtrfFrame {
  uint32_t frame_index = 0;
  uint8_t sc_phase = 0;
  uint8_t flags = 0;
  uint16_t reserved = 0;
  uint64_t audio_start_sample = 0;
  std::vector<int16_t> composite;
  std::vector<int16_t> audio_pcm;
};

class NtrfWriter {
 public:
  NtrfWriter() = default;
  ~NtrfWriter();

  void SetFrameCompression(bool enabled) { compress_frames_ = enabled; }
  bool Open(const std::string& path, const NtrfHeader& header, std::string* error);
  bool WriteFrame(const NtrfFrame& frame, std::string* error);
  void Close();

 private:
  std::unique_ptr<std::ostream> owned_out_;
  std::ostream* out_ = nullptr;
  NtrfHeader header_{};
  bool header_written_ = false;
  bool compress_frames_ = true;
};

struct NtrfFrameIndexEntry {
  std::streamoff offset = 0;
  uint64_t audio_start_sample = 0;
  uint32_t audio_samples = 0;
};

class NtrfReader {
 public:
  NtrfReader() = default;
  ~NtrfReader();

  bool Open(const std::string& path, std::string* error);
  bool ReadNextFrame(NtrfFrame* frame, std::string* error);
  bool BuildIndex(std::string* error);
  bool ReadFrameAt(size_t index, NtrfFrame* frame, std::string* error);
  void ResetToFramesStart();
  void Close();

  const NtrfHeader& header() const { return header_; }
  const std::vector<NtrfFrameIndexEntry>& index() const { return index_; }
  size_t FrameCount() const { return index_.size(); }

 private:
  bool ReadFrameInternal(NtrfFrame* frame, std::string* error);

  std::unique_ptr<std::istream> owned_in_;
  std::istream* in_ = nullptr;
  std::streamoff frames_start_offset_ = 0;
  NtrfHeader header_{};
  std::vector<NtrfFrameIndexEntry> index_;
};

}  // namespace rf2
