// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#include "rf2/ntrf_container.h"

#include <array>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>
#include <compression.h>

namespace rf2 {

namespace {

constexpr std::array<char, 4> kFileMagic = {'N', 'T', 'R', 'F'};
constexpr std::array<char, 4> kFrameTag = {'F', 'R', 'M', '0'};
constexpr uint32_t kFramePayloadFixedBytes =
    sizeof(uint32_t) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint16_t) + sizeof(uint64_t) +
    sizeof(uint32_t) + sizeof(uint32_t);

template <typename T>
bool WritePod(std::ostream* out, const T& value) {
  out->write(reinterpret_cast<const char*>(&value), sizeof(T));
  return out->good();
}

template <typename T>
bool ReadPod(std::istream* in, T* value) {
  in->read(reinterpret_cast<char*>(value), sizeof(T));
  return in->good();
}

bool WriteHeader(std::ostream* out, const NtrfHeader& header) {
  out->write(kFileMagic.data(), static_cast<std::streamsize>(kFileMagic.size()));
  return out->good() && WritePod(out, header.version) && WritePod(out, header.width) &&
         WritePod(out, header.height) && WritePod(out, header.lines_per_frame) &&
         WritePod(out, header.samples_per_line) && WritePod(out, header.active_start) &&
         WritePod(out, header.active_samples) && WritePod(out, header.fps_num) &&
         WritePod(out, header.fps_den) &&
         WritePod(out, header.composite_sample_rate_num) &&
         WritePod(out, header.composite_sample_rate_den) && WritePod(out, header.audio_rate) &&
         WritePod(out, header.audio_channels) && WritePod(out, header.reserved);
}

bool ReadHeader(std::istream* in, NtrfHeader* header) {
  std::array<char, 4> magic{};
  in->read(magic.data(), static_cast<std::streamsize>(magic.size()));
  if (!in->good() || magic != kFileMagic) {
    return false;
  }
  return ReadPod(in, &header->version) && ReadPod(in, &header->width) &&
         ReadPod(in, &header->height) && ReadPod(in, &header->lines_per_frame) &&
         ReadPod(in, &header->samples_per_line) && ReadPod(in, &header->active_start) &&
         ReadPod(in, &header->active_samples) && ReadPod(in, &header->fps_num) &&
         ReadPod(in, &header->fps_den) &&
         ReadPod(in, &header->composite_sample_rate_num) &&
         ReadPod(in, &header->composite_sample_rate_den) && ReadPod(in, &header->audio_rate) &&
         ReadPod(in, &header->audio_channels) && ReadPod(in, &header->reserved);
}

bool CompressLzfse(const uint8_t* src, size_t src_size, std::vector<uint8_t>* dst) {
  if (!dst) {
    return false;
  }
  if (src_size == 0) {
    dst->clear();
    return true;
  }
  size_t cap = std::max<size_t>(1024, src_size / 2 + 64);
  dst->assign(cap, 0);
  while (true) {
    const size_t written = compression_encode_buffer(dst->data(),
                                                     dst->size(),
                                                     src,
                                                     src_size,
                                                     nullptr,
                                                     COMPRESSION_LZFSE);
    if (written != 0) {
      dst->resize(written);
      return true;
    }
    if (dst->size() >= src_size * 4 + 65536) {
      return false;
    }
    dst->resize(dst->size() * 2);
  }
}

bool DecompressLzfse(const uint8_t* src, size_t src_size, uint8_t* dst, size_t dst_size) {
  if (dst_size == 0) {
    return src_size == 0;
  }
  const size_t written =
      compression_decode_buffer(dst, dst_size, src, src_size, nullptr, COMPRESSION_LZFSE);
  return written == dst_size;
}

}  // namespace

NtrfWriter::~NtrfWriter() { Close(); }

bool NtrfWriter::Open(const std::string& path, const NtrfHeader& header, std::string* error) {
  Close();
  header_ = header;

  if (path == "-") {
    out_ = &std::cout;
    std::ios::sync_with_stdio(false);
  } else {
    auto stream = std::make_unique<std::ofstream>(path, std::ios::binary | std::ios::trunc);
    if (!stream->is_open()) {
      if (error) {
        *error = "failed to open output file: " + path;
      }
      return false;
    }
    out_ = stream.get();
    owned_out_ = std::move(stream);
  }

  header_written_ = WriteHeader(out_, header_);
  if (!header_written_) {
    if (error) {
      *error = "failed to write NTRF header";
    }
    Close();
    return false;
  }
  return true;
}

bool NtrfWriter::WriteFrame(const NtrfFrame& frame, std::string* error) {
  if (!out_ || !header_written_) {
    if (error) {
      *error = "writer is not open";
    }
    return false;
  }

  const uint32_t composite_samples = static_cast<uint32_t>(frame.composite.size());
  const uint32_t audio_samples = static_cast<uint32_t>(frame.audio_pcm.size());
  const size_t composite_bytes = static_cast<size_t>(composite_samples) * sizeof(int16_t);
  const size_t audio_bytes = static_cast<size_t>(audio_samples) * sizeof(int16_t);
  const size_t raw_data_bytes = composite_bytes + audio_bytes;

  std::vector<uint8_t> raw_data;
  raw_data.reserve(raw_data_bytes);
  const auto* comp_ptr = reinterpret_cast<const uint8_t*>(frame.composite.data());
  const auto* aud_ptr = reinterpret_cast<const uint8_t*>(frame.audio_pcm.data());
  raw_data.insert(raw_data.end(), comp_ptr, comp_ptr + composite_bytes);
  raw_data.insert(raw_data.end(), aud_ptr, aud_ptr + audio_bytes);

  bool use_compressed = false;
  std::vector<uint8_t> compressed_data;
  if (compress_frames_ && raw_data_bytes > 0 &&
      CompressLzfse(raw_data.data(), raw_data.size(), &compressed_data) &&
      compressed_data.size() < raw_data.size()) {
    use_compressed = true;
  }

  const uint8_t frame_flags = use_compressed ? static_cast<uint8_t>(frame.flags | kNtrfFrameFlagLzfseCompressed)
                                             : static_cast<uint8_t>(frame.flags & ~kNtrfFrameFlagLzfseCompressed);
  const uint8_t* payload_data = use_compressed ? compressed_data.data() : raw_data.data();
  const size_t payload_data_bytes = use_compressed ? compressed_data.size() : raw_data.size();

  const uint64_t payload_size =
      static_cast<uint64_t>(kFramePayloadFixedBytes) + static_cast<uint64_t>(payload_data_bytes);

  if (payload_size > std::numeric_limits<uint32_t>::max()) {
    if (error) {
      *error = "frame payload exceeds format limit";
    }
    return false;
  }

  const uint32_t payload_size32 = static_cast<uint32_t>(payload_size);
  out_->write(kFrameTag.data(), static_cast<std::streamsize>(kFrameTag.size()));
  if (!WritePod(out_, payload_size32) || !WritePod(out_, frame.frame_index) ||
      !WritePod(out_, frame.sc_phase) || !WritePod(out_, frame_flags) ||
      !WritePod(out_, frame.reserved) || !WritePod(out_, frame.audio_start_sample) ||
      !WritePod(out_, composite_samples) || !WritePod(out_, audio_samples)) {
    if (error) {
      *error = "failed to write frame header";
    }
    return false;
  }

  out_->write(reinterpret_cast<const char*>(payload_data), static_cast<std::streamsize>(payload_data_bytes));
  if (!out_->good()) {
    if (error) {
      *error = "failed to write frame payload";
    }
    return false;
  }

  return true;
}

void NtrfWriter::Close() {
  if (owned_out_) {
    auto* file = dynamic_cast<std::ofstream*>(owned_out_.get());
    if (file) {
      file->flush();
      file->close();
    }
  } else if (out_ == &std::cout) {
    std::cout.flush();
  }
  owned_out_.reset();
  out_ = nullptr;
  header_written_ = false;
}

NtrfReader::~NtrfReader() { Close(); }

bool NtrfReader::Open(const std::string& path, std::string* error) {
  Close();
  if (path == "-") {
    in_ = &std::cin;
    std::ios::sync_with_stdio(false);
  } else {
    auto stream = std::make_unique<std::ifstream>(path, std::ios::binary);
    if (!stream->is_open()) {
      if (error) {
        *error = "failed to open input file: " + path;
      }
      return false;
    }
    in_ = stream.get();
    owned_in_ = std::move(stream);
  }

  if (!ReadHeader(in_, &header_)) {
    if (error) {
      *error = "failed to read NTRF header or unsupported file magic";
    }
    Close();
    return false;
  }

  if (header_.version != kNtrfVersion) {
    if (error) {
      *error = "unsupported NTRF version";
    }
    Close();
    return false;
  }

  frames_start_offset_ = in_->tellg();
  index_.clear();
  return true;
}

bool NtrfReader::ReadFrameInternal(NtrfFrame* frame, std::string* error) {
  std::array<char, 4> tag{};
  in_->read(tag.data(), static_cast<std::streamsize>(tag.size()));
  if (in_->eof()) {
    return false;
  }
  if (!in_->good() || tag != kFrameTag) {
    if (error) {
      *error = "invalid frame tag";
    }
    return false;
  }

  uint32_t payload_size = 0;
  uint32_t composite_samples = 0;
  uint32_t audio_samples = 0;
  if (!ReadPod(in_, &payload_size) || !ReadPod(in_, &frame->frame_index) ||
      !ReadPod(in_, &frame->sc_phase) || !ReadPod(in_, &frame->flags) ||
      !ReadPod(in_, &frame->reserved) || !ReadPod(in_, &frame->audio_start_sample) ||
      !ReadPod(in_, &composite_samples) || !ReadPod(in_, &audio_samples)) {
    if (error) {
      *error = "failed to read frame header";
    }
    return false;
  }

  frame->composite.resize(composite_samples);
  frame->audio_pcm.resize(audio_samples);
  if (payload_size < kFramePayloadFixedBytes) {
    if (error) {
      *error = "invalid frame payload size";
    }
    return false;
  }
  const size_t data_bytes = static_cast<size_t>(payload_size - kFramePayloadFixedBytes);
  const size_t expected_raw_bytes = static_cast<size_t>(composite_samples) * sizeof(int16_t) +
                                    static_cast<size_t>(audio_samples) * sizeof(int16_t);

  if ((frame->flags & kNtrfFrameFlagLzfseCompressed) != 0) {
    std::vector<uint8_t> compressed(data_bytes, 0);
    in_->read(reinterpret_cast<char*>(compressed.data()), static_cast<std::streamsize>(compressed.size()));
    if (!in_->good()) {
      if (error) {
        *error = "failed to read compressed frame payload";
      }
      return false;
    }
    std::vector<uint8_t> raw(expected_raw_bytes, 0);
    if (!DecompressLzfse(compressed.data(), compressed.size(), raw.data(), raw.size())) {
      if (error) {
        *error = "failed to decompress frame payload";
      }
      return false;
    }
    if (expected_raw_bytes > 0) {
      std::memcpy(frame->composite.data(), raw.data(),
                  static_cast<size_t>(composite_samples) * sizeof(int16_t));
      std::memcpy(frame->audio_pcm.data(),
                  raw.data() + static_cast<size_t>(composite_samples) * sizeof(int16_t),
                  static_cast<size_t>(audio_samples) * sizeof(int16_t));
    }
  } else {
    if (data_bytes != expected_raw_bytes) {
      if (error) {
        *error = "raw frame payload size mismatch";
      }
      return false;
    }
    in_->read(reinterpret_cast<char*>(frame->composite.data()),
              static_cast<std::streamsize>(frame->composite.size() * sizeof(int16_t)));
    in_->read(reinterpret_cast<char*>(frame->audio_pcm.data()),
              static_cast<std::streamsize>(frame->audio_pcm.size() * sizeof(int16_t)));
    if (!in_->good()) {
      if (error) {
        *error = "failed to read frame payload";
      }
      return false;
    }
  }

  return true;
}

bool NtrfReader::ReadNextFrame(NtrfFrame* frame, std::string* error) {
  if (!in_) {
    if (error) {
      *error = "reader is not open";
    }
    return false;
  }
  return ReadFrameInternal(frame, error);
}

bool NtrfReader::BuildIndex(std::string* error) {
  if (!in_) {
    if (error) {
      *error = "reader is not open";
    }
    return false;
  }
  if (frames_start_offset_ < 0) {
    if (error) {
      *error = "reader stream is not seekable";
    }
    return false;
  }

  index_.clear();
  in_->clear();
  in_->seekg(frames_start_offset_, std::ios::beg);
  if (!in_->good()) {
    if (error) {
      *error = "failed to seek to first frame";
    }
    return false;
  }

  while (true) {
    const std::streamoff frame_offset = in_->tellg();
    NtrfFrame frame;
    std::string read_error;
    if (!ReadFrameInternal(&frame, &read_error)) {
      if (in_->eof()) {
        break;
      }
      if (error) {
        *error = read_error.empty() ? "failed to build frame index" : read_error;
      }
      return false;
    }
    NtrfFrameIndexEntry entry;
    entry.offset = frame_offset;
    entry.audio_start_sample = frame.audio_start_sample;
    entry.audio_samples = static_cast<uint32_t>(frame.audio_pcm.size());
    index_.push_back(entry);
  }

  in_->clear();
  in_->seekg(frames_start_offset_, std::ios::beg);
  return in_->good();
}

bool NtrfReader::ReadFrameAt(size_t index, NtrfFrame* frame, std::string* error) {
  if (!in_) {
    if (error) {
      *error = "reader is not open";
    }
    return false;
  }
  if (index_.empty()) {
    if (!BuildIndex(error)) {
      return false;
    }
  }
  if (index >= index_.size()) {
    if (error) {
      *error = "frame index out of range";
    }
    return false;
  }

  in_->clear();
  in_->seekg(index_[index].offset, std::ios::beg);
  if (!in_->good()) {
    if (error) {
      *error = "failed to seek to frame";
    }
    return false;
  }
  return ReadFrameInternal(frame, error);
}

void NtrfReader::ResetToFramesStart() {
  if (!in_) {
    return;
  }
  in_->clear();
  in_->seekg(frames_start_offset_, std::ios::beg);
}

void NtrfReader::Close() {
  if (owned_in_) {
    auto* file = dynamic_cast<std::ifstream*>(owned_in_.get());
    if (file) {
      file->close();
    }
  }
  owned_in_.reset();
  in_ = nullptr;
  frames_start_offset_ = 0;
  index_.clear();
}

}  // namespace rf2
