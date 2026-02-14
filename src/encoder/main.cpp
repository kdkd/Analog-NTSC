// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#include <cerrno>
#include <cinttypes>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <limits>
#include <mutex>
#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <atomic>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

#include "rf2/ntrf_container.h"
#include "rf2/ntsc_encoder.h"

namespace {

struct Options {
  std::string input;
  std::string output;
  std::string output_tcp;
  uint32_t max_frames = 0;
  double max_ahead_sec = 2.0;
  bool live = false;
  uint32_t live_audio_buffer_ms = 500;
  uint32_t live_audio_preroll_ms = 120;
  bool frame_compress = true;
};

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --input <file|url> [--output <file.ntrf>] [--output-tcp <host:port>]"
            << " [--max-frames N] [--max-ahead-sec S]"
            << " [--live] [--live-audio-buffer-ms N] [--live-audio-preroll-ms N]"
            << " [--no-frame-compress]\n";
}

std::string ShellQuote(const std::string& s) {
  std::string out;
  out.reserve(s.size() + 8);
  out.push_back('\'');
  for (char c : s) {
    if (c == '\'') {
      out.append("'\\''");
    } else {
      out.push_back(c);
    }
  }
  out.push_back('\'');
  return out;
}

template <typename T>
void AppendPod(std::vector<uint8_t>* out, const T& value) {
  const auto* p = reinterpret_cast<const uint8_t*>(&value);
  out->insert(out->end(), p, p + sizeof(T));
}

bool WriteAll(int fd, const uint8_t* data, size_t bytes) {
  size_t written = 0;
  while (written < bytes) {
    const ssize_t n = ::send(fd, data + written, bytes - written, 0);
    if (n <= 0) {
      return false;
    }
    written += static_cast<size_t>(n);
  }
  return true;
}

bool ParseHostPort(const std::string& endpoint, std::string* host, std::string* port) {
  const size_t pos = endpoint.rfind(':');
  if (pos == std::string::npos || pos == 0 || pos + 1 >= endpoint.size()) {
    return false;
  }
  *host = endpoint.substr(0, pos);
  *port = endpoint.substr(pos + 1);
  return true;
}

class TcpNtrfSender {
 public:
  ~TcpNtrfSender() { Close(); }

  bool Connect(const std::string& endpoint, std::string* error) {
    std::string host;
    std::string port;
    if (!ParseHostPort(endpoint, &host, &port)) {
      if (error) {
        *error = "invalid --output-tcp endpoint, expected host:port";
      }
      return false;
    }

    struct addrinfo hints {};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    struct addrinfo* res = nullptr;
    const int rc = ::getaddrinfo(host.c_str(), port.c_str(), &hints, &res);
    if (rc != 0) {
      if (error) {
        *error = std::string("getaddrinfo failed: ") + gai_strerror(rc);
      }
      return false;
    }

    bool connected = false;
    for (addrinfo* ai = res; ai != nullptr && !connected; ai = ai->ai_next) {
      int s = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
      if (s < 0) {
        continue;
      }
      if (::connect(s, ai->ai_addr, ai->ai_addrlen) == 0) {
        fd_ = s;
        connected = true;
      } else {
        ::close(s);
      }
    }
    ::freeaddrinfo(res);

    if (!connected) {
      if (error) {
        *error = "failed to connect TCP output: " + endpoint;
      }
      return false;
    }
    return true;
  }

  bool SendHeader(const rf2::NtrfHeader& header, std::string* error) {
    std::vector<uint8_t> bytes;
    bytes.reserve(128);
    bytes.insert(bytes.end(), {'N', 'T', 'R', 'F'});
    AppendPod(&bytes, header.version);
    AppendPod(&bytes, header.width);
    AppendPod(&bytes, header.height);
    AppendPod(&bytes, header.lines_per_frame);
    AppendPod(&bytes, header.samples_per_line);
    AppendPod(&bytes, header.active_start);
    AppendPod(&bytes, header.active_samples);
    AppendPod(&bytes, header.fps_num);
    AppendPod(&bytes, header.fps_den);
    AppendPod(&bytes, header.composite_sample_rate_num);
    AppendPod(&bytes, header.composite_sample_rate_den);
    AppendPod(&bytes, header.audio_rate);
    AppendPod(&bytes, header.audio_channels);
    AppendPod(&bytes, header.reserved);
    if (!WriteAll(fd_, bytes.data(), bytes.size())) {
      if (error) {
        *error = "failed to send NTRF TCP header";
      }
      return false;
    }
    return true;
  }

  bool SendFrame(const rf2::NtrfFrame& frame, std::string* error) {
    const uint32_t composite_samples = static_cast<uint32_t>(frame.composite.size());
    const uint32_t audio_samples = static_cast<uint32_t>(frame.audio_pcm.size());
    const uint64_t payload_size = sizeof(uint32_t) + sizeof(uint8_t) + sizeof(uint8_t) +
                                  sizeof(uint16_t) + sizeof(uint64_t) + sizeof(uint32_t) +
                                  sizeof(uint32_t) +
                                  static_cast<uint64_t>(composite_samples) * sizeof(int16_t) +
                                  static_cast<uint64_t>(audio_samples) * sizeof(int16_t);
    if (payload_size > std::numeric_limits<uint32_t>::max()) {
      if (error) {
        *error = "frame too large for TCP packetization";
      }
      return false;
    }

    std::vector<uint8_t> bytes;
    bytes.reserve(static_cast<size_t>(payload_size) + 8U);
    bytes.insert(bytes.end(), {'F', 'R', 'M', '0'});
    const uint32_t payload_u32 = static_cast<uint32_t>(payload_size);
    AppendPod(&bytes, payload_u32);
    AppendPod(&bytes, frame.frame_index);
    AppendPod(&bytes, frame.sc_phase);
    AppendPod(&bytes, frame.flags);
    AppendPod(&bytes, frame.reserved);
    AppendPod(&bytes, frame.audio_start_sample);
    AppendPod(&bytes, composite_samples);
    AppendPod(&bytes, audio_samples);
    const auto* cbytes = reinterpret_cast<const uint8_t*>(frame.composite.data());
    const auto* abytes = reinterpret_cast<const uint8_t*>(frame.audio_pcm.data());
    bytes.insert(bytes.end(), cbytes, cbytes + frame.composite.size() * sizeof(int16_t));
    bytes.insert(bytes.end(), abytes, abytes + frame.audio_pcm.size() * sizeof(int16_t));

    if (!WriteAll(fd_, bytes.data(), bytes.size())) {
      if (error) {
        *error = "failed to send NTRF TCP frame";
      }
      return false;
    }
    return true;
  }

  void Close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

 private:
  int fd_ = -1;
};

bool ParseArgs(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--input" && (i + 1) < argc) {
      options->input = argv[++i];
    } else if (arg == "--output" && (i + 1) < argc) {
      options->output = argv[++i];
    } else if (arg == "--output-tcp" && (i + 1) < argc) {
      options->output_tcp = argv[++i];
    } else if (arg == "--max-frames" && (i + 1) < argc) {
      options->max_frames = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (arg == "--max-ahead-sec" && (i + 1) < argc) {
      options->max_ahead_sec = std::strtod(argv[++i], nullptr);
    } else if (arg == "--live") {
      options->live = true;
    } else if (arg == "--live-audio-buffer-ms" && (i + 1) < argc) {
      options->live_audio_buffer_ms = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (arg == "--live-audio-preroll-ms" && (i + 1) < argc) {
      options->live_audio_preroll_ms = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (arg == "--no-frame-compress") {
      options->frame_compress = false;
    } else if (arg == "--help" || arg == "-h") {
      return false;
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return false;
    }
  }
  return !options->input.empty() &&
         (!options->output.empty() || !options->output_tcp.empty());
}

bool ReadAllPcmMono16(FILE* pipe, std::vector<int16_t>* out_samples, std::string* error) {
  std::vector<uint8_t> bytes;
  std::array<uint8_t, 1 << 16> buf{};
  while (true) {
    const size_t n = std::fread(buf.data(), 1, buf.size(), pipe);
    if (n > 0) {
      bytes.insert(bytes.end(), buf.data(), buf.data() + n);
    }
    if (n < buf.size()) {
      if (std::ferror(pipe)) {
        if (error) {
          *error = "error while reading ffmpeg audio pipe";
        }
        return false;
      }
      break;
    }
  }

  if ((bytes.size() & 1U) != 0U) {
    bytes.pop_back();
  }
  out_samples->resize(bytes.size() / 2U);
  std::memcpy(out_samples->data(), bytes.data(), out_samples->size() * sizeof(int16_t));
  return true;
}

uint64_t FrameAudioStartSample(uint32_t frame_index, uint32_t audio_rate, uint32_t fps_num, uint32_t fps_den) {
  return (static_cast<uint64_t>(frame_index) * audio_rate * fps_den) / fps_num;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!ParseArgs(argc, argv, &options)) {
    PrintUsage(argv[0]);
    return 1;
  }

  const uint32_t width = 720;
  const uint32_t height = 480;
  const uint32_t fps_num = 30000;
  const uint32_t fps_den = 1001;
  const uint32_t audio_rate = 48000;
  const size_t rgb_frame_bytes = static_cast<size_t>(width) * height * 3U;

  const std::string input_q = ShellQuote(options.input);
  const bool network_input = (options.input.find("://") != std::string::npos);
  std::vector<int16_t> audio_samples;
  FILE* live_audio_pipe = nullptr;
  std::atomic<bool> live_audio_done(false);
  std::atomic<bool> live_audio_error(false);
  std::deque<int16_t> live_audio_queue;
  std::mutex live_audio_mutex;
  std::thread live_audio_thread;
  uint64_t live_audio_dropped = 0;

  if (!options.live) {
    std::cerr << "[rf2] extracting mono audio from input...\n";
    const std::string audio_cmd = "ffmpeg -v error -nostdin -i " + input_q +
                                  " -vn -ac 1 -ar 48000 -f s16le -";
    FILE* audio_pipe = popen(audio_cmd.c_str(), "r");
    if (!audio_pipe) {
      std::cerr << "failed to start ffmpeg audio process: " << std::strerror(errno) << "\n";
      return 1;
    }
    std::string audio_error;
    if (!ReadAllPcmMono16(audio_pipe, &audio_samples, &audio_error)) {
      (void)pclose(audio_pipe);
      std::cerr << audio_error << "\n";
      return 1;
    }
    const int audio_status = pclose(audio_pipe);
    if (audio_status != 0) {
      std::cerr << "ffmpeg audio process exited with status " << audio_status << "\n";
      return 1;
    }
    std::cerr << "[rf2] audio samples: " << audio_samples.size() << "\n";
  } else {
    std::cerr << "[rf2] live mode enabled (low buffering ingest)\n";
    const std::string audio_cmd =
        "ffmpeg -v error -nostdin " +
        std::string(network_input ? "-fflags nobuffer " : "") +
        "-i " + input_q +
        " -vn -ac 1 -ar 48000 -f s16le -";
    live_audio_pipe = popen(audio_cmd.c_str(), "r");
    if (!live_audio_pipe) {
      std::cerr << "failed to start live ffmpeg audio process: " << std::strerror(errno) << "\n";
      return 1;
    }
    const size_t max_audio_queue_samples =
        static_cast<size_t>(audio_rate) * std::max<uint32_t>(50, options.live_audio_buffer_ms) / 1000U;
    live_audio_thread = std::thread([&]() {
      std::array<int16_t, 4096> buf{};
      while (true) {
        const size_t n = std::fread(buf.data(), sizeof(int16_t), buf.size(), live_audio_pipe);
        if (n > 0) {
          std::lock_guard<std::mutex> lock(live_audio_mutex);
          for (size_t i = 0; i < n; ++i) {
            live_audio_queue.push_back(buf[i]);
          }
          while (live_audio_queue.size() > max_audio_queue_samples) {
            live_audio_queue.pop_front();
            ++live_audio_dropped;
          }
        }
        if (n < buf.size()) {
          if (std::ferror(live_audio_pipe)) {
            live_audio_error.store(true);
          }
          break;
        }
      }
      live_audio_done.store(true);
    });
  }

  std::cerr << "[rf2] extracting and encoding video frames...\n";
  std::string video_cmd = "ffmpeg -v error -nostdin ";
  if (options.live && network_input) {
    video_cmd += "-fflags nobuffer ";
  }
  video_cmd += "-i " + input_q +
               " -an -vf \"fps=30000/1001,"
               "scale=704:480:force_original_aspect_ratio=decrease:flags=bicubic,"
               "pad=704:480:(ow-iw)/2:(oh-ih)/2,"
               "pad=720:480:8:0,format=rgb24\"";
  if (options.max_frames > 0) {
    video_cmd += " -frames:v " + std::to_string(options.max_frames);
  }
  video_cmd += " -pix_fmt rgb24 -f rawvideo -";
  FILE* video_pipe = popen(video_cmd.c_str(), "r");
  if (!video_pipe) {
    std::cerr << "failed to start ffmpeg video process: " << std::strerror(errno) << "\n";
    return 1;
  }

  rf2::NtrfHeader header;
  header.width = width;
  header.height = height;
  header.lines_per_frame = 525;
  header.samples_per_line = 910;
  header.active_start = 142;
  header.active_samples = 754;
  header.fps_num = fps_num;
  header.fps_den = fps_den;
  header.audio_rate = audio_rate;
  header.audio_channels = 1;

  rf2::NtrfWriter writer;
  const bool output_file = !options.output.empty();
  const bool output_tcp = !options.output_tcp.empty();
  std::string error;
  if (output_file) {
    writer.SetFrameCompression(options.frame_compress);
    if (!writer.Open(options.output, header, &error)) {
      (void)pclose(video_pipe);
      std::cerr << error << "\n";
      return 1;
    }
  }

  TcpNtrfSender tcp_sender;
  if (output_tcp) {
    std::cerr << "[rf2] connecting TCP output " << options.output_tcp << "...\n";
    if (!tcp_sender.Connect(options.output_tcp, &error) ||
        !tcp_sender.SendHeader(header, &error)) {
      if (output_file) {
        writer.Close();
      }
      (void)pclose(video_pipe);
      std::cerr << error << "\n";
      return 1;
    }
  }

  rf2::NtscSignalConfig signal{};
  signal.timing.lines_per_frame = header.lines_per_frame;
  signal.timing.samples_per_line = header.samples_per_line;
  signal.timing.active_start = header.active_start;
  signal.timing.active_samples = header.active_samples;

  std::vector<uint8_t> rgb(rgb_frame_bytes, 0);
  uint32_t frame_index = 0;
  bool stopped_early = false;
  const auto stream_start = std::chrono::steady_clock::now();
  const double fps = static_cast<double>(fps_num) / static_cast<double>(fps_den);
  double worst_lag_sec = 0.0;
  if (options.live) {
    const size_t preroll_samples =
        static_cast<size_t>(audio_rate) * options.live_audio_preroll_ms / 1000U;
    while (true) {
      size_t qsize = 0;
      {
        std::lock_guard<std::mutex> lock(live_audio_mutex);
        qsize = live_audio_queue.size();
      }
      if (qsize >= preroll_samples || live_audio_done.load()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  uint64_t live_audio_underflow_samples = 0;
  while (true) {
    size_t total = 0;
    while (total < rgb_frame_bytes) {
      const size_t n = std::fread(rgb.data() + total, 1, rgb_frame_bytes - total, video_pipe);
      if (n == 0) {
        break;
      }
      total += n;
    }
    if (total == 0) {
      break;
    }
    if (total != rgb_frame_bytes) {
      std::cerr << "short video frame read (" << total << " bytes), stopping\n";
      break;
    }

    const uint64_t audio_start = FrameAudioStartSample(frame_index, audio_rate, fps_num, fps_den);
    const uint64_t audio_next = FrameAudioStartSample(frame_index + 1, audio_rate, fps_num, fps_den);
    const uint64_t audio_count_u64 = (audio_next > audio_start) ? (audio_next - audio_start) : 0;
    const uint32_t audio_count = static_cast<uint32_t>(audio_count_u64);

    rf2::NtrfFrame frame;
    frame.frame_index = frame_index;
    frame.sc_phase = static_cast<uint8_t>(frame_index & 0x3U);
    frame.audio_start_sample = audio_start;

    rf2::EncodeNtscCompositeFrame(rgb.data(), frame_index, signal, &frame.composite);
    frame.audio_pcm.assign(audio_count, 0);
    if (!options.live) {
      for (uint32_t i = 0; i < audio_count; ++i) {
        const uint64_t idx = audio_start + i;
        if (idx < audio_samples.size()) {
          frame.audio_pcm[i] = audio_samples[static_cast<size_t>(idx)];
        }
      }
    } else {
      uint32_t copied = 0;
      {
        std::lock_guard<std::mutex> lock(live_audio_mutex);
        while (copied < audio_count && !live_audio_queue.empty()) {
          frame.audio_pcm[copied++] = live_audio_queue.front();
          live_audio_queue.pop_front();
        }
      }
      if (copied < audio_count) {
        live_audio_underflow_samples += static_cast<uint64_t>(audio_count - copied);
      }
    }

    if (output_tcp) {
      const double frame_pts = static_cast<double>(frame_index) / fps;
      while (true) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed =
            std::chrono::duration_cast<std::chrono::duration<double>>(now - stream_start)
                .count();
        const double ahead = frame_pts - elapsed;
        if (ahead <= options.max_ahead_sec) {
          break;
        }
        const double sleep_sec = std::min(0.05, ahead - options.max_ahead_sec);
        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_sec));
      }
    }

    if (output_file && !writer.WriteFrame(frame, &error)) {
      std::cerr << "failed writing frame " << frame_index << ": " << error << "\n";
      writer.Close();
      tcp_sender.Close();
      if (live_audio_pipe) {
        (void)pclose(live_audio_pipe);
      }
      if (live_audio_thread.joinable()) {
        live_audio_thread.join();
      }
      (void)pclose(video_pipe);
      return 1;
    }
    if (output_tcp && !tcp_sender.SendFrame(frame, &error)) {
      std::cerr << "failed TCP send frame " << frame_index << ": " << error << "\n";
      if (output_file) {
        writer.Close();
      }
      tcp_sender.Close();
      if (live_audio_pipe) {
        (void)pclose(live_audio_pipe);
      }
      if (live_audio_thread.joinable()) {
        live_audio_thread.join();
      }
      (void)pclose(video_pipe);
      return 1;
    }

    if (output_tcp) {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed =
          std::chrono::duration_cast<std::chrono::duration<double>>(now - stream_start)
              .count();
      const double frame_pts = static_cast<double>(frame_index) / fps;
      const double lag = elapsed - frame_pts;
      worst_lag_sec = std::max(worst_lag_sec, lag);
      if ((frame_index % 120U) == 0U && lag > 0.25) {
        std::cerr << "[rf2] warning: encoder lagging realtime by " << lag << "s\n";
      }
    }

    ++frame_index;
    if (options.max_frames > 0 && frame_index >= options.max_frames) {
      stopped_early = true;
      break;
    }
    if ((frame_index % 30U) == 0U) {
      std::cerr << "[rf2] encoded " << frame_index << " frames\r";
    }
  }

  if (output_file) {
    writer.Close();
  }
  tcp_sender.Close();
  if (live_audio_pipe) {
    if (live_audio_thread.joinable()) {
      live_audio_thread.join();
    }
    const int live_audio_status = pclose(live_audio_pipe);
    if (!stopped_early && live_audio_status != 0 && !live_audio_done.load()) {
      std::cerr << "[rf2] warning: live audio process exited with status " << live_audio_status << "\n";
    }
  }
  const int video_status = pclose(video_pipe);
  if (video_status != 0 && !stopped_early) {
    std::cerr << "\nffmpeg video process exited with status " << video_status << "\n";
    return 1;
  }

  std::cerr << "\n[rf2] encoded " << frame_index << " frames";
  if (output_file) {
    std::cerr << " to " << options.output;
  }
  if (output_tcp) {
    std::cerr << " and streamed to " << options.output_tcp
              << " (worst lag " << worst_lag_sec << "s)";
  }
  if (options.live) {
    std::cerr << " [live audio underflow samples=" << live_audio_underflow_samples
              << ", dropped samples=" << live_audio_dropped;
    if (live_audio_error.load()) {
      std::cerr << ", audio-read-error=1";
    }
    std::cerr << "]";
  }
  std::cerr << "\n";
  return 0;
}
