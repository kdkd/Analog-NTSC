// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#import <AVFoundation/AVFoundation.h>
#import <Cocoa/Cocoa.h>

#include <cerrno>
#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

#include "rf2/ntrf_container.h"
#include "rf2/ntsc_effects.h"
#include "rf2/ntsc_decoder.h"
#if defined(HAVE_NDI_SDK)
#include "Processing.NDI.Lib.h"
#include "Processing.NDI.utilities.h"
#endif

namespace {

struct PlayerOptions {
  std::string input_path;
  std::string listen_endpoint;
  std::string ndi_name;
  std::string ndi_group;
  bool mute = false;
  std::string export_path;
  bool export_only = false;
  int crf = 18;
  std::string preset = "medium";
  uint32_t prebuffer_frames = 8;
  uint32_t max_buffer_frames = 120;
  bool low_latency_live = false;
  bool prebuffer_set = false;
  bool max_buffer_set = false;
};

uint64_t FrameToAudioSample(uint64_t frame_index, const rf2::NtrfHeader& header) {
  return (frame_index * header.audio_rate * header.fps_den) / header.fps_num;
}

uint64_t AudioSampleToFrame(uint64_t sample_index, const rf2::NtrfHeader& header) {
  return (sample_index * header.fps_num) / (static_cast<uint64_t>(header.audio_rate) * header.fps_den);
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

bool WriteWavMono16(const std::string& path, uint32_t sample_rate, const std::vector<int16_t>& pcm) {
  FILE* fp = std::fopen(path.c_str(), "wb");
  if (!fp) {
    return false;
  }

  const uint32_t data_size = static_cast<uint32_t>(pcm.size() * sizeof(int16_t));
  const uint32_t riff_size = 36U + data_size;
  const uint16_t audio_format = 1;
  const uint16_t channels = 1;
  const uint16_t bits_per_sample = 16;
  const uint32_t byte_rate = sample_rate * channels * (bits_per_sample / 8U);
  const uint16_t block_align = channels * (bits_per_sample / 8U);

  auto write_u32 = [&](uint32_t v) { return std::fwrite(&v, sizeof(v), 1, fp) == 1; };
  auto write_u16 = [&](uint16_t v) { return std::fwrite(&v, sizeof(v), 1, fp) == 1; };

  bool ok = true;
  ok = ok && std::fwrite("RIFF", 1, 4, fp) == 4;
  ok = ok && write_u32(riff_size);
  ok = ok && std::fwrite("WAVE", 1, 4, fp) == 4;
  ok = ok && std::fwrite("fmt ", 1, 4, fp) == 4;
  ok = ok && write_u32(16);
  ok = ok && write_u16(audio_format);
  ok = ok && write_u16(channels);
  ok = ok && write_u32(sample_rate);
  ok = ok && write_u32(byte_rate);
  ok = ok && write_u16(block_align);
  ok = ok && write_u16(bits_per_sample);
  ok = ok && std::fwrite("data", 1, 4, fp) == 4;
  ok = ok && write_u32(data_size);
  ok = ok && std::fwrite(pcm.data(), sizeof(int16_t), pcm.size(), fp) == pcm.size();

  std::fclose(fp);
  return ok;
}

bool ReadExactFd(int fd, void* dst, size_t bytes) {
  uint8_t* out = reinterpret_cast<uint8_t*>(dst);
  size_t total = 0;
  while (total < bytes) {
    const ssize_t n = ::recv(fd, out + total, bytes - total, 0);
    if (n <= 0) {
      return false;
    }
    total += static_cast<size_t>(n);
  }
  return true;
}

template <typename T>
bool ReadPodFd(int fd, T* value) {
  return ReadExactFd(fd, value, sizeof(T));
}

bool ReadNtrfHeaderFromFd(int fd, rf2::NtrfHeader* header, std::string* error) {
  std::array<char, 4> magic{};
  if (!ReadExactFd(fd, magic.data(), magic.size()) ||
      magic != std::array<char, 4>{'N', 'T', 'R', 'F'}) {
    if (error) {
      *error = "invalid or missing NTRF stream header";
    }
    return false;
  }
  if (!ReadPodFd(fd, &header->version) || !ReadPodFd(fd, &header->width) ||
      !ReadPodFd(fd, &header->height) || !ReadPodFd(fd, &header->lines_per_frame) ||
      !ReadPodFd(fd, &header->samples_per_line) || !ReadPodFd(fd, &header->active_start) ||
      !ReadPodFd(fd, &header->active_samples) || !ReadPodFd(fd, &header->fps_num) ||
      !ReadPodFd(fd, &header->fps_den) ||
      !ReadPodFd(fd, &header->composite_sample_rate_num) ||
      !ReadPodFd(fd, &header->composite_sample_rate_den) || !ReadPodFd(fd, &header->audio_rate) ||
      !ReadPodFd(fd, &header->audio_channels) || !ReadPodFd(fd, &header->reserved)) {
    if (error) {
      *error = "failed reading NTRF stream header fields";
    }
    return false;
  }
  return true;
}

bool ReadNtrfFrameFromFd(int fd, rf2::NtrfFrame* frame, std::string* error) {
  std::array<char, 4> tag{};
  if (!ReadExactFd(fd, tag.data(), tag.size())) {
    if (error) {
      *error = "stream closed";
    }
    return false;
  }
  if (tag != std::array<char, 4>{'F', 'R', 'M', '0'}) {
    if (error) {
      *error = "invalid stream frame tag";
    }
    return false;
  }

  uint32_t payload_size = 0;
  uint32_t composite_samples = 0;
  uint32_t audio_samples = 0;
  if (!ReadPodFd(fd, &payload_size) || !ReadPodFd(fd, &frame->frame_index) ||
      !ReadPodFd(fd, &frame->sc_phase) || !ReadPodFd(fd, &frame->flags) ||
      !ReadPodFd(fd, &frame->reserved) || !ReadPodFd(fd, &frame->audio_start_sample) ||
      !ReadPodFd(fd, &composite_samples) || !ReadPodFd(fd, &audio_samples)) {
    if (error) {
      *error = "failed reading stream frame header";
    }
    return false;
  }
  (void)payload_size;

  frame->composite.resize(composite_samples);
  frame->audio_pcm.resize(audio_samples);
  if (!ReadExactFd(fd, frame->composite.data(), frame->composite.size() * sizeof(int16_t)) ||
      !ReadExactFd(fd, frame->audio_pcm.data(), frame->audio_pcm.size() * sizeof(int16_t))) {
    if (error) {
      *error = "failed reading stream frame payload";
    }
    return false;
  }
  return true;
}

bool ParseListenEndpoint(const std::string& endpoint, std::string* host, std::string* port) {
  const size_t pos = endpoint.rfind(':');
  if (pos == std::string::npos) {
    if (endpoint.empty()) {
      return false;
    }
    *host = "0.0.0.0";
    *port = endpoint;
    return true;
  }
  if (pos == 0 || pos + 1 >= endpoint.size()) {
    return false;
  }
  *host = endpoint.substr(0, pos);
  *port = endpoint.substr(pos + 1);
  return true;
}

int OpenListenSocket(const std::string& endpoint, std::string* error) {
  std::string host;
  std::string port;
  if (!ParseListenEndpoint(endpoint, &host, &port)) {
    if (error) {
      *error = "invalid --listen endpoint (expected <port> or <host:port>)";
    }
    return -1;
  }

  struct addrinfo hints {};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;
  struct addrinfo* res = nullptr;
  const int rc = ::getaddrinfo(host == "0.0.0.0" ? nullptr : host.c_str(), port.c_str(), &hints, &res);
  if (rc != 0) {
    if (error) {
      *error = std::string("getaddrinfo failed: ") + gai_strerror(rc);
    }
    return -1;
  }

  int listen_fd = -1;
  int last_errno = 0;
  std::string last_bind_addr;
  for (addrinfo* ai = res; ai != nullptr; ai = ai->ai_next) {
    int s = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (s < 0) {
      last_errno = errno;
      continue;
    }
    int yes = 1;
    (void)::setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
#ifdef SO_REUSEPORT
    (void)::setsockopt(s, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes));
#endif
    if (::bind(s, ai->ai_addr, ai->ai_addrlen) == 0 && ::listen(s, 1) == 0) {
      listen_fd = s;
      break;
    }
    last_errno = errno;
    char host_buf[NI_MAXHOST] = {};
    char serv_buf[NI_MAXSERV] = {};
    if (::getnameinfo(ai->ai_addr,
                      ai->ai_addrlen,
                      host_buf,
                      sizeof(host_buf),
                      serv_buf,
                      sizeof(serv_buf),
                      NI_NUMERICHOST | NI_NUMERICSERV) == 0) {
      last_bind_addr = std::string(host_buf) + ":" + serv_buf;
    }
    ::close(s);
  }
  ::freeaddrinfo(res);
  if (listen_fd < 0 && error) {
    if (last_errno != 0) {
      *error = "failed to open listen socket on " +
               (last_bind_addr.empty() ? endpoint : last_bind_addr) + ": " +
               std::strerror(last_errno);
    } else {
      *error = "failed to open listen socket";
    }
  }
  return listen_fd;
}

bool ParseArgs(int argc, const char** argv, PlayerOptions* options) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--input" && (i + 1) < argc) {
      options->input_path = argv[++i];
    } else if (arg == "--listen" && (i + 1) < argc) {
      options->listen_endpoint = argv[++i];
    } else if (arg == "--ndi-name" && (i + 1) < argc) {
      options->ndi_name = argv[++i];
    } else if (arg == "--ndi-group" && (i + 1) < argc) {
      options->ndi_group = argv[++i];
    } else if (arg == "--mute") {
      options->mute = true;
    } else if (arg == "--export" && (i + 1) < argc) {
      options->export_path = argv[++i];
    } else if (arg == "--export-only") {
      options->export_only = true;
    } else if (arg == "--crf" && (i + 1) < argc) {
      options->crf = std::atoi(argv[++i]);
    } else if (arg == "--preset" && (i + 1) < argc) {
      options->preset = argv[++i];
    } else if (arg == "--prebuffer-frames" && (i + 1) < argc) {
      options->prebuffer_frames = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
      options->prebuffer_set = true;
    } else if (arg == "--max-buffer-frames" && (i + 1) < argc) {
      options->max_buffer_frames = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
      options->max_buffer_set = true;
    } else if (arg == "--low-latency-live") {
      options->low_latency_live = true;
    } else if (arg == "--help" || arg == "-h") {
      return false;
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return false;
    }
  }
  if (!options->listen_endpoint.empty() && options->low_latency_live) {
    if (!options->prebuffer_set) {
      options->prebuffer_frames = 2;
    }
    if (!options->max_buffer_set) {
      options->max_buffer_frames = 24;
    }
  }
  return !options->input_path.empty() || !options->listen_endpoint.empty();
}

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0 << " (--input <file.ntrf> | --listen <port|host:port>) [--mute]"
            << " [--export out.mp4] [--export-only] [--crf N] [--preset name]"
            << " [--prebuffer-frames N] [--max-buffer-frames N] [--low-latency-live]"
            << " [--ndi-name name] [--ndi-group group]\n";
}

void ScaleLetterboxRgbToBgra1920x1080(const std::vector<uint8_t>& src_rgb,
                                      int src_w,
                                      int src_h,
                                      float overscan_reveal,
                                      std::vector<uint8_t>* out_bgra) {
  constexpr int kDstW = 1920;
  constexpr int kDstH = 1080;
  if (!out_bgra) {
    return;
  }
  out_bgra->assign(static_cast<size_t>(kDstW) * kDstH * 4U, 0U);
  if (src_rgb.empty() || src_w <= 0 || src_h <= 0) {
    return;
  }

  const double src_aspect = static_cast<double>(src_w) / static_cast<double>(src_h);
  const double dst_aspect = static_cast<double>(kDstW) / static_cast<double>(kDstH);
  int draw_w = kDstW;
  int draw_h = kDstH;
  if (dst_aspect > src_aspect) {
    draw_w = std::max(1, static_cast<int>(std::lround(static_cast<double>(kDstH) * src_aspect)));
  } else {
    draw_h = std::max(1, static_cast<int>(std::lround(static_cast<double>(kDstW) / src_aspect)));
  }
  const float reveal = std::max(0.0f, std::min(0.30f, overscan_reveal));
  const double zoom = std::max(0.5, 1.0 - static_cast<double>(reveal));
  draw_w = std::max(1, static_cast<int>(std::lround(static_cast<double>(draw_w) * zoom)));
  draw_h = std::max(1, static_cast<int>(std::lround(static_cast<double>(draw_h) * zoom)));
  const int off_x = (kDstW - draw_w) / 2;
  const int off_y = (kDstH - draw_h) / 2;

  std::vector<int> x_map(static_cast<size_t>(draw_w), 0);
  std::vector<int> y_map(static_cast<size_t>(draw_h), 0);
  for (int x = 0; x < draw_w; ++x) {
    int sx = static_cast<int>((static_cast<int64_t>(x) * src_w) / draw_w);
    sx = std::max(0, std::min(src_w - 1, sx));
    x_map[static_cast<size_t>(x)] = sx;
  }
  for (int y = 0; y < draw_h; ++y) {
    int sy = static_cast<int>((static_cast<int64_t>(y) * src_h) / draw_h);
    sy = std::max(0, std::min(src_h - 1, sy));
    y_map[static_cast<size_t>(y)] = sy;
  }

  uint8_t* dst = out_bgra->data();
  for (int y = 0; y < draw_h; ++y) {
    const int sy = y_map[static_cast<size_t>(y)];
    const uint8_t* src_row = src_rgb.data() + static_cast<size_t>(sy) * src_w * 3U;
    uint8_t* dst_row =
        dst + (static_cast<size_t>(off_y + y) * kDstW + static_cast<size_t>(off_x)) * 4U;
    for (int x = 0; x < draw_w; ++x) {
      const int sx = x_map[static_cast<size_t>(x)];
      const uint8_t* p = src_row + static_cast<size_t>(sx) * 3U;
      dst_row[static_cast<size_t>(x) * 4U + 0U] = p[2];
      dst_row[static_cast<size_t>(x) * 4U + 1U] = p[1];
      dst_row[static_cast<size_t>(x) * 4U + 2U] = p[0];
      dst_row[static_cast<size_t>(x) * 4U + 3U] = 255U;
    }
  }
}

bool ExportNtrfToMp4(const PlayerOptions& options) {
  rf2::NtrfReader reader;
  std::string error;
  if (!reader.Open(options.input_path, &error)) {
    std::cerr << error << "\n";
    return false;
  }
  if (!reader.BuildIndex(&error)) {
    std::cerr << error << "\n";
    return false;
  }

  const rf2::NtrfHeader header = reader.header();
  const size_t frame_count = reader.FrameCount();
  if (frame_count == 0) {
    std::cerr << "no frames in input\n";
    return false;
  }

  rf2::NtscSignalConfig signal{};
  signal.timing.lines_per_frame = header.lines_per_frame;
  signal.timing.samples_per_line = header.samples_per_line;
  signal.timing.active_start = header.active_start;
  signal.timing.active_samples = header.active_samples;

  rf2::NtscEffects effects(signal);
  rf2::NtscDecoder decoder(signal);
  effects.controls().random_seed = 1;
  std::vector<float> frame_ire;

  char tmp_template[] = "/tmp/rf2_export_XXXXXX";
  char* tmp_dir = mkdtemp(tmp_template);
  if (!tmp_dir) {
    std::cerr << "failed to create temp directory: " << std::strerror(errno) << "\n";
    return false;
  }
  const std::filesystem::path temp_dir(tmp_dir);
  const std::filesystem::path raw_path = temp_dir / "video.rgb";
  const std::filesystem::path wav_path = temp_dir / "audio.wav";

  FILE* raw_fp = std::fopen(raw_path.c_str(), "wb");
  if (!raw_fp) {
    std::cerr << "failed to open temp video file\n";
    std::filesystem::remove_all(temp_dir);
    return false;
  }

  uint64_t total_audio_samples = 0;
  for (const auto& e : reader.index()) {
    total_audio_samples = std::max(total_audio_samples, e.audio_start_sample + e.audio_samples);
  }
  std::vector<int16_t> audio_pcm(total_audio_samples, 0);

  std::vector<uint8_t> rgb;
  rf2::NtrfFrame frame;
  for (size_t i = 0; i < frame_count; ++i) {
    if (!reader.ReadFrameAt(i, &frame, &error)) {
      std::cerr << "failed to read frame " << i << ": " << error << "\n";
      std::fclose(raw_fp);
      std::filesystem::remove_all(temp_dir);
      return false;
    }
    effects.controls().random_seed = static_cast<uint32_t>(i + 1);
    decoder.controls().random_seed = static_cast<uint32_t>(i + 1);
    frame_ire.resize(frame.composite.size());
    for (size_t s = 0; s < frame.composite.size(); ++s) {
      frame_ire[s] = rf2::SampleToIre(frame.composite[s]);
    }
    effects.Apply(&frame_ire);
    decoder.Decode(frame_ire, &rgb);
    if (std::fwrite(rgb.data(), 1, rgb.size(), raw_fp) != rgb.size()) {
      std::cerr << "failed to write temp video frame\n";
      std::fclose(raw_fp);
      std::filesystem::remove_all(temp_dir);
      return false;
    }
    for (size_t s = 0; s < frame.audio_pcm.size(); ++s) {
      const uint64_t pos = frame.audio_start_sample + s;
      if (pos < audio_pcm.size()) {
        audio_pcm[static_cast<size_t>(pos)] = frame.audio_pcm[s];
      }
    }
    if ((i % 60U) == 0U) {
      std::cerr << "[rf2] export decoded " << i << "/" << frame_count << " frames\r";
    }
  }
  std::fclose(raw_fp);
  std::cerr << "\n";

  if (!WriteWavMono16(wav_path.string(), header.audio_rate, audio_pcm)) {
    std::cerr << "failed to write temp wav\n";
    std::filesystem::remove_all(temp_dir);
    return false;
  }

  const std::string ffmpeg_cmd =
      "ffmpeg -y -v error -f rawvideo -pix_fmt rgb24 -s 720x480 -r 30000/1001 -i " +
      ShellQuote(raw_path.string()) + " -i " + ShellQuote(wav_path.string()) +
      " -c:v libx264 -preset " + ShellQuote(options.preset) + " -crf " + std::to_string(options.crf) +
      " -pix_fmt yuv420p -c:a aac -b:a 128k -movflags +faststart " +
      ShellQuote(options.export_path);

  std::cerr << "[rf2] running ffmpeg export...\n";
  const int status = std::system(ffmpeg_cmd.c_str());
  std::filesystem::remove_all(temp_dir);
  if (status != 0) {
    std::cerr << "ffmpeg export failed with status " << status << "\n";
    return false;
  }
  std::cerr << "[rf2] export complete: " << options.export_path << "\n";
  return true;
}

void BuildMainMenu() {
  NSMenu* menubar = [[NSMenu alloc] initWithTitle:@""];

  NSMenuItem* appMenuItem = [[NSMenuItem alloc] initWithTitle:@"" action:nil keyEquivalent:@""];
  [menubar addItem:appMenuItem];
  NSMenu* appMenu = [[NSMenu alloc] initWithTitle:@"Application"];
  NSString* appName = [[NSProcessInfo processInfo] processName];

  NSString* aboutTitle = [NSString stringWithFormat:@"About %@", appName];
  [appMenu addItem:[[NSMenuItem alloc] initWithTitle:aboutTitle
                                               action:@selector(orderFrontStandardAboutPanel:)
                                        keyEquivalent:@""]];
  [appMenu addItem:[NSMenuItem separatorItem]];
  NSString* hideTitle = [NSString stringWithFormat:@"Hide %@", appName];
  [appMenu addItem:[[NSMenuItem alloc] initWithTitle:hideTitle
                                               action:@selector(hide:)
                                        keyEquivalent:@"h"]];
  [appMenu addItem:[[NSMenuItem alloc] initWithTitle:@"Hide Others"
                                               action:@selector(hideOtherApplications:)
                                        keyEquivalent:@"h"]];
  [[appMenu itemAtIndex:3] setKeyEquivalentModifierMask:(NSEventModifierFlagOption | NSEventModifierFlagCommand)];
  [appMenu addItem:[[NSMenuItem alloc] initWithTitle:@"Show All"
                                               action:@selector(unhideAllApplications:)
                                        keyEquivalent:@""]];
  [appMenu addItem:[NSMenuItem separatorItem]];
  NSString* quitTitle = [NSString stringWithFormat:@"Quit %@", appName];
  [appMenu addItem:[[NSMenuItem alloc] initWithTitle:quitTitle
                                               action:@selector(terminate:)
                                        keyEquivalent:@"q"]];
  [appMenuItem setSubmenu:appMenu];

  NSMenuItem* windowMenuItem = [[NSMenuItem alloc] initWithTitle:@"" action:nil keyEquivalent:@""];
  [menubar addItem:windowMenuItem];
  NSMenu* windowMenu = [[NSMenu alloc] initWithTitle:@"Window"];
  [windowMenu addItem:[[NSMenuItem alloc] initWithTitle:@"Minimize"
                                                  action:@selector(performMiniaturize:)
                                           keyEquivalent:@"m"]];
  [windowMenu addItem:[[NSMenuItem alloc] initWithTitle:@"Zoom"
                                                  action:@selector(performZoom:)
                                           keyEquivalent:@""]];
  [windowMenu addItem:[NSMenuItem separatorItem]];
  [windowMenu addItem:[[NSMenuItem alloc] initWithTitle:@"Bring All to Front"
                                                  action:@selector(arrangeInFront:)
                                           keyEquivalent:@""]];
  [windowMenuItem setSubmenu:windowMenu];

  [NSApp setMainMenu:menubar];
  [NSApp setWindowsMenu:windowMenu];
}

}  // namespace

@interface RFFrameView : NSView
- (void)updateFrameRgb:(const std::vector<uint8_t>&)rgb width:(int)width height:(int)height;
- (void)setOverscanReveal:(float)overscanReveal;
@end

@implementation RFFrameView {
  std::vector<uint8_t> _rgb;
  int _width;
  int _height;
  float _overscanReveal;
}

- (instancetype)initWithFrame:(NSRect)frameRect {
  self = [super initWithFrame:frameRect];
  if (self) {
    _width = 0;
    _height = 0;
    _overscanReveal = 0.0f;
    self.wantsLayer = YES;
  }
  return self;
}

- (BOOL)isFlipped {
  return YES;
}

- (void)updateFrameRgb:(const std::vector<uint8_t>&)rgb width:(int)width height:(int)height {
  _rgb = rgb;
  _width = width;
  _height = height;
  [self setNeedsDisplay:YES];
}

- (void)setOverscanReveal:(float)overscanReveal {
  _overscanReveal = std::max(0.0f, std::min(0.30f, overscanReveal));
  [self setNeedsDisplay:YES];
}

- (void)drawRect:(NSRect)dirtyRect {
  (void)dirtyRect;
  [[NSColor blackColor] setFill];
  NSRectFill(self.bounds);

  if (_rgb.empty() || _width <= 0 || _height <= 0) {
    return;
  }

  CGColorSpaceRef color_space = CGColorSpaceCreateDeviceRGB();
  CGDataProviderRef provider =
      CGDataProviderCreateWithData(nullptr, _rgb.data(), _rgb.size(), nullptr);
  CGImageRef image = CGImageCreate(_width, _height, 8, 24, _width * 3, color_space,
                                   kCGImageAlphaNone, provider, nullptr, false,
                                   kCGRenderingIntentDefault);
  CGDataProviderRelease(provider);
  CGColorSpaceRelease(color_space);
  if (!image) {
    return;
  }

  NSRect bounds = self.bounds;
  const CGFloat src_aspect = static_cast<CGFloat>(_width) / static_cast<CGFloat>(_height);
  const CGFloat dst_aspect = bounds.size.width / bounds.size.height;
  NSRect draw = bounds;
  if (dst_aspect > src_aspect) {
    const CGFloat w = bounds.size.height * src_aspect;
    draw.origin.x = (bounds.size.width - w) * 0.5;
    draw.size.width = w;
  } else {
    const CGFloat h = bounds.size.width / src_aspect;
    draw.origin.y = (bounds.size.height - h) * 0.5;
    draw.size.height = h;
  }
  const CGFloat zoom = std::max<CGFloat>(0.5, 1.0 - static_cast<CGFloat>(_overscanReveal));
  const CGFloat cx = draw.origin.x + draw.size.width * 0.5;
  const CGFloat cy = draw.origin.y + draw.size.height * 0.5;
  draw.size.width *= zoom;
  draw.size.height *= zoom;
  draw.origin.x = cx - draw.size.width * 0.5;
  draw.origin.y = cy - draw.size.height * 0.5;

  CGContextRef ctx = [[NSGraphicsContext currentContext] CGContext];
  CGContextSetInterpolationQuality(ctx, kCGInterpolationNone);
  CGContextSaveGState(ctx);
  CGContextTranslateCTM(ctx, draw.origin.x, draw.origin.y + draw.size.height);
  CGContextScaleCTM(ctx, 1.0, -1.0);
  CGContextDrawImage(ctx, CGRectMake(0, 0, draw.size.width, draw.size.height), image);
  CGContextRestoreGState(ctx);
  CGImageRelease(image);
}

@end

@interface RFAppDelegate : NSObject <NSApplicationDelegate>
- (instancetype)initWithOptions:(const PlayerOptions&)options;
@end

@implementation RFAppDelegate {
  PlayerOptions _options;

  NSWindow* _window;
  RFFrameView* _frameView;
  NSButton* _playPauseButton;
  NSSlider* _scrubSlider;
  NSTextField* _timeLabel;
  NSTextField* _bufferLabel;
  NSButton* _loopCheck;
  NSSlider* _brightnessSlider;
  NSSlider* _contrastSlider;
  NSSlider* _saturationSlider;
  NSSlider* _tintSlider;
  NSSlider* _sharpnessSlider;
  NSSlider* _noiseSlider;
  NSSlider* _multipathGainSlider;
  NSSlider* _multipathDelaySlider;
  NSSlider* _jitterSlider;
  NSSlider* _rfDriftSlider;
  NSSlider* _amNonlinearSlider;
  NSSlider* _impulseNoiseSlider;
  NSSlider* _afcHuntSlider;
  NSSlider* _chromaFlutterSlider;
  NSSlider* _agcPumpSlider;
  NSSlider* _multipathEnsembleSlider;
  NSSlider* _humSlider;
  NSSlider* _ycCrosstalkSlider;
  NSSlider* _overscanSlider;
  NSSlider* _chromaDelaySlider;
  NSSlider* _hSyncNoiseSlider;
  NSSlider* _vSyncNoiseSlider;
  NSSlider* _burstNoiseSlider;
  NSSlider* _hLockInstabilitySlider;
  NSSlider* _vHoldInstabilitySlider;
  NSSlider* _burstInstabilitySlider;
  NSSlider* _vhsTrackingSlider;
  NSSlider* _vhsWrinkleSlider;
  NSSlider* _vhsHeadSwitchSlider;
  NSSlider* _vhsDropoutSlider;
  NSButton* _combCheck;
  NSButton* _dotCrawlCheck;
  NSButton* _resetButton;

  NSTimer* _timer;
  bool _playing;
  uint64_t _currentFrame;
  NSDate* _wallStart;
  uint64_t _wallStartFrame;

  rf2::NtrfReader _reader;
  rf2::NtrfHeader _header;
  rf2::NtscSignalConfig _signal;
  rf2::NtscEffects _effects;
  rf2::NtscDecoder _decoder;
  std::vector<float> _frame_ire;
  std::vector<uint8_t> _rgb;
  int _rgbWidth;
  int _rgbHeight;
  std::vector<int16_t> _audioPcm;

  AVAudioEngine* _audioEngine;
  AVAudioPlayerNode* _playerNode;
  AVAudioFormat* _audioFormat;
  uint64_t _audioStartSample;
  bool _updatingScrubber;
  bool _controlsDirty;

  bool _streamMode;
  std::atomic<bool> _streamReady;
  std::atomic<bool> _streamConnected;
  bool _streamPrimed;
  std::atomic<bool> _streamStopRequested;
  std::thread _streamThread;
  std::mutex _streamMutex;
  std::condition_variable _streamCv;
  std::deque<rf2::NtrfFrame> _streamQueue;
  size_t _streamPrebufferFrames;
  size_t _streamMaxQueueFrames;
  int _listenFd;
  int _clientFd;
  NSDate* _streamLastTick;
  double _streamAccumulator;
  bool _haveStreamFrame;
  rf2::NtrfFrame _lastStreamFrame;
  uint64_t _streamUnderflowCount;

  bool _ndiEnabled;
  bool _ndiInitialized;
#if defined(HAVE_NDI_SDK)
  NDIlib_send_instance_t _ndiSend;
#endif
  std::vector<uint8_t> _ndiBgra;
  float _overscanReveal;
}

- (instancetype)initWithOptions:(const PlayerOptions&)options {
  self = [super init];
  if (self) {
    _options = options;
    _playing = false;
    _currentFrame = 0;
    _wallStartFrame = 0;
    _audioStartSample = 0;
    _effects.controls().random_seed = 1;
    _rgbWidth = 0;
    _rgbHeight = 0;
    _updatingScrubber = false;
    _controlsDirty = false;
    _streamMode = !_options.listen_endpoint.empty();
    _streamReady = false;
    _streamConnected = false;
    _streamPrimed = false;
    _streamStopRequested = false;
    _streamPrebufferFrames = std::max<size_t>(1, _options.prebuffer_frames);
    _streamMaxQueueFrames = std::max<size_t>(_streamPrebufferFrames + 1, _options.max_buffer_frames);
    _listenFd = -1;
    _clientFd = -1;
    _streamLastTick = nil;
    _streamAccumulator = 0.0;
    _haveStreamFrame = false;
    _streamUnderflowCount = 0;
    _ndiEnabled = !_options.ndi_name.empty();
    _ndiInitialized = false;
    _overscanReveal = 0.0f;
#if defined(HAVE_NDI_SDK)
    _ndiSend = nullptr;
#endif
  }
  return self;
}

- (void)applicationDidFinishLaunching:(NSNotification*)notification {
  (void)notification;
  if (![self loadInput]) {
    [NSApp terminate:nil];
    return;
  }
  if (![self setupNdiOutput]) {
    [NSApp terminate:nil];
    return;
  }
  [self buildUi];
  [NSApp activateIgnoringOtherApps:YES];
  if (_streamMode) {
    [self startPlayback];
    [self updateTimelineUi];
  } else {
    [self renderFrameAtIndex:0];
  }
}

- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)sender {
  (void)sender;
  return YES;
}

- (void)applicationWillTerminate:(NSNotification*)notification {
  (void)notification;
  _streamStopRequested.store(true);
  if (_clientFd >= 0) {
    ::shutdown(_clientFd, SHUT_RDWR);
  }
  if (_listenFd >= 0) {
    ::shutdown(_listenFd, SHUT_RDWR);
  }
  _streamCv.notify_all();
  if (_streamThread.joinable()) {
    _streamThread.join();
  }
  [self shutdownNdiOutput];
}

- (bool)setupAudioEngineWithRate:(uint32_t)sampleRate {
  _audioEngine = [[AVAudioEngine alloc] init];
  _playerNode = [[AVAudioPlayerNode alloc] init];
  [_audioEngine attachNode:_playerNode];
  _audioFormat = [[AVAudioFormat alloc] initStandardFormatWithSampleRate:sampleRate channels:1];
  [_audioEngine connect:_playerNode to:_audioEngine.mainMixerNode format:_audioFormat];
  NSError* err = nil;
  if (![_audioEngine startAndReturnError:&err]) {
    std::cerr << "failed to start audio engine: "
              << (err ? err.localizedDescription.UTF8String : "unknown") << "\n";
    return false;
  }
  return true;
}

- (bool)setupNdiOutput {
  if (!_ndiEnabled) {
    return true;
  }
#if defined(HAVE_NDI_SDK)
  if (!NDIlib_initialize()) {
    std::cerr << "failed to initialize NDI runtime\n";
    return false;
  }

  NDIlib_send_create_t create_desc;
  create_desc.p_ndi_name = _options.ndi_name.c_str();
  create_desc.p_groups = _options.ndi_group.empty() ? nullptr : _options.ndi_group.c_str();
  create_desc.clock_video = true;
  create_desc.clock_audio = false;
  _ndiSend = NDIlib_send_create(&create_desc);
  if (!_ndiSend) {
    std::cerr << "failed to create NDI sender\n";
    NDIlib_destroy();
    return false;
  }
  _ndiInitialized = true;
  std::cerr << "[rf2] NDI output enabled as '" << _options.ndi_name << "'\n";
  return true;
#else
  std::cerr << "--ndi-name requested, but this build has no NDI SDK support\n";
  return false;
#endif
}

- (void)shutdownNdiOutput {
#if defined(HAVE_NDI_SDK)
  if (_ndiSend) {
    NDIlib_send_destroy(_ndiSend);
    _ndiSend = nullptr;
  }
  if (_ndiInitialized) {
    NDIlib_destroy();
    _ndiInitialized = false;
  }
#endif
}

- (void)sendNdiVideoCurrentFrame {
  if (!_ndiEnabled || _rgb.empty() || _rgbWidth <= 0 || _rgbHeight <= 0) {
    return;
  }
#if defined(HAVE_NDI_SDK)
  if (!_ndiSend) {
    return;
  }
  ScaleLetterboxRgbToBgra1920x1080(_rgb, _rgbWidth, _rgbHeight, 0.0f, &_ndiBgra);
  NDIlib_video_frame_v2_t video_frame;
  video_frame.xres = 1920;
  video_frame.yres = 1080;
  video_frame.FourCC = NDIlib_FourCC_type_BGRA;
  video_frame.frame_rate_N = 60;
  video_frame.frame_rate_D = 1;
  video_frame.picture_aspect_ratio = 16.0f / 9.0f;
  video_frame.frame_format_type = NDIlib_frame_format_type_progressive;
  video_frame.timecode = NDIlib_send_timecode_synthesize;
  video_frame.p_data = _ndiBgra.data();
  video_frame.line_stride_in_bytes = 1920 * 4;
  video_frame.p_metadata = nullptr;
  video_frame.timestamp = 0;
  NDIlib_send_send_video_v2(_ndiSend, &video_frame);
#endif
}

- (void)sendNdiAudioChunk:(const std::vector<int16_t>&)pcm {
  if (!_ndiEnabled || pcm.empty()) {
    return;
  }
#if defined(HAVE_NDI_SDK)
  if (!_ndiSend) {
    return;
  }
  NDIlib_audio_frame_interleaved_16s_t audio_frame;
  audio_frame.sample_rate = static_cast<int>(_header.audio_rate);
  audio_frame.no_channels = 1;
  audio_frame.no_samples = static_cast<int>(pcm.size());
  audio_frame.timecode = NDIlib_send_timecode_synthesize;
  audio_frame.reference_level = 0;
  audio_frame.p_data = const_cast<int16_t*>(pcm.data());
  NDIlib_util_send_send_audio_interleaved_16s(_ndiSend, &audio_frame);
#endif
}

- (void)sendNdiAudioForOfflineFrame:(uint64_t)frameIndex {
  if (!_ndiEnabled || _audioPcm.empty()) {
    return;
  }
  const uint64_t start = FrameToAudioSample(frameIndex, _header);
  const uint64_t end = FrameToAudioSample(frameIndex + 1, _header);
  if (end <= start || start >= _audioPcm.size()) {
    return;
  }
  const uint64_t bounded_end = std::min<uint64_t>(end, _audioPcm.size());
  std::vector<int16_t> chunk(static_cast<size_t>(bounded_end - start));
  std::copy(_audioPcm.begin() + static_cast<ptrdiff_t>(start),
            _audioPcm.begin() + static_cast<ptrdiff_t>(bounded_end),
            chunk.begin());
  [self sendNdiAudioChunk:chunk];
}

- (void)startStreamThread {
  _streamThread = std::thread([self]() {
    std::string error;
    const int listen_fd = OpenListenSocket(self->_options.listen_endpoint, &error);
    if (listen_fd < 0) {
      std::cerr << "[rf2] stream listen failed: " << error << "\n";
      return;
    }
    self->_listenFd = listen_fd;
    std::cerr << "[rf2] listening for NTRF stream on " << self->_options.listen_endpoint << "\n";

    while (!self->_streamStopRequested.load()) {
      sockaddr_storage addr {};
      socklen_t addrlen = sizeof(addr);
      const int client_fd = ::accept(listen_fd, reinterpret_cast<sockaddr*>(&addr), &addrlen);
      if (client_fd < 0) {
        if (self->_streamStopRequested.load()) {
          break;
        }
        continue;
      }
      self->_clientFd = client_fd;
      self->_streamConnected = true;
      std::cerr << "[rf2] stream client connected\n";

      rf2::NtrfHeader stream_header;
      std::string read_error;
      if (!ReadNtrfHeaderFromFd(client_fd, &stream_header, &read_error)) {
        std::cerr << "[rf2] stream header read failed: " << read_error << "\n";
        ::close(client_fd);
        self->_clientFd = -1;
        self->_streamConnected = false;
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(self->_streamMutex);
        self->_header = stream_header;
        self->_signal = rf2::NtscSignalConfig{};
        self->_signal.timing.lines_per_frame = self->_header.lines_per_frame;
        self->_signal.timing.samples_per_line = self->_header.samples_per_line;
        self->_signal.timing.active_start = self->_header.active_start;
        self->_signal.timing.active_samples = self->_header.active_samples;
        self->_streamQueue.clear();
        self->_streamReady = true;
        self->_streamPrimed = false;
        self->_streamAccumulator = 0.0;
      }
      self->_streamCv.notify_all();

      while (!self->_streamStopRequested.load()) {
        rf2::NtrfFrame frame;
        if (!ReadNtrfFrameFromFd(client_fd, &frame, &read_error)) {
          break;
        }

        std::unique_lock<std::mutex> lock(self->_streamMutex);
        self->_streamCv.wait(lock, [self]() {
          return self->_streamStopRequested.load() || self->_streamQueue.size() < self->_streamMaxQueueFrames;
        });
        if (self->_streamStopRequested.load()) {
          break;
        }
        self->_streamQueue.push_back(std::move(frame));
        lock.unlock();
        self->_streamCv.notify_all();
      }

      std::cerr << "[rf2] stream client disconnected\n";
      ::close(client_fd);
      self->_clientFd = -1;
      self->_streamConnected = false;
      {
        std::lock_guard<std::mutex> lock(self->_streamMutex);
        self->_streamReady = false;
        self->_streamPrimed = false;
        self->_streamQueue.clear();
        self->_streamCv.notify_all();
      }
    }

    if (listen_fd >= 0) {
      ::close(listen_fd);
      self->_listenFd = -1;
    }
  });
}

- (bool)loadInput {
  if (_streamMode) {
    _header = rf2::NtrfHeader{};
    _signal = rf2::NtscSignalConfig{};
    _signal.timing.lines_per_frame = _header.lines_per_frame;
    _signal.timing.samples_per_line = _header.samples_per_line;
    _signal.timing.active_start = _header.active_start;
    _signal.timing.active_samples = _header.active_samples;
    if (![self setupAudioEngineWithRate:_header.audio_rate]) {
      return false;
    }
    [self startStreamThread];
    return true;
  }

  std::string error;
  if (!_reader.Open(_options.input_path, &error)) {
    std::cerr << error << "\n";
    return false;
  }
  if (!_reader.BuildIndex(&error)) {
    std::cerr << error << "\n";
    return false;
  }
  _header = _reader.header();
  if (_reader.FrameCount() == 0) {
    std::cerr << "input has no frames\n";
    return false;
  }

  _signal = rf2::NtscSignalConfig{};
  _signal.timing.lines_per_frame = _header.lines_per_frame;
  _signal.timing.samples_per_line = _header.samples_per_line;
  _signal.timing.active_start = _header.active_start;
  _signal.timing.active_samples = _header.active_samples;

  uint64_t total_audio_samples = 0;
  for (const auto& e : _reader.index()) {
    total_audio_samples = std::max(total_audio_samples, e.audio_start_sample + e.audio_samples);
  }
  _audioPcm.assign(static_cast<size_t>(total_audio_samples), 0);

  rf2::NtrfFrame frame;
  for (size_t i = 0; i < _reader.FrameCount(); ++i) {
    if (!_reader.ReadFrameAt(i, &frame, &error)) {
      std::cerr << "failed to read audio for frame " << i << ": " << error << "\n";
      return false;
    }
    for (size_t s = 0; s < frame.audio_pcm.size(); ++s) {
      const uint64_t pos = frame.audio_start_sample + s;
      if (pos < _audioPcm.size()) {
        _audioPcm[static_cast<size_t>(pos)] = frame.audio_pcm[s];
      }
    }
  }

  return [self setupAudioEngineWithRate:_header.audio_rate];
}

- (NSSlider*)makeSliderWithMin:(double)min
                           max:(double)max
                         value:(double)value
                         frame:(NSRect)frame {
  NSSlider* slider = [[NSSlider alloc] initWithFrame:frame];
  slider.minValue = min;
  slider.maxValue = max;
  slider.doubleValue = value;
  slider.continuous = YES;
  slider.target = self;
  slider.action = @selector(controlChanged:);
  return slider;
}

- (NSTextField*)makeLabel:(NSString*)text frame:(NSRect)frame {
  NSTextField* label = [[NSTextField alloc] initWithFrame:frame];
  label.stringValue = text;
  label.bezeled = NO;
  label.drawsBackground = NO;
  label.editable = NO;
  label.selectable = NO;
  label.font = [NSFont systemFontOfSize:11];
  return label;
}

- (void)applyTooltipsForRoot:(NSView*)root {
  NSDictionary<NSString*, NSString*>* tips = @{
    @"Play" : @"Start or pause playback. In live mode this indicates stream status.",
    @"Timeline" : @"Seek position within offline files. Disabled for live socket playback.",
    @"Loop" : @"Replay offline input continuously when playback reaches the end.",
    @"Brightness" : @"DC luma offset after demodulation. Adjusts picture black/white placement.",
    @"Contrast" : @"Luma gain after demodulation. Higher values expand dynamic range.",
    @"Saturation" : @"Chroma gain after demodulation. Controls color intensity.",
    @"Tint" : @"Rotates chroma phase (hue) in degrees, like TV tint control.",
    @"Sharpness" : @"Adds luma edge enhancement similar to TV peaking.",
    @"Noise IRE" : @"Adds broadband composite noise in IRE units.",
    @"Ghost gain" : @"Single-path multipath reflection strength.",
    @"Ghost delay" : @"Delay of the primary ghost path in samples.",
    @"Jitter" : @"Random per-line timing jitter of horizontal sampling.",
    @"Chroma delay" : @"Relative chroma-vs-luma timing offset in pixels.",
    @"Overscan" : @"Zooms out to reveal normally hidden overscan and VBI areas.",
    @"H lock noise" : @"Noise on horizontal sync region that perturbs line lock timing.",
    @"V hold noise" : @"Noise on vertical sync that can induce sporadic frame vertical lock slips.",
    @"Burst noise" : @"Noise on color burst that can cause burst lock loss and color killer action.",
    @"VHS track" : @"Tracking mistrack. Negative biases errors toward top, positive toward bottom.",
    @"Wrinkle" : @"Moving tape wrinkle/dropout band with noisy edges and chroma smear.",
    @"Head switch" : @"Bottom head-switching seam noise and displacement.",
    @"Dropouts" : @"Tape oxide dropouts causing white hits and chroma/luma loss.",
    @"RF Drift" : @"Slow RF/IF gain and baseline wander with timing instability.",
    @"AM Nonlinear" : @"Asymmetric AM detector compression and knee behavior.",
    @"Impulse" : @"Short impulsive interference hits and bursty spikes.",
    @"AFC Hunt" : @"Horizontal AFC/line-lock hunting focused near line start (left-edge flagging).",
    @"Chroma Flutter" : @"Rapid chroma phase/amplitude flutter independent of luma.",
    @"AGC Pump" : @"Event-driven AGC compression pulses on bright/high-transient scenes.",
    @"MP Ensemble" : @"Multi-path echo cluster with mixed-polarity delayed replicas.",
    @"Hum" : @"Mains hum/rolling low-frequency interference bars and ripple.",
    @"Y/C Crosstalk" : @"Luma/chroma leakage creating dot-crawl and edge color contamination.",
    @"Comb Filter" : @"Enable line comb Y/C separation. Disable for notch-like decoding.",
    @"Dot Crawl" : @"Preserve line-sequential chroma artifacts instead of suppressing them.",
    @"Reset" : @"Restore all controls to defaults."
  };

  auto setTip = ^(NSView* view, NSString* key) {
    if (!view || !key) {
      return;
    }
    NSString* tip = [tips objectForKey:key];
    if (tip) {
      view.toolTip = tip;
    }
  };

  setTip(_playPauseButton, @"Play");
  setTip(_scrubSlider, @"Timeline");
  setTip(_timeLabel, @"Timeline");
  setTip(_loopCheck, @"Loop");
  setTip(_brightnessSlider, @"Brightness");
  setTip(_contrastSlider, @"Contrast");
  setTip(_saturationSlider, @"Saturation");
  setTip(_tintSlider, @"Tint");
  setTip(_sharpnessSlider, @"Sharpness");
  setTip(_noiseSlider, @"Noise IRE");
  setTip(_multipathGainSlider, @"Ghost gain");
  setTip(_multipathDelaySlider, @"Ghost delay");
  setTip(_jitterSlider, @"Jitter");
  setTip(_chromaDelaySlider, @"Chroma delay");
  setTip(_overscanSlider, @"Overscan");
  setTip(_hSyncNoiseSlider, @"H sync noise");
  setTip(_vSyncNoiseSlider, @"V sync noise");
  setTip(_burstNoiseSlider, @"Burst noise");
  setTip(_hLockInstabilitySlider, @"H PLL instability");
  setTip(_vHoldInstabilitySlider, @"V hold instability");
  setTip(_burstInstabilitySlider, @"Burst PLL instability");
  setTip(_vhsTrackingSlider, @"VHS track");
  setTip(_vhsWrinkleSlider, @"Wrinkle");
  setTip(_vhsHeadSwitchSlider, @"Head switch");
  setTip(_vhsDropoutSlider, @"Dropouts");
  setTip(_rfDriftSlider, @"RF Drift");
  setTip(_amNonlinearSlider, @"AM Nonlinear");
  setTip(_impulseNoiseSlider, @"Impulse");
  setTip(_afcHuntSlider, @"AFC Hunt");
  setTip(_chromaFlutterSlider, @"Chroma Flutter");
  setTip(_agcPumpSlider, @"AGC Pump");
  setTip(_multipathEnsembleSlider, @"MP Ensemble");
  setTip(_humSlider, @"Hum");
  setTip(_ycCrosstalkSlider, @"Y/C Crosstalk");
  setTip(_combCheck, @"Comb Filter");
  setTip(_dotCrawlCheck, @"Dot Crawl");
  setTip(_resetButton, @"Reset");

  for (NSView* subview in root.subviews) {
    if (![subview isKindOfClass:[NSTextField class]]) {
      continue;
    }
    NSTextField* label = static_cast<NSTextField*>(subview);
    NSString* tip = [tips objectForKey:label.stringValue];
    if (tip) {
      label.toolTip = tip;
    }
  }
}

- (void)buildUi {
  _window =
      [[NSWindow alloc] initWithContentRect:NSMakeRect(80, 80, 1100, 970)
                                   styleMask:(NSWindowStyleMaskTitled | NSWindowStyleMaskClosable |
                                              NSWindowStyleMaskMiniaturizable |
                                              NSWindowStyleMaskResizable)
                                     backing:NSBackingStoreBuffered
                                       defer:NO];
  _window.title = @"RF2 NTSC Player";
  [_window makeKeyAndOrderFront:nil];

  NSView* root = _window.contentView;
  _frameView = [[RFFrameView alloc] initWithFrame:NSMakeRect(10, 330, 1080, 630)];
  [root addSubview:_frameView];

  [root addSubview:[self makeLabel:@"RF Drift" frame:NSMakeRect(10, 302, 60, 16)]];
  _rfDriftSlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(70, 297, 250, 24)];
  [root addSubview:_rfDriftSlider];

  [root addSubview:[self makeLabel:@"AM Nonlinear" frame:NSMakeRect(345, 302, 80, 16)]];
  _amNonlinearSlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(425, 297, 250, 24)];
  [root addSubview:_amNonlinearSlider];

  [root addSubview:[self makeLabel:@"Impulse" frame:NSMakeRect(700, 302, 60, 16)]];
  _impulseNoiseSlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(760, 297, 250, 24)];
  [root addSubview:_impulseNoiseSlider];

  [root addSubview:[self makeLabel:@"AFC Hunt" frame:NSMakeRect(10, 272, 60, 16)]];
  _afcHuntSlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(75, 267, 230, 24)];
  [root addSubview:_afcHuntSlider];

  [root addSubview:[self makeLabel:@"Chroma Flutter" frame:NSMakeRect(340, 272, 90, 16)]];
  _chromaFlutterSlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(430, 267, 230, 24)];
  [root addSubview:_chromaFlutterSlider];

  [root addSubview:[self makeLabel:@"AGC Pump" frame:NSMakeRect(690, 272, 65, 16)]];
  _agcPumpSlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(755, 267, 230, 24)];
  [root addSubview:_agcPumpSlider];

  [root addSubview:[self makeLabel:@"MP Ensemble" frame:NSMakeRect(10, 242, 80, 16)]];
  _multipathEnsembleSlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(90, 237, 230, 24)];
  [root addSubview:_multipathEnsembleSlider];

  [root addSubview:[self makeLabel:@"Hum" frame:NSMakeRect(350, 242, 35, 16)]];
  _humSlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(385, 237, 230, 24)];
  [root addSubview:_humSlider];

  [root addSubview:[self makeLabel:@"Y/C Crosstalk" frame:NSMakeRect(645, 242, 85, 16)]];
  _ycCrosstalkSlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(730, 237, 230, 24)];
  [root addSubview:_ycCrosstalkSlider];

  _playPauseButton = [[NSButton alloc] initWithFrame:NSMakeRect(10, 210, 100, 30)];
  _playPauseButton.title = @"Play";
  _playPauseButton.bezelStyle = NSBezelStyleRounded;
  _playPauseButton.target = self;
  _playPauseButton.action = @selector(togglePlay:);
  [root addSubview:_playPauseButton];

  [root addSubview:[self makeLabel:@"Timeline" frame:NSMakeRect(130, 217, 70, 16)]];
  _scrubSlider = [self makeSliderWithMin:0
                                      max:static_cast<double>(std::max<size_t>(1, _reader.FrameCount()) - 1)
                                    value:0
                                    frame:NSMakeRect(195, 212, 620, 24)];
  _scrubSlider.target = self;
  _scrubSlider.action = @selector(scrubChanged:);
  _scrubSlider.continuous = YES;
  [root addSubview:_scrubSlider];

  _timeLabel = [self makeLabel:@"00:00.000 / 00:00.000" frame:NSMakeRect(825, 217, 180, 16)];
  _timeLabel.alignment = NSTextAlignmentRight;
  [root addSubview:_timeLabel];

  _bufferLabel = [self makeLabel:@"" frame:NSMakeRect(825, 200, 260, 14)];
  _bufferLabel.alignment = NSTextAlignmentRight;
  _bufferLabel.textColor = [NSColor secondaryLabelColor];
  [root addSubview:_bufferLabel];

  _loopCheck = [[NSButton alloc] initWithFrame:NSMakeRect(1010, 212, 80, 24)];
  _loopCheck.buttonType = NSButtonTypeSwitch;
  _loopCheck.title = @"Loop";
  _loopCheck.state = NSControlStateValueOff;
  [root addSubview:_loopCheck];

  if (_streamMode) {
    _playPauseButton.enabled = NO;
    _playPauseButton.title = @"Live";
    _scrubSlider.enabled = NO;
    _loopCheck.enabled = NO;
    _timeLabel.stringValue = @"00:00.000 / LIVE";
    _bufferLabel.stringValue =
        [NSString stringWithFormat:@"Buffer: 0/%zu (waiting)", _streamPrebufferFrames];
  } else {
    _bufferLabel.stringValue = @"";
  }

  [root addSubview:[self makeLabel:@"Brightness" frame:NSMakeRect(120, 175, 90, 16)]];
  _brightnessSlider = [self makeSliderWithMin:-0.5 max:0.5 value:0.0 frame:NSMakeRect(185, 170, 110, 24)];
  [root addSubview:_brightnessSlider];

  [root addSubview:[self makeLabel:@"Contrast" frame:NSMakeRect(320, 175, 70, 16)]];
  _contrastSlider = [self makeSliderWithMin:0.2 max:2.0 value:1.0 frame:NSMakeRect(385, 170, 160, 24)];
  [root addSubview:_contrastSlider];

  [root addSubview:[self makeLabel:@"Saturation" frame:NSMakeRect(560, 175, 80, 16)]];
  _saturationSlider = [self makeSliderWithMin:0.0 max:2.0 value:0.9 frame:NSMakeRect(645, 170, 160, 24)];
  [root addSubview:_saturationSlider];

  [root addSubview:[self makeLabel:@"Tint" frame:NSMakeRect(820, 175, 40, 16)]];
  _tintSlider = [self makeSliderWithMin:-45 max:45 value:0 frame:NSMakeRect(860, 170, 140, 24)];
  [root addSubview:_tintSlider];

  [root addSubview:[self makeLabel:@"Sharpness" frame:NSMakeRect(10, 135, 80, 16)]];
  _sharpnessSlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(80, 130, 180, 24)];
  [root addSubview:_sharpnessSlider];

  [root addSubview:[self makeLabel:@"Noise IRE" frame:NSMakeRect(280, 135, 80, 16)]];
  _noiseSlider = [self makeSliderWithMin:0 max:10 value:0 frame:NSMakeRect(360, 130, 180, 24)];
  [root addSubview:_noiseSlider];

  [root addSubview:[self makeLabel:@"Ghost gain" frame:NSMakeRect(550, 135, 80, 16)]];
  _multipathGainSlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(630, 130, 150, 24)];
  [root addSubview:_multipathGainSlider];

  [root addSubview:[self makeLabel:@"Ghost delay" frame:NSMakeRect(790, 135, 85, 16)]];
  _multipathDelaySlider = [self makeSliderWithMin:1 max:200 value:24 frame:NSMakeRect(875, 130, 120, 24)];
  [root addSubview:_multipathDelaySlider];

  [root addSubview:[self makeLabel:@"Jitter" frame:NSMakeRect(10, 95, 60, 16)]];
  _jitterSlider = [self makeSliderWithMin:0 max:2 value:0 frame:NSMakeRect(70, 90, 180, 24)];
  [root addSubview:_jitterSlider];

  [root addSubview:[self makeLabel:@"Chroma delay" frame:NSMakeRect(260, 95, 90, 16)]];
  _chromaDelaySlider = [self makeSliderWithMin:-6 max:6 value:0 frame:NSMakeRect(350, 90, 180, 24)];
  [root addSubview:_chromaDelaySlider];

  [root addSubview:[self makeLabel:@"Overscan" frame:NSMakeRect(820, 95, 65, 16)]];
  _overscanSlider = [self makeSliderWithMin:0 max:0.25 value:0 frame:NSMakeRect(885, 90, 145, 24)];
  [root addSubview:_overscanSlider];

  [root addSubview:[self makeLabel:@"H sync noise" frame:NSMakeRect(10, 55, 90, 16)]];
  _hSyncNoiseSlider = [self makeSliderWithMin:0 max:16 value:0 frame:NSMakeRect(100, 50, 170, 24)];
  [root addSubview:_hSyncNoiseSlider];

  [root addSubview:[self makeLabel:@"V sync noise" frame:NSMakeRect(290, 55, 90, 16)]];
  _vSyncNoiseSlider = [self makeSliderWithMin:0 max:16 value:0 frame:NSMakeRect(380, 50, 170, 24)];
  [root addSubview:_vSyncNoiseSlider];

  [root addSubview:[self makeLabel:@"Burst noise" frame:NSMakeRect(570, 55, 85, 16)]];
  _burstNoiseSlider = [self makeSliderWithMin:0 max:16 value:0 frame:NSMakeRect(655, 50, 170, 24)];
  [root addSubview:_burstNoiseSlider];

  [root addSubview:[self makeLabel:@"VHS track" frame:NSMakeRect(10, 35, 70, 16)]];
  _vhsTrackingSlider = [self makeSliderWithMin:-1 max:1 value:0 frame:NSMakeRect(80, 30, 170, 24)];
  [root addSubview:_vhsTrackingSlider];

  [root addSubview:[self makeLabel:@"Wrinkle" frame:NSMakeRect(280, 35, 55, 16)]];
  _vhsWrinkleSlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(335, 30, 170, 24)];
  [root addSubview:_vhsWrinkleSlider];

  [root addSubview:[self makeLabel:@"Head switch" frame:NSMakeRect(530, 35, 80, 16)]];
  _vhsHeadSwitchSlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(610, 30, 170, 24)];
  [root addSubview:_vhsHeadSwitchSlider];

  [root addSubview:[self makeLabel:@"Dropouts" frame:NSMakeRect(800, 35, 60, 16)]];
  _vhsDropoutSlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(860, 30, 170, 24)];
  [root addSubview:_vhsDropoutSlider];

  [root addSubview:[self makeLabel:@"H PLL" frame:NSMakeRect(10, 5, 50, 16)]];
  _hLockInstabilitySlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(55, 0, 170, 24)];
  [root addSubview:_hLockInstabilitySlider];

  [root addSubview:[self makeLabel:@"V hold" frame:NSMakeRect(250, 5, 50, 16)]];
  _vHoldInstabilitySlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(295, 0, 170, 24)];
  [root addSubview:_vHoldInstabilitySlider];

  [root addSubview:[self makeLabel:@"Burst PLL" frame:NSMakeRect(490, 5, 70, 16)]];
  _burstInstabilitySlider = [self makeSliderWithMin:0 max:1 value:0 frame:NSMakeRect(560, 0, 170, 24)];
  [root addSubview:_burstInstabilitySlider];

  _resetButton = [[NSButton alloc] initWithFrame:NSMakeRect(1035, 30, 55, 24)];
  _resetButton.title = @"Reset";
  _resetButton.bezelStyle = NSBezelStyleRounded;
  _resetButton.target = self;
  _resetButton.action = @selector(resetControls:);
  [root addSubview:_resetButton];

  _combCheck = [[NSButton alloc] initWithFrame:NSMakeRect(550, 90, 130, 24)];
  _combCheck.buttonType = NSButtonTypeSwitch;
  _combCheck.title = @"Comb Filter";
  _combCheck.state = NSControlStateValueOn;
  _combCheck.target = self;
  _combCheck.action = @selector(controlChanged:);
  [root addSubview:_combCheck];

  _dotCrawlCheck = [[NSButton alloc] initWithFrame:NSMakeRect(690, 90, 130, 24)];
  _dotCrawlCheck.buttonType = NSButtonTypeSwitch;
  _dotCrawlCheck.title = @"Dot Crawl";
  _dotCrawlCheck.state = NSControlStateValueOff;
  _dotCrawlCheck.target = self;
  _dotCrawlCheck.action = @selector(controlChanged:);
  [root addSubview:_dotCrawlCheck];
  [_frameView setOverscanReveal:0.0f];

  _timer = [NSTimer timerWithTimeInterval:(1.0 / 60.0)
                                   target:self
                                 selector:@selector(onTick:)
                                 userInfo:nil
                                  repeats:YES];
  [[NSRunLoop mainRunLoop] addTimer:_timer forMode:NSRunLoopCommonModes];
  [self applyTooltipsForRoot:root];
}

- (void)resetControls:(id)sender {
  (void)sender;
  _brightnessSlider.floatValue = 0.0f;
  _contrastSlider.floatValue = 1.0f;
  _saturationSlider.floatValue = 0.9f;
  _tintSlider.floatValue = 0.0f;
  _sharpnessSlider.floatValue = 0.0f;
  _noiseSlider.floatValue = 0.0f;
  _multipathGainSlider.floatValue = 0.0f;
  _multipathDelaySlider.floatValue = 24.0f;
  _jitterSlider.floatValue = 0.0f;
  _rfDriftSlider.floatValue = 0.0f;
  _amNonlinearSlider.floatValue = 0.0f;
  _impulseNoiseSlider.floatValue = 0.0f;
  _afcHuntSlider.floatValue = 0.0f;
  _chromaFlutterSlider.floatValue = 0.0f;
  _agcPumpSlider.floatValue = 0.0f;
  _multipathEnsembleSlider.floatValue = 0.0f;
  _humSlider.floatValue = 0.0f;
  _ycCrosstalkSlider.floatValue = 0.0f;
  _overscanSlider.floatValue = 0.0f;
  _chromaDelaySlider.floatValue = 0.0f;
  _hSyncNoiseSlider.floatValue = 0.0f;
  _vSyncNoiseSlider.floatValue = 0.0f;
  _burstNoiseSlider.floatValue = 0.0f;
  _hLockInstabilitySlider.floatValue = 0.0f;
  _vHoldInstabilitySlider.floatValue = 0.0f;
  _burstInstabilitySlider.floatValue = 0.0f;
  _vhsTrackingSlider.floatValue = 0.0f;
  _vhsWrinkleSlider.floatValue = 0.0f;
  _vhsHeadSwitchSlider.floatValue = 0.0f;
  _vhsDropoutSlider.floatValue = 0.0f;
  _combCheck.state = NSControlStateValueOn;
  _dotCrawlCheck.state = NSControlStateValueOff;
  [self controlChanged:nil];
}

- (void)updateTimelineUi {
  if (!_scrubSlider || !_timeLabel) {
    return;
  }

  if (_streamMode) {
    const double fps = static_cast<double>(_header.fps_num) / static_cast<double>(_header.fps_den);
    const double cur_s = fps > 0.0 ? (static_cast<double>(_currentFrame) / fps) : 0.0;
    const int cur_m = static_cast<int>(cur_s / 60.0);
    const int cur_sec = static_cast<int>(cur_s) % 60;
    const int cur_ms = static_cast<int>((cur_s - std::floor(cur_s)) * 1000.0);
    size_t qsize = 0;
    {
      std::lock_guard<std::mutex> lock(_streamMutex);
      qsize = _streamQueue.size();
    }
    NSString* state = @"playing";
    if (!_streamConnected.load()) {
      state = @"waiting";
    } else if (!_streamPrimed) {
      state = @"buffering";
    }
    _timeLabel.stringValue =
        [NSString stringWithFormat:@"%02d:%02d.%03d / LIVE (%zu)", cur_m, cur_sec, cur_ms, qsize];
    if (_bufferLabel) {
      _bufferLabel.stringValue =
          [NSString stringWithFormat:@"Buffer: %zu/%zu (max %zu)  %@  underflow:%llu",
                                     qsize,
                                     _streamPrebufferFrames,
                                     _streamMaxQueueFrames,
                                     state,
                                     static_cast<unsigned long long>(_streamUnderflowCount)];
    }
    return;
  }

  if (_reader.FrameCount() == 0) {
    return;
  }

  _updatingScrubber = true;
  _scrubSlider.doubleValue = static_cast<double>(_currentFrame);
  _updatingScrubber = false;

  const double fps = static_cast<double>(_header.fps_num) / static_cast<double>(_header.fps_den);
  const double cur_s = static_cast<double>(_currentFrame) / fps;
  const double total_s = static_cast<double>(_reader.FrameCount() - 1) / fps;

  const int cur_m = static_cast<int>(cur_s / 60.0);
  const int cur_sec = static_cast<int>(cur_s) % 60;
  const int cur_ms = static_cast<int>((cur_s - std::floor(cur_s)) * 1000.0);

  const int tot_m = static_cast<int>(total_s / 60.0);
  const int tot_sec = static_cast<int>(total_s) % 60;
  const int tot_ms = static_cast<int>((total_s - std::floor(total_s)) * 1000.0);

  _timeLabel.stringValue =
      [NSString stringWithFormat:@"%02d:%02d.%03d / %02d:%02d.%03d",
                                 cur_m,
                                 cur_sec,
                                 cur_ms,
                                 tot_m,
                                 tot_sec,
                                 tot_ms];
}

- (void)applyControlsFromUi {
  auto& dc = _decoder.controls();
  dc.brightness = _brightnessSlider.floatValue;
  dc.contrast = _contrastSlider.floatValue;
  dc.saturation = _saturationSlider.floatValue;
  dc.tint_degrees = _tintSlider.floatValue;
  dc.sharpness = _sharpnessSlider.floatValue;
  dc.chroma_delay_pixels = _chromaDelaySlider.floatValue;
  dc.overscan_reveal = _overscanSlider.floatValue;
  dc.comb_filter = (_combCheck.state == NSControlStateValueOn);
  dc.dot_crawl = (_dotCrawlCheck.state == NSControlStateValueOn);
  dc.h_lock_instability = _hLockInstabilitySlider.floatValue;
  dc.v_hold_instability = _vHoldInstabilitySlider.floatValue;
  dc.burst_lock_instability = _burstInstabilitySlider.floatValue;
  _overscanReveal = 0.0f;
  [_frameView setOverscanReveal:0.0f];

  auto& ec = _effects.controls();
  ec.noise_stddev_ire = _noiseSlider.floatValue;
  ec.multipath_gain = _multipathGainSlider.floatValue;
  ec.multipath_delay_samples = static_cast<uint32_t>(_multipathDelaySlider.integerValue);
  ec.line_time_jitter_samples = _jitterSlider.floatValue;
  ec.rf_drift = _rfDriftSlider.floatValue;
  ec.am_nonlinearity = _amNonlinearSlider.floatValue;
  ec.impulse_noise = _impulseNoiseSlider.floatValue;
  ec.afc_hunt = _afcHuntSlider.floatValue;
  ec.chroma_flutter = _chromaFlutterSlider.floatValue;
  ec.agc_pump = _agcPumpSlider.floatValue;
  ec.multipath_ensemble = _multipathEnsembleSlider.floatValue;
  ec.hum = _humSlider.floatValue;
  ec.yc_crosstalk = _ycCrosstalkSlider.floatValue;
  ec.h_sync_noise_ire = _hSyncNoiseSlider.floatValue;
  ec.v_sync_noise_ire = _vSyncNoiseSlider.floatValue;
  ec.burst_noise_ire = _burstNoiseSlider.floatValue;
  ec.vhs_tracking = _vhsTrackingSlider.floatValue;
  ec.vhs_wrinkle = _vhsWrinkleSlider.floatValue;
  ec.vhs_head_switch = _vhsHeadSwitchSlider.floatValue;
  ec.vhs_dropouts = _vhsDropoutSlider.floatValue;
}

- (void)renderFrameAtIndex:(uint64_t)frameIndex {
  if (_streamMode) {
    return;
  }
  if (_reader.FrameCount() == 0) {
    return;
  }
  if (frameIndex >= _reader.FrameCount()) {
    frameIndex = _reader.FrameCount() - 1;
  }
  rf2::NtrfFrame frame;
  std::string error;
  if (!_reader.ReadFrameAt(static_cast<size_t>(frameIndex), &frame, &error)) {
    std::cerr << "decode read error: " << error << "\n";
    return;
  }

  _effects.controls().random_seed = static_cast<uint32_t>(frameIndex + 1);
  _decoder.controls().random_seed = static_cast<uint32_t>(frameIndex + 1);
  [self applyControlsFromUi];

  // Convert int16 composite to IRE, apply effects, decode to RGB.
  _frame_ire.resize(frame.composite.size());
  for (size_t i = 0; i < frame.composite.size(); ++i) {
    _frame_ire[i] = rf2::SampleToIre(frame.composite[i]);
  }
  _effects.Apply(&_frame_ire);
  _decoder.Decode(_frame_ire, &_rgb);

  _rgbWidth = 720;
  _rgbHeight = 480;
  [_frameView updateFrameRgb:_rgb width:_rgbWidth height:_rgbHeight];
  _currentFrame = frameIndex;
  [self updateTimelineUi];
}

- (void)seekToFrame:(uint64_t)frameIndex restartPlayback:(BOOL)restartPlayback {
  if (_streamMode) {
    return;
  }
  const bool wasPlaying = _playing;
  if (restartPlayback && wasPlaying) {
    [self stopPlayback];
  }
  [self renderFrameAtIndex:frameIndex];
  if (restartPlayback && wasPlaying) {
    [self startPlayback];
  }
}

- (bool)popStreamFrame:(rf2::NtrfFrame*)out_frame {
  std::lock_guard<std::mutex> lock(_streamMutex);
  if (_streamQueue.empty()) {
    return false;
  }
  *out_frame = std::move(_streamQueue.front());
  _streamQueue.pop_front();
  _streamCv.notify_all();
  return true;
}

- (void)renderStreamFrame:(const rf2::NtrfFrame&)frame {
  _effects.controls().random_seed = static_cast<uint32_t>(frame.frame_index + 1);
  _decoder.controls().random_seed = static_cast<uint32_t>(frame.frame_index + 1);
  [self applyControlsFromUi];

  _frame_ire.resize(frame.composite.size());
  for (size_t i = 0; i < frame.composite.size(); ++i) {
    _frame_ire[i] = rf2::SampleToIre(frame.composite[i]);
  }
  _effects.Apply(&_frame_ire);
  _decoder.Decode(_frame_ire, &_rgb);

  _rgbWidth = 720;
  _rgbHeight = 480;
  [_frameView updateFrameRgb:_rgb width:_rgbWidth height:_rgbHeight];
  _currentFrame = frame.frame_index;
  _lastStreamFrame = frame;
  _haveStreamFrame = true;
  [self updateTimelineUi];
}

- (void)scheduleAudioChunk:(const std::vector<int16_t>&)pcm {
  if (pcm.empty()) {
    return;
  }
  if (!_options.mute) {
    const AVAudioFrameCount frames = static_cast<AVAudioFrameCount>(pcm.size());
    AVAudioPCMBuffer* buffer =
        [[AVAudioPCMBuffer alloc] initWithPCMFormat:_audioFormat frameCapacity:frames];
    buffer.frameLength = frames;
    float* channel = buffer.floatChannelData[0];
    for (AVAudioFrameCount i = 0; i < frames; ++i) {
      channel[i] = static_cast<float>(pcm[static_cast<size_t>(i)]) / 32768.0f;
    }
    [_playerNode scheduleBuffer:buffer completionHandler:nil];
    if (!_playerNode.isPlaying) {
      [_playerNode play];
    }
  }
  [self sendNdiAudioChunk:pcm];
}

- (AVAudioPCMBuffer*)makeAudioBufferFromSample:(uint64_t)startSample {
  if (startSample >= _audioPcm.size()) {
    return nil;
  }
  const AVAudioFrameCount frames = static_cast<AVAudioFrameCount>(_audioPcm.size() - startSample);
  AVAudioPCMBuffer* buffer = [[AVAudioPCMBuffer alloc] initWithPCMFormat:_audioFormat frameCapacity:frames];
  buffer.frameLength = frames;
  float* channel = buffer.floatChannelData[0];
  const int16_t* src = _audioPcm.data() + startSample;
  for (AVAudioFrameCount i = 0; i < frames; ++i) {
    channel[i] = static_cast<float>(src[i]) / 32768.0f;
  }
  return buffer;
}

- (void)startPlayback {
  if (_playing) {
    return;
  }
  _playing = true;
  _wallStart = [NSDate date];
  _wallStartFrame = _currentFrame;
  _playPauseButton.title = @"Pause";

  if (_streamMode) {
    _playPauseButton.title = @"Live";
    _streamPrimed = false;
    _streamAccumulator = 0.0;
    _streamLastTick = [NSDate date];
    if (!_options.mute && !_playerNode.isPlaying) {
      [_playerNode play];
    }
    return;
  }

  if (_options.mute || _audioPcm.empty()) {
    return;
  }
  _audioStartSample = FrameToAudioSample(_currentFrame, _header);
  AVAudioPCMBuffer* buffer = [self makeAudioBufferFromSample:_audioStartSample];
  if (!buffer) {
    return;
  }
  [_playerNode stop];
  [_playerNode scheduleBuffer:buffer completionHandler:nil];
  [_playerNode play];
}

- (void)stopPlayback {
  if (!_playing) {
    return;
  }
  _playing = false;
  _playPauseButton.title = @"Play";
  if (!_options.mute) {
    [_playerNode stop];
  }
}

- (void)togglePlay:(id)sender {
  (void)sender;
  if (_streamMode) {
    return;
  }
  if (_playing) {
    [self stopPlayback];
  } else {
    [self startPlayback];
  }
}

- (void)controlChanged:(id)sender {
  (void)sender;
  _overscanReveal = 0.0f;
  [_frameView setOverscanReveal:0.0f];
  if (_playing) {
    // Avoid heavy per-event decode while dragging sliders during playback.
    _controlsDirty = true;
    return;
  }
  if (_streamMode) {
    if (_haveStreamFrame) {
      [self renderStreamFrame:_lastStreamFrame];
    }
    return;
  }
  [self renderFrameAtIndex:_currentFrame];
}

- (void)scrubChanged:(id)sender {
  (void)sender;
  if (_streamMode || _updatingScrubber || _reader.FrameCount() == 0) {
    return;
  }
  const uint64_t target =
      static_cast<uint64_t>(std::llround(_scrubSlider.doubleValue));
  [self seekToFrame:target restartPlayback:_playing ? YES : NO];
}

- (void)onTick:(NSTimer*)timer {
  (void)timer;
  if (!_playing) {
    if (_streamMode) {
      [self updateTimelineUi];
    }
    [self sendNdiVideoCurrentFrame];
    return;
  }

  if (_streamMode) {
    if (!_streamReady.load()) {
      [self updateTimelineUi];
      [self sendNdiVideoCurrentFrame];
      return;
    }

    size_t qsize = 0;
    {
      std::lock_guard<std::mutex> lock(_streamMutex);
      qsize = _streamQueue.size();
    }

    if (!_streamPrimed) {
      if (qsize < _streamPrebufferFrames) {
        [self updateTimelineUi];
        [self sendNdiVideoCurrentFrame];
        return;
      }
      _streamPrimed = true;
      _streamAccumulator = 0.0;
      _streamLastTick = [NSDate date];
    }

    NSDate* now = [NSDate date];
    NSTimeInterval dt = [now timeIntervalSinceDate:_streamLastTick];
    _streamLastTick = now;
    const double fps = static_cast<double>(_header.fps_num) / static_cast<double>(_header.fps_den);
    _streamAccumulator += std::max(0.0, dt) * fps;

    int frames_due = static_cast<int>(std::floor(_streamAccumulator));
    bool underflow = false;
    for (int i = 0; i < frames_due; ++i) {
      rf2::NtrfFrame frame;
      if (![self popStreamFrame:&frame]) {
        underflow = true;
        break;
      }
      [self renderStreamFrame:frame];
      [self scheduleAudioChunk:frame.audio_pcm];
      _streamAccumulator -= 1.0;
    }

    if (underflow) {
      ++_streamUnderflowCount;
      _streamPrimed = false;
      _streamAccumulator = 0.0;
    }

    if (_controlsDirty && _haveStreamFrame) {
      _controlsDirty = false;
      [self renderStreamFrame:_lastStreamFrame];
    } else {
      [self updateTimelineUi];
    }
    [self sendNdiVideoCurrentFrame];
    return;
  }

  uint64_t nextFrame = _currentFrame;
  bool usedAudioClock = false;
  if (!_options.mute && !_audioPcm.empty()) {
    AVAudioTime* nodeTime = _playerNode.lastRenderTime;
    if (nodeTime) {
      AVAudioTime* playerTime = [_playerNode playerTimeForNodeTime:nodeTime];
      if (playerTime && playerTime.sampleTime >= 0) {
        const uint64_t sample = _audioStartSample + static_cast<uint64_t>(playerTime.sampleTime);
        nextFrame = AudioSampleToFrame(sample, _header);
        usedAudioClock = true;
      }
    }
  }

  if (!usedAudioClock) {
    NSTimeInterval elapsed = [[NSDate date] timeIntervalSinceDate:_wallStart];
    const double fps = static_cast<double>(_header.fps_num) / static_cast<double>(_header.fps_den);
    nextFrame = _wallStartFrame + static_cast<uint64_t>(std::floor(elapsed * fps));
  }

  if (nextFrame >= _reader.FrameCount()) {
    if (_loopCheck.state == NSControlStateValueOn && _reader.FrameCount() > 0) {
      [self seekToFrame:0 restartPlayback:YES];
      return;
    }
    [self stopPlayback];
    nextFrame = _reader.FrameCount() - 1;
  }
  const bool frame_advanced = (nextFrame != _currentFrame);
  if (_controlsDirty || nextFrame != _currentFrame) {
    _controlsDirty = false;
    [self renderFrameAtIndex:nextFrame];
  }
  if (frame_advanced) {
    [self sendNdiAudioForOfflineFrame:nextFrame];
  }
  [self sendNdiVideoCurrentFrame];
}

@end

int main(int argc, const char** argv) {
  PlayerOptions options;
  if (!ParseArgs(argc, argv, &options)) {
    PrintUsage(argv[0]);
    return 1;
  }

  if (options.export_only && options.input_path.empty()) {
    std::cerr << "--export-only requires --input <file.ntrf>\n";
    return 1;
  }

  if (!options.export_path.empty() && options.export_only) {
    return ExportNtrfToMp4(options) ? 0 : 1;
  }

  @autoreleasepool {
    NSApplication* app = [NSApplication sharedApplication];
    [app setActivationPolicy:NSApplicationActivationPolicyRegular];
    BuildMainMenu();
    RFAppDelegate* delegate = [[RFAppDelegate alloc] initWithOptions:options];
    app.delegate = delegate;
    [app run];
  }
  return 0;
}
