# Platform detection
ifeq ($(OS),Windows_NT)
PLATFORM := windows
else
SHELL := /bin/zsh
PLATFORM := macos
endif

BUILD_DIR := build
OBJ_DIR   := $(BUILD_DIR)/obj
BIN_DIR   := $(BUILD_DIR)/bin

# ---------- NTSC library sources (platform-independent) --------------------
NTSC_CPP := \
	src/common/ntsc_signal.cpp \
	src/common/ntsc_params.cpp \
	src/common/ntsc_effects.cpp \
	src/common/ntsc_encoder.cpp \
	src/common/ntsc_decoder.cpp \
	src/common/ntsc_processor.cpp

NTSC_OBJS := $(NTSC_CPP:src/%.cpp=$(OBJ_DIR)/%.o)

# ---------- Plugin sources (shared across platforms) -----------------------
PLUGIN_SRC := src/plugin/rf2_ntsc.cpp
PLUGIN_OBJ := $(OBJ_DIR)/plugin/rf2_ntsc.o
PLUGIN_R   := src/plugin/rf2_ntsc.r

# ==========================================================================
ifeq ($(PLATFORM),windows)
# ===== Windows (MinGW-w64) — plugin only ==================================

CXX      := g++
CXXFLAGS := -std=c++20 -O3 -march=x86-64-v2 -ffast-math -flto \
            -Wall -Wextra -Wpedantic -Iinclude
LDFLAGS  := -flto -static-libgcc -static-libstdc++

AE_SDK_DIR := 3rdparty/windows/AfterEffectsSDK_25.6_61_win/ae25.6_61.64bit.AfterEffectsSDK
PR_SDK_DIR := 3rdparty/windows/Premiere Pro 26.0 C++ SDK
PIPL_TOOL  := $(AE_SDK_DIR)/Examples/Resources/PiPLtool.exe

SDK_INCLUDES := \
	-I"$(AE_SDK_DIR)/Examples/Headers" \
	-I"$(AE_SDK_DIR)/Examples/Headers/SP" \
	-I"$(AE_SDK_DIR)/Examples/Util" \
	-I"$(PR_SDK_DIR)/Examples/Headers"

# MinGW compatibility: _WINDOWS triggers platform detection in the Adobe SDK
# headers (SPConfig.h sets WIN_ENV, which enables Windows type definitions).
# __int64 is an MSVC keyword that g++ doesn't recognize in strict C++ mode.
SDK_COMPAT := -D_WINDOWS -DMSWindows -D"__int64=long long" -Wno-unknown-pragmas

# PiPL resource pipeline intermediates
PIPL_RR  := $(OBJ_DIR)/plugin/rf2_ntsc.rr
PIPL_RRC := $(OBJ_DIR)/plugin/rf2_ntsc.rrc
PIPL_RC  := $(OBJ_DIR)/plugin/rf2_ntsc_pipl.rc
PIPL_RES := $(OBJ_DIR)/plugin/rf2_ntsc_pipl.o

PLUGIN_DLL       := $(BUILD_DIR)/plugin/Analog_NTSC.prm
PLUGIN_INSTALLER := $(BUILD_DIR)/Analog-NTSC-Setup.exe

.PHONY: plugin plugin-release clean

plugin: $(PLUGIN_DLL)

# Self-extracting installer (requires NSIS: pacman -S mingw-w64-x86_64-nsis)
plugin-release: $(PLUGIN_INSTALLER)

$(PLUGIN_DLL): $(NTSC_OBJS) $(PLUGIN_OBJ) $(PIPL_RES)
	@mkdir -p "$(@D)"
	$(CXX) -shared -o "$@" $^ $(LDFLAGS)

$(PLUGIN_INSTALLER): $(PLUGIN_DLL)
	makensis src/plugin/installer.nsi

# PiPL resource pipeline: .r -> preprocess -> PiPLTool -> preprocess -> windres
$(PIPL_RR): $(PLUGIN_R)
	@mkdir -p $(@D)
	$(CXX) -E -P -w -x c $(SDK_INCLUDES) $(SDK_COMPAT) \
		-I"$(AE_SDK_DIR)/Examples/Resources" \
		$< > $@

$(PIPL_RRC): $(PIPL_RR)
	"$(PIPL_TOOL)" $< $@

# PiPLtool emits MSVC rc.exe syntax; fix three issues for GNU windres:
#   1. Remove DISCARDABLE (unsupported for custom resource types)
#   2. Strip L suffix from integer literals (0x65564552L -> 0x65564552)
#   3. Add missing commas between data values
$(PIPL_RC): $(PIPL_RRC)
	$(CXX) -E -P -DMSWindows -w -x c $< | \
	sed -e 's/DISCARDABLE//' \
	    -e 's/\([0-9a-fA-F]\)L\([, ]\)/\1\2/g' \
	    -e 's/\([0-9a-fA-F]\)L$$/\1/' \
	    -e '/^ /{ /,[[:space:]]*$$/!s/$$/,/; }' \
	    > $@

$(PIPL_RES): $(PIPL_RC)
	windres $< -o $@

# Plugin source (with SDK includes)
$(PLUGIN_OBJ): $(PLUGIN_SRC)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(SDK_INCLUDES) $(SDK_COMPAT) \
		-fvisibility=hidden -Wno-multichar \
		-Wno-unused-parameter -Wno-missing-field-initializers \
		-c $< -o $@

# NTSC library objects (explicit subdirectory rule for Windows Make compatibility)
$(OBJ_DIR)/common/%.o: src/common/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

else
# ===== macOS (clang++) =====================================================

CXX    := clang++
OBJCXX := clang++

CXXFLAGS    := -std=c++20 -O3 -march=native -ffast-math -flto -Wall -Wextra -Wpedantic -Iinclude
OBJCXXFLAGS := $(CXXFLAGS) -fobjc-arc
LDFLAGS     := -lcompression -flto

FRAMEWORKS_APP := \
	-framework Cocoa \
	-framework Metal \
	-framework MetalKit \
	-framework AVFoundation \
	-framework AudioToolbox \
	-framework CoreAudio \
	-framework QuartzCore

# macOS COMMON includes ntrf_container (uses Apple compression library)
COMMON_CPP := src/common/ntrf_container.cpp $(NTSC_CPP)
COMMON_OBJS := $(COMMON_CPP:src/%.cpp=$(OBJ_DIR)/%.o)

ENCODER_CPP  := src/encoder/main.cpp
ENCODER_OBJS := $(ENCODER_CPP:src/%.cpp=$(OBJ_DIR)/%.o)

APP_MM   := src/app/main.mm
APP_OBJS := $(APP_MM:src/%.mm=$(OBJ_DIR)/%.o)

ENCODER_BIN := $(BIN_DIR)/ntsc-encode
PLAYER_BIN  := $(BIN_DIR)/ntsc-player

NDI_SDK_DIR := ndisdk
NDI_APP_CPPFLAGS :=
NDI_APP_LDFLAGS :=
ifneq ($(wildcard $(NDI_SDK_DIR)/include/Processing.NDI.Lib.h),)
NDI_APP_CPPFLAGS += -DHAVE_NDI_SDK=1 -I$(NDI_SDK_DIR)/include
NDI_APP_LDFLAGS  += -L$(NDI_SDK_DIR)/lib/macOS -lndi -Wl,-rpath,@executable_path/../../ndisdk/lib/macOS
endif

# ---------- Adobe plugin SDK paths (macOS) ---------------------------------
AE_SDK_DIR := 3rdparty/macos/AfterEffectsSDK_25.6_61_mac/ae25.6_61.64bit.AfterEffectsSDK
PR_SDK_DIR := 3rdparty/macos/Premiere Pro 26.0 C++ SDK

PLUGIN_STAMP := $(BUILD_DIR)/plugin/.stamp

# ---------- targets --------------------------------------------------------
.PHONY: all clean run-player plugin plugin-release

all: $(ENCODER_BIN) $(PLAYER_BIN)

$(ENCODER_BIN): $(COMMON_OBJS) $(ENCODER_OBJS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(PLAYER_BIN): $(COMMON_OBJS) $(APP_OBJS) | $(BIN_DIR)
	$(OBJCXX) $(OBJCXXFLAGS) $(NDI_APP_CPPFLAGS) $^ -o $@ $(LDFLAGS) $(NDI_APP_LDFLAGS) $(FRAMEWORKS_APP)

$(OBJ_DIR)/%.o: src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/%.o: src/%.mm
	@mkdir -p $(dir $@)
	$(OBJCXX) $(OBJCXXFLAGS) $(NDI_APP_CPPFLAGS) -c $< -o $@

$(BIN_DIR):
	@mkdir -p $(BIN_DIR)

run-player: $(PLAYER_BIN)
	$(PLAYER_BIN)

# ---------- Premiere Pro plugin (macOS) ------------------------------------
plugin: $(PLUGIN_STAMP)

$(PLUGIN_OBJ): $(PLUGIN_SRC)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) \
		-I'$(AE_SDK_DIR)/Examples/Headers' \
		-I'$(AE_SDK_DIR)/Examples/Headers/SP' \
		-I'$(AE_SDK_DIR)/Examples/Util' \
		-I'$(PR_SDK_DIR)/Examples/Headers' \
		-fvisibility=hidden -Wno-four-char-constants \
		-Wno-unused-parameter -Wno-missing-field-initializers \
		-c $< -o $@

$(PLUGIN_STAMP): $(COMMON_OBJS) $(PLUGIN_OBJ) $(PLUGIN_R) src/plugin/Info.plist
	@mkdir -p '$(BUILD_DIR)/plugin/Analog NTSC.plugin/Contents/MacOS'
	@mkdir -p '$(BUILD_DIR)/plugin/Analog NTSC.plugin/Contents/Resources'
	$(CXX) -dynamiclib -o '$(BUILD_DIR)/plugin/Analog NTSC.plugin/Contents/MacOS/Analog NTSC' \
		$(COMMON_OBJS) $(PLUGIN_OBJ) $(LDFLAGS)
	xcrun Rez -d AE_OS_MAC \
		-I '$(AE_SDK_DIR)/Examples/Headers' \
		-I '$(AE_SDK_DIR)/Examples/Resources' \
		-I '$(PR_SDK_DIR)/Examples/Headers' \
		$(PLUGIN_R) -o '$(BUILD_DIR)/plugin/Analog NTSC.plugin/Contents/Resources/Analog NTSC.rsrc'
	cp src/plugin/Info.plist '$(BUILD_DIR)/plugin/Analog NTSC.plugin/Contents/Info.plist'
	@touch $@

# ---------- Plugin installer DMG -------------------------------------------
PLUGIN_PKG := $(BUILD_DIR)/Install Analog NTSC.pkg
PLUGIN_DMG := $(BUILD_DIR)/Analog-NTSC.dmg
PLUGIN_INSTALL_DIR := /Library/Application Support/Adobe/Common/Plug-ins/7.0/MediaCore

plugin-release: $(PLUGIN_DMG)

$(PLUGIN_DMG): $(PLUGIN_STAMP)
	@rm -rf '$(BUILD_DIR)/pkg-root' '$(BUILD_DIR)/dmg-stage'
	@mkdir -p '$(BUILD_DIR)/pkg-root'
	@cp -R '$(BUILD_DIR)/plugin/Analog NTSC.plugin' '$(BUILD_DIR)/pkg-root/'
	pkgbuild --root '$(BUILD_DIR)/pkg-root' \
		--identifier day.kevin.analog-ntsc \
		--version 1.0 \
		--install-location '$(PLUGIN_INSTALL_DIR)' \
		'$(PLUGIN_PKG)'
	@mkdir -p '$(BUILD_DIR)/dmg-stage'
	@mv '$(PLUGIN_PKG)' '$(BUILD_DIR)/dmg-stage/'
	hdiutil create -volname 'Analog NTSC' \
		-srcfolder '$(BUILD_DIR)/dmg-stage' \
		-ov -format UDZO \
		'$(PLUGIN_DMG)'
	@rm -rf '$(BUILD_DIR)/pkg-root' '$(BUILD_DIR)/dmg-stage'
	@echo '=> $(PLUGIN_DMG)'

endif
# ==========================================================================

clean:
	rm -rf $(BUILD_DIR)
