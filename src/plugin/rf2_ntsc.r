// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#include "AEConfig.h"
#include "AE_EffectVers.h"
#include "AE_General.r"

// Use a backward-compatible spec subversion so the plugin loads on older
// hosts (AE 2023 / 23.x and later).  The SDK 25.6 header defines
// PF_PLUG_IN_SUBVERS as 29 which is too new for pre-2025 hosts.
// Subversion 25 corresponds roughly to the AE 2022-era API and our
// plugin does not use any features introduced after that.
#define PF_PLUG_IN_SUBVERS_COMPAT 25

resource 'PiPL' (16000) {
	{
		Kind {
			AEEffect
		},
		Name {
			"Analog NTSC"
		},
		Category {
			"Stylize"
		},
#ifdef AE_OS_WIN
		CodeWin64X86 {"EffectMain"},
#else
		CodeMacARM64 {"EffectMain"},
		CodeMacIntel64 {"EffectMain"},
#endif
		AE_PiPL_Version {
			2,
			0
		},
		AE_Effect_Spec_Version {
			PF_PLUG_IN_VERSION,
			PF_PLUG_IN_SUBVERS_COMPAT
		},
		AE_Effect_Version {
			524289	/* 1.0.0 (build 1) */
		},
		AE_Effect_Info_Flags {
			0
		},
		AE_Effect_Global_OutFlags {
			0x00000400	/* PF_OutFlag_USE_OUTPUT_EXTENT */
		},
		AE_Effect_Global_OutFlags_2 {
			0x00000000
		},
		AE_Effect_Match_Name {
			"Analog NTSC Composite"
		},
		AE_Reserved_Info {
			8
		}
	}
};
