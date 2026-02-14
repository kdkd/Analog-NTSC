// Copyright (c) 2026 Kevin Day
// SPDX-License-Identifier: BSD-2-Clause

#include "AEConfig.h"
#include "AE_EffectVers.h"
#include "AE_General.r"

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
			PF_PLUG_IN_SUBVERS
		},
		AE_Effect_Version {
			524288	/* 1.0.0 */
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
