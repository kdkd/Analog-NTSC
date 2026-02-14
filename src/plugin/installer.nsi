; Copyright (c) 2026 Kevin Day
; SPDX-License-Identifier: BSD-2-Clause

; Analog NTSC — Premiere Pro plugin installer
; Requires NSIS 3.x  (MSYS2: pacman -S mingw-w64-x86_64-nsis)

; NSIS resolves File paths relative to the script location. Change to the
; project root (this script lives in src\plugin\).
!cd "..\.."

!include "MUI2.nsh"

Name "Analog NTSC"
OutFile "build\Analog-NTSC-Setup.exe"
InstallDir "$PROGRAMFILES64\Adobe\Common\Plug-ins\7.0\MediaCore"
RequestExecutionLevel admin

; ---- Pages ----------------------------------------------------------------
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_PAGE_FINISH

!insertmacro MUI_LANGUAGE "English"

; ---- Install section ------------------------------------------------------
Section "Analog NTSC Plugin"
  SetOutPath $INSTDIR
  File "build\plugin\Analog_NTSC.prm"
SectionEnd

; ---- Uninstaller ----------------------------------------------------------
Section "-Uninstaller"
  WriteUninstaller "$INSTDIR\Analog-NTSC-Uninstall.exe"
SectionEnd

Section "Uninstall"
  Delete "$INSTDIR\Analog_NTSC.prm"
  Delete "$INSTDIR\Analog-NTSC-Uninstall.exe"
SectionEnd
