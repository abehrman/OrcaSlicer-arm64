# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Snapmaker_Orca is Snapmaker's fork of OrcaSlicer (itself a Bambu Studio fork): a C++17 FDM slicer with a wxWidgets GUI, built with CMake. The full source tree is present, including the GUI in `src/slic3r/GUI/`. Vendored dependency sources live in `deps/` and `deps_src/`; don't edit them without mirroring upstream. `AGENTS.md` has the repo's contributor guidelines (clang-format: 4-space indent, 140 columns; sentence-style commit subjects) and `tests/CLAUDE.md` has Catch2 rules (read it before writing tests; it still says "OrcaSlicer" but applies here).

## Build

Dependencies are built once, separately from the app, then the app is built against them.

```bash
./build_release_macos.sh          # deps + slicer; -d deps only, -s slicer only, -x Ninja, -a arm64|x86_64|universal, -b debug
./build_linux.sh -dsi             # -u installs system packages first; -d deps, -s slicer, -i AppImage
build_release_vs2022.bat          # Windows; args: deps | slicer | debug
```

- macOS slicer builds go to `build/<arch>/` (e.g. `build/arm64`, Ninja Multi-Config, so pass `--config Release`). Deps go to `deps/build/<arch>/`.
- `build_release_macos.sh -s` re-runs cmake then builds; it also runs `scripts/run_gettext.sh` and packages the `.app`. For a fast rebuild in an existing tree, use `cmake --build build/arm64 --config Release --target Snapmaker_Orca` (or a test target below).
- Tests are off by default: configure with `-DBUILD_TESTS=ON` (the existing `build/arm64` has it on). `-DSLIC3R_GUI=OFF` gives a headless build for tests only. `-DORCA_TOOLS=ON` adds `Snapmaker_Orca_profile_validator`.

## Tests

Catch2 v3 (fetched via `cmake/catch2.cmake`). Executables: `libslic3r_tests`, `fff_print_tests`, `sla_print_tests`, `libnest2d_tests`, `slic3rutils_tests`; the `tests` target builds them all.

```bash
cmake --build build/arm64 --config Release --target fff_print_tests
ctest --test-dir build/arm64 -C Release --output-on-failure
ctest --test-dir build/arm64 -C Release -R "prime"                      # filter by name
build/arm64/tests/fff_print/Release/fff_print_tests.app/Contents/MacOS/fff_print_tests "[tag]"   # run Catch2 directly (macOS bundle path)
python3 scripts/run_tests.py                                              # configure + build + ctest wrapper
```

- Run tests in **Release**. Debug fails a pre-existing placeholder check.
- The G-code processor drops a repeated `F` value, so regexes over generated G-code must make ` F300` optional.
- The `test-linux` CI workflow is manual-dispatch only.

## Architecture

- `src/libslic3r/` is the platform-independent core: `Print`/`PrintObject`/`Layer` drive slicing, `GCode.cpp` + `GCode/` emit G-code, `PrintConfig.cpp` defines every print/filament/printer setting, and `Preset*.cpp` handle profile loading and inheritance. `Fill/`, `Support/`, `Arachne/`, `Geometry/`, `Format/` (3MF/STEP/OBJ/AMF/STL) are subsystems.
- `src/slic3r/` is the application: `GUI/` (wx widgets, 3D scene, plater, preset tabs), `Utils/` (printer communication: Moonraker, MQTT, SSWCP protocol for Snapmaker devices), `Config/`.
- `src/Snapmaker_Orca.cpp` is the entry point. Other top-level `src/` dirs (`mqtt`, `sentry_wrapper`, `bury_cfg`, `common_func`) are small support libs.
- Snapmaker-specific code to know about: `libslic3r/Support/SupportFins.*` (fins that use Top interface layers), `MixedFilament.*` (mixed/blended filaments), and the U1 smart tool change prime (`toolchange_prime_*` settings; touches `PrintConfig`, `GCode.cpp`, `GCodeProcessor`, `Preset.cpp`/`PresetBundle.cpp`, and `tests/fff_print/test_toolchange_prime.cpp`). The companion Klipper macro is in `doc/printer_settings/snapmaker-u1-smart-toolchange-prime.cfg`.

### Adding or changing a setting

A setting spans several places; missing one produces silent bugs:
1. Define it in `PrintConfig.cpp`/`.hpp` (bounds, default, enum maps).
2. Add it to the preset option lists in `Preset.cpp` (and `PresetBundle.cpp` if it needs cross-preset handling).
3. Add UI in `src/slic3r/GUI/Tab.cpp` (and `ConfigManipulation.cpp` for enable/disable logic).
4. Add the tooltip/label with translatable `L()` strings; regenerate the pot with `scripts/run_gettext.sh`.
5. If the G-code processor needs it, check `GCode/GCodeProcessor.cpp`.

### Profiles

- `resources/profiles/<Vendor>.json` is the vendor index; `resources/profiles/<Vendor>/{machine,process,filament}/` hold presets that inherit from `fdm_*.json` bases (Snapmaker: `fdm_U1.json`, `fdm_a250.json`, etc.).
- The app caches system profiles in the user data dir (`~/Library/Application Support/Snapmaker_Orca/system` on macOS) and only refreshes them when the `version` in `resources/profiles/Snapmaker.json` is newer. **Bump that version whenever any Snapmaker preset changes**, or a local run keeps using stale G-code templates.
- CI validates profiles (`check_profiles.yml`, `scripts/orca_extra_profile_check.py`, `Snapmaker_Orca_profile_validator`).

## Conventions

- PascalCase classes, snake_case functions/variables, `#pragma once`.
- Changes must stay compatible with existing project (3MF) files and profiles; settings renames need migration handling.
- Keep cross-platform (macOS/Windows/Linux) behavior in mind; platform code is in `libslic3r/Platform.cpp`, `MacUtils.mm`, and `#ifdef`s in GUI.
- Documentation for settings and developer references is in `doc/` (see `doc/developer-reference/`, `doc/Localization_guide.md`).
