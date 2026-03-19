# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RTKLIB-EX (formerly demo5) — a C library and toolset for GNSS raw data processing, optimized for low-cost receivers (especially u-blox). Supports GPS, GLONASS, Galileo, BeiDou, QZSS, IRNSS, SBAS. Based on RTKLIB 2.4.3. Licensed under BSD 2-clause.

## Build Commands

```bash
# CMake build (primary method, works on Windows/Linux/macOS)
mkdir build && cd build
cmake ..
make            # or: cmake --build .

# Windows with Visual Studio
cmake .. -G "Visual Studio 16 2019"
cmake --build . --config Release

# Run all tests
make test       # or: ctest

# Run a single unit test
./bin/t_ppp     # executables are in build/bin/
```

Unit tests link against `lapack`, `blas`, and `m` — these must be available. On Windows/MSVC, `m.lib` link errors for tests are pre-existing (MSVC doesn't have libm).

## Compile Definitions

Set in root `CMakeLists.txt`: `-DENAGLO -DENAQZS -DENACMP -DENAGAL -DENAIRN -DNFREQ=3 -DNEXOBS=3 -DTRACE -DSVR_REUSEADDR`

## Architecture

### Core Library (`src/`)

All core code is C99. The single public header is `src/rtklib.h` — it defines every struct and function prototype. Key source modules:

| Area | Files | Purpose |
|------|-------|---------|
| Common utilities | `rtkcmn.c`, `trace.c` | Matrix ops, time/coord conversion, satellite functions, trace logging |
| Positioning | `pntpos.c` (SPP), `rtkpos.c` (RTK), `ppp.c` + `ppp_ar.c` (PPP) | Core positioning algorithms |
| Ephemeris | `ephemeris.c`, `preceph.c` | Broadcast & precise orbit/clock |
| RTCM/SSR | `rtcm.c`, `rtcm2.c`, `rtcm3.c`, `rtcm3e.c` | RTCM 2/3 encode/decode, SSR corrections |
| Observations | `rinex.c`, `rcvraw.c`, `convrnx.c` | RINEX I/O, receiver raw data |
| Receiver drivers | `rcv/*.c` | u-blox, Novatel, Septentrio, BINEX, etc. (13 formats) |
| Corrections | `sbas.c`, `ionex.c`, `tides.c` | SBAS, ionosphere maps, tide models |
| Ambiguity | `lambda.c`, `ppp_ar.c` | LAMBDA algorithm, PPP ambiguity resolution |
| Streams | `stream.c`, `streamsvr.c`, `rtksvr.c` | Serial/TCP/NTRIP/file I/O, real-time server |

### Matrix functions (commonly misused)

- `matmul("NN",n,k,m,A,B,C)` → `C = A*B` (no alpha/beta — overwrites C)
- `matmulp(...)` → `C += A*B`; `matmulm(...)` → `C -= A*B`
- `matinv(A,n)` → in-place inverse, returns 0 on success

### Applications

- **Console apps** (`app/consapp/`): `rnx2rtkp` (post-processing), `convbin` (format conversion), `pos2kml`, `str2str`, `rtkrcv`
- **Qt GUI apps** (`app/qtapp/`): C++ with Qt5/6 — `rtkpost_qt`, `rtknavi_qt`, `rtkplot_qt`, `rtkconv_qt`, etc.
- **Windows GUI** (`app/winapp/`): Embarcadero C++ Builder (legacy)

### Third-party libraries (`lib/`)

- `sofa/` — IAU SOFA astronomical routines
- `openblas/` — OpenBLAS headers
- `iers/` — IERS Earth models (optional, Fortran, `-DIERS_MODEL=ON`)

### Tests (`test/utest/`)

13 unit test executables (t_matrix, t_time, t_coord, t_rinex, t_lambda, t_atmos, t_misc, t_preceph, t_gloeph, t_geoid, t_ppp, t_ionex, t_tle). Each links specific source files directly rather than the shared library. Integration tests for console apps are defined in `app/consapp/CMakeLists.txt`.

## Git Remote Policy

- `origin` → `rstltd/RTKLIB` (organization fork) — all pushes go here
- `upstream` → `rtklibexplorer/RTKLIB` (original author) — **READ-ONLY**
- **NEVER run `git push upstream`** — this would push to the original author's repository
- Always use `git push` or `git push origin` only

## Trace/Debug

Pass `-t <level>` to console apps or set `pos1-trace=<level>` in config. Trace file is created alongside the output file (e.g., `output.pos` → `output.pos.trace`). Levels: 1=errors, 2=warnings, 3=info, 4+=debug.
