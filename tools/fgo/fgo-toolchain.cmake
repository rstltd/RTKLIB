# fgo-toolchain.cmake : pinned toolchain for the RTKLIB FGO extension
#
#   cmake -S <dir> -B <build> --toolchain <path>/tools/fgo/fgo-toolchain.cmake
#
# Two independent things are pinned here, and the distinction matters:
#
#   * the C and C++ compilers -- these come from the SYSTEM, not from conda.
#     plan.md 6.9 (traps 1 and 2) requires the byte-diff baselines of the EKF
#     path to be reproducible, which means the compiler that builds src/*.c
#     must never drift.  A conda toolchain would silently change with every
#     `conda update`, so it is deliberately not used.
#
#   * GTSAM / Eigen / Boost / TBB -- these come from the conda environment
#     described by tools/fgo/environment.yml, located via CMAKE_PREFIX_PATH.
#
# Overridable with environment variables:
#   FGO_ENV_PREFIX   conda env prefix  (default: $CONDA_PREFIX, else the
#                                       rtklib-fgo env under $HOME/miniconda3)
#   FGO_CC           C compiler        (default: /usr/bin/gcc)
#   FGO_CXX          C++ compiler      (default: /usr/bin/g++)
#   FGO_EXPECT_GCC   expected compiler version, "" disables the check
#                                      (default: 12.2.0)
#   FGO_STRICT_TOOLCHAIN  1 = version mismatch is a fatal error, not a warning

# a toolchain file is read more than once by CMake; keep it idempotent
if(DEFINED FGO_TOOLCHAIN_INCLUDED)
  return()
endif()
set(FGO_TOOLCHAIN_INCLUDED TRUE)

# ---- scrub inherited build flags ---------------------------------------------
# CMake seeds CMAKE_<LANG>_FLAGS and CMAKE_EXE_LINKER_FLAGS from the CFLAGS /
# CXXFLAGS / CPPFLAGS / LDFLAGS environment variables.  An activated conda
# environment exports all four -- typically with -march/-mtune, -O2, an -isystem
# pointing into the env, and RPATHs back to it.  Inheriting those would silently
# change the optimisation level and floating-point code generation of the very
# build whose output plan.md 6.9 requires to be byte-reproducible, and would do
# so differently depending on whether the developer had run `conda activate`.
#
# Clearing them here (rather than in the calling script) protects every consumer
# of this toolchain file, not just tools/fgo/verify_env.sh.
foreach(_v CFLAGS CXXFLAGS CPPFLAGS LDFLAGS)
  if(NOT "$ENV{${_v}}" STREQUAL "")
    message(STATUS "fgo-toolchain: ignoring inherited ${_v} from the environment")
    set(ENV{${_v}} "")
  endif()
endforeach()

# ---- compilers ---------------------------------------------------------------
if(DEFINED ENV{FGO_CC})
  set(CMAKE_C_COMPILER "$ENV{FGO_CC}"   CACHE FILEPATH "FGO pinned C compiler")
else()
  set(CMAKE_C_COMPILER "/usr/bin/gcc"   CACHE FILEPATH "FGO pinned C compiler")
endif()

if(DEFINED ENV{FGO_CXX})
  set(CMAKE_CXX_COMPILER "$ENV{FGO_CXX}" CACHE FILEPATH "FGO pinned C++ compiler")
else()
  set(CMAKE_CXX_COMPILER "/usr/bin/g++"  CACHE FILEPATH "FGO pinned C++ compiler")
endif()

# ---- compiler version ---------------------------------------------------------
# "Pinned" means the baseline records which compiler produced it and the build
# says so out loud when that compiler changes.  It is a warning by default
# because only the C byte-diff baseline needs bit-exactness -- src/fgo/ itself
# builds fine on any C++17 compiler -- and hard-failing would block FGO work on
# every machine whose distro ships something else.  Set FGO_STRICT_TOOLCHAIN=1
# when regenerating or comparing baselines, where a mismatch really is fatal.
if(DEFINED ENV{FGO_EXPECT_GCC})
  set(_fgo_expect "$ENV{FGO_EXPECT_GCC}")
else()
  set(_fgo_expect "12.2.0")
endif()

if(NOT _fgo_expect STREQUAL "")
  execute_process(COMMAND "${CMAKE_C_COMPILER}" -dumpfullversion -dumpversion
                  OUTPUT_VARIABLE _fgo_ccver
                  OUTPUT_STRIP_TRAILING_WHITESPACE
                  ERROR_QUIET)
  string(REGEX REPLACE "\n.*" "" _fgo_ccver "${_fgo_ccver}")
  if(NOT _fgo_ccver STREQUAL _fgo_expect)
    set(_fgo_msg
        "fgo-toolchain: ${CMAKE_C_COMPILER} is version '${_fgo_ccver}', "
        "expected '${_fgo_expect}'.  Byte-diff baselines produced with a "
        "different compiler are not comparable (plan.md 6.11 G1).")
    if("$ENV{FGO_STRICT_TOOLCHAIN}" STREQUAL "1")
      message(FATAL_ERROR ${_fgo_msg})
    else()
      message(WARNING ${_fgo_msg})
    endif()
  endif()
endif()

# ---- conda environment prefix ------------------------------------------------
if(DEFINED ENV{FGO_ENV_PREFIX})
  set(_fgo_prefix "$ENV{FGO_ENV_PREFIX}")
elseif(DEFINED ENV{CONDA_PREFIX})
  set(_fgo_prefix "$ENV{CONDA_PREFIX}")
else()
  set(_fgo_prefix "$ENV{HOME}/miniconda3/envs/rtklib-fgo")
endif()

if(NOT EXISTS "${_fgo_prefix}/lib/cmake/GTSAM/GTSAMConfig.cmake")
  message(FATAL_ERROR
    "GTSAM not found under '${_fgo_prefix}'.\n"
    "Run tools/fgo/setup_env.sh first, or set FGO_ENV_PREFIX to the env that "
    "has it.")
endif()

set(FGO_ENV_PREFIX "${_fgo_prefix}" CACHE PATH "FGO conda environment prefix")
list(APPEND CMAKE_PREFIX_PATH "${_fgo_prefix}")

# The conda libraries were built against a newer libstdc++ than the system
# compiler ships.  Baking an RPATH to the env keeps the resulting binaries
# runnable without activating the environment first.
foreach(_t EXE SHARED MODULE)
  set(CMAKE_${_t}_LINKER_FLAGS_INIT
      "-L${_fgo_prefix}/lib -Wl,-rpath,${_fgo_prefix}/lib")
endforeach()

# so the effective compile lines can be recorded alongside a baseline
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "" FORCE)
