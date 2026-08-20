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
#   FGO_ENV_PREFIX  conda env prefix   (default: $CONDA_PREFIX, else the
#                                       rtklib-fgo env under $HOME/miniconda3)
#   FGO_CC          C compiler         (default: /usr/bin/gcc)
#   FGO_CXX         C++ compiler       (default: /usr/bin/g++)

# a toolchain file is read more than once by CMake; keep it idempotent
if(DEFINED FGO_TOOLCHAIN_INCLUDED)
  return()
endif()
set(FGO_TOOLCHAIN_INCLUDED TRUE)

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
