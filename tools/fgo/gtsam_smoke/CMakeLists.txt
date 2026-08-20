# GTSAM capability probe for the RTKLIB FGO extension.
#
# Standalone project on purpose: it must be buildable before any part of
# src/fgo/ exists, and it must not depend on the RTKLIB build at all.
#
#   cmake -S . -B build --toolchain ../fgo-toolchain.cmake
#   cmake --build build && ./build/gtsam_smoke

cmake_minimum_required(VERSION 3.16)
project(gtsam_smoke LANGUAGES CXX)

find_package(GTSAM 4.2 REQUIRED)

add_executable(gtsam_smoke gtsam_smoke.cpp)
set_property(TARGET gtsam_smoke PROPERTY CXX_STANDARD 17)
set_property(TARGET gtsam_smoke PROPERTY CXX_STANDARD_REQUIRED ON)

# gtsam_unstable is required for {Batch,Incremental}FixedLagSmoother, which
# plan.md 4.5.3/4.5.4 rely on for the sliding-window and iSAM2 solvers.
target_link_libraries(gtsam_smoke PRIVATE gtsam gtsam_unstable)
