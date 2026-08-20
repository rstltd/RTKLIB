include_directories(.)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

if(WIN32)
  add_definitions(-DWIN_DLL)
else()
  add_definitions(-DDLL)
endif()

if (GCC)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -O3 -ansi -pedantic")
    set(CMAKE_C_FLAGS "-Wno-unused-but-set-variable -Wno-format-overflow -Wno-unused-result -Wpointer-to-int-cast")
endif()

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    if (GCC)
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
    endif()
endif()

if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

aux_source_directory(. DIR_SRCS_RTKLIB)
set(DIR_SRCS_RTKLIB_RCV
  ${DIR_SRCS_RTKLIB_RCV}
  rcv/binex.c
  rcv/crescent.c
  rcv/javad.c
  rcv/novatel.c
  rcv/nvs.c
  rcv/rt17.c
  rcv/septentrio.c
  rcv/skytraq.c
  rcv/swiftnav.c
  rcv/ublox.c
  rcv/unicore.c
)
list(APPEND DIR_SRCS ${DIR_SRCS_RTKLIB} ${DIR_SRCS_RTKLIB_RCV})

add_library(rtklib SHARED ${DIR_SRCS} rtklib.h)
set_property(TARGET rtklib PROPERTY C_STANDARD 99)

# Give the shared library a SONAME derived from VER_RTKLIB_ABI (src/rtklib.h).
# Structures such as prcopt_t and rtk_t are part of the public surface, so
# appending a field changes their size and breaks binary compatibility.
# Without a SONAME the loader has no way to tell the layouts apart and an
# application built against the old headers would corrupt memory inside
# rtkinit() (plan.md 6.10 RC-2).
if(RTKLIB_ABI)
    set_target_properties(rtklib PROPERTIES
        VERSION   ${RTKLIB_ABI}.0.0
        SOVERSION ${RTKLIB_ABI})
else()
    message(WARNING "VER_RTKLIB_ABI not found in src/rtklib.h; "
                    "librtklib will be built without a SONAME")
endif()


target_include_directories(rtklib
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

if(WIN32)
  target_link_libraries(rtklib wsock32 ws2_32 winmm)
  add_definitions(-DWIN_DLL)
else()
  target_link_libraries(rtklib m pthread)
  add_definitions(-DDLL)
endif()
# On ELF platforms a SHARED library is a LIBRARY artifact, not a RUNTIME one,
# so the RUNTIME-only form installed nothing at all outside Windows -- and a
# SONAME that never reaches an installed file is no protection.
#
# The unversioned librtklib.so symlink is split into the Development
# component.  It exists only so that `-lrtklib` can find the library at link
# time; a runtime install has no use for it, and shipping it lets an
# executable whose DT_NEEDED is the bare "librtklib.so" -- which is what the
# linker recorded before this library had a SONAME -- resolve to an ABI it was
# not built against.  The loader matches DT_NEEDED by filename and never
# checks the target's SONAME, so this cannot be prevented outright for
# binaries that were already linked; keeping the bare name out of runtime
# packages is the part that is actually in our control.  See docs/fgo/abi.md.
#
# RUNTIME stays at "lib" rather than moving to CMAKE_INSTALL_BINDIR: that is
# the Windows DLL path and changing it is not part of this work.
install(TARGETS rtklib
        RUNTIME DESTINATION lib
                COMPONENT Runtime
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
                COMPONENT Runtime
                NAMELINK_COMPONENT Development
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
                COMPONENT Development)
