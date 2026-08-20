/*------------------------------------------------------------------------------
* rtklib.h : RTKLIB constants, types and function prototypes
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* options : -DENAGLO   enable GLONASS
*           -DENAGAL   enable Galileo
*           -DENAQZS   enable QZSS
*           -DENACMP   enable BeiDou
*           -DENAIRN   enable IRNSS
*           -DNFREQ=n  set number of obs codes/frequencies
*           -DNEXOBS=n set number of extended obs codes
*           -DMAXOBS=n set max number of obs data in an epoch
*           -DWIN32    use WIN32 API
*           -DWIN_DLL  generate library as Windows DLL
*
* version : $Revision:$ $Date:$
* history : 2007/01/13 1.0  rtklib ver.1.0.0
*           2007/03/20 1.1  rtklib ver.1.1.0
*           2008/07/15 1.2  rtklib ver.2.1.0
*           2008/10/19 1.3  rtklib ver.2.1.1
*           2009/01/31 1.4  rtklib ver.2.2.0
*           2009/04/30 1.5  rtklib ver.2.2.1
*           2009/07/30 1.6  rtklib ver.2.2.2
*           2009/12/25 1.7  rtklib ver.2.3.0
*           2010/07/29 1.8  rtklib ver.2.4.0
*           2011/05/27 1.9  rtklib ver.2.4.1
*           2013/03/28 1.10 rtklib ver.2.4.2
*           2020/11/30 1.11 rtklib ver.2.4.3 b34
*-----------------------------------------------------------------------------*/
#ifndef RTKLIB_H
#define RTKLIB_H
#ifdef __APPLE__
#define _DARWIN_C_SOURCE
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <ctype.h>
#include <stdint.h>
#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
#else
#include <pthread.h>
#include <sys/select.h>
#endif
#ifdef __cplusplus
extern "C" {
#endif

#ifdef _MSC_VER
#ifdef WIN_DLL /* for Windows DLL */
#define EXPORT __declspec(dllexport)
#elif !defined(WIN_STATIC)
#define EXPORT __declspec(dllimport)
#else
#define EXPORT // For files bundled into an app.
#endif
#else
#define EXPORT
#endif

#if (__STDC_VERSION__ >= 201710L)
#define THREADLOCAL _Thread_local
#elif defined(__GNUC__)
#define THREADLOCAL __thread
#elif defined(_MSC_VER)
#define THREADLOCAL __declspec(thread)
#else
#define THREADLOCAL
#endif

/* constants -----------------------------------------------------------------*/

#define VER_RTKLIB  "EX"             /* library version */

#define PATCH_LEVEL "2.5.2"               /* patch level */

/* ABI version of librtklib, used as the ELF SONAME (see src/CMakeLists.txt).
 *
 * Bump this whenever an already-linked executable would be unsafe against the
 * new library.  Struct layout is only one of the ways that happens:
 *
 *   - a public struct changes layout (prcopt_t, rtk_t, sol_t, nav_t, obsd_t,
 *     ssat_t ...), including merely appending a field, since callers allocate
 *     these and sizeof() is part of the contract
 *   - an exported function's signature changes: parameters added, removed,
 *     reordered or retyped, or a different return type
 *   - an exported symbol is removed, renamed, or becomes static
 *   - the meaning of an argument or return value changes with its type
 *     intact: units, sign, pointer ownership, error-code values
 *   - a public macro callers compile into their own code changes value
 *     (MAXSAT, NFREQ, ...)
 *
 * Adding a new exported function or a new #define does not need a bump.
 * When in doubt, bump: an unnecessary bump costs a rebuild, a missed one
 * costs memory corruption in the field.
 *
 * PATCH_LEVEL is not a substitute -- it never reaches the SONAME, so on its
 * own it documents a break without preventing it (plan.md 6.10 RC-2).
 *
 * Full rationale, and what the SONAME does and does not protect against:
 * docs/fgo/abi.md
 *
 * 1 : first versioned ABI.  prcopt_t gained the fgo_* fields and rtk_t gained
 *     rtk_t::fgo, so this is not layout-compatible with any earlier build.  */
#define VER_RTKLIB_ABI "1"

/* --- domain fragments, assembled here in original order (umbrella header).
 *     Split from the historical single-file rtklib.h; behaviour-preserving. --- */
#include "rtklib_const.h"
#include "rtklib_types.h"
#include "rtklib_api.h"
#ifdef __cplusplus
}
#endif
#endif /* RTKLIB_H */
