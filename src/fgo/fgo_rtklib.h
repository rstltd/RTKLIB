/*------------------------------------------------------------------------------
* fgo_rtklib.h : RTKLIB, made safe to include alongside C++ libraries
*
*          Copyright (C) 2026 by RST Ltd., All rights reserved.
*
* rtklib_api.h defines a function-like macro named `trace`.  Eigen has
* Matrix::trace(), so any GTSAM header parsed while that macro is live fails
* with errors pointing at rtklib_api.h rather than at the real cause:
*
*   error: expected unqualified-id before 'do'
*     #define trace(level, ...) do { ... } while (0)
*
* Include order alone would fix it -- GTSAM first, RTKLIB second -- but that is
* a rule someone has to remember every time, and breaking it produces a
* confusing error a long way from the edit.  This header removes the macro
* instead, and offers fgo_trace() in its place, so order stops mattering.
*
* Every C++ file under src/fgo/ should include THIS rather than rtklib.h or
* rtklib_fgo_api.h directly.
*
* version : $Revision:$ $Date:$
* history : 2026/08/21 1.0 new
*-----------------------------------------------------------------------------*/
#ifndef FGO_RTKLIB_H
#define FGO_RTKLIB_H

#include "rtklib_fgo_api.h"

/* Capture the behaviour before dropping the macro.  It cannot simply be
   forwarded -- a macro that expands to `trace(...)` would be expanded at the
   point of USE, by which time the name is gone -- so the body is restated
   here.  It has to track rtklib_api.h, which is why both spellings live
   behind the same TRACE switch that file uses. */
#ifdef trace
#undef trace
#endif

#ifdef TRACE
#define fgo_trace(level, ...)                                                  \
    do {                                                                       \
        if ((level) <= gettracelevel()) trace_impl(level, __VA_ARGS__);        \
    } while (0)
#else
/* Without TRACE the implementation functions are not compiled at all, so this
   must not reference them. */
#define fgo_trace(level, ...) ((void)0)
#endif

/* The other trace spellings are left alone: none of them collide with a C++
   library name, and undefining them would only make this header a place where
   things quietly disappear. */

#endif /* FGO_RTKLIB_H */
