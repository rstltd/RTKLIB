/*------------------------------------------------------------------------------
* fgo_abi_cxx.cpp : C++-side check of the FGO ABI header
*
*          Copyright (C) 2026 by RST Ltd., All rights reserved.
*
* src/fgo/ is C++, so the ABI header has to be usable from C++ and the symbols
* it declares must link with C linkage.  If the extern "C" guard were missing
* or misplaced, this translation unit would emit mangled references and fail
* to link against the C stub -- which is precisely the failure it exists to
* catch, since nothing else in the tree includes the header from C++ yet.
*
* version : $Revision:$ $Date:$
* history : 2026/08/21 1.0 new
*-----------------------------------------------------------------------------*/
#include <string>
#include <vector>

#include "rtklib_fgo_api.h"

#include <cstdio>
#include <cstdarg>

extern "C" int  showmsg (const char *format, ...) { (void)format; return 0; }
extern "C" void settspan(gtime_t ts, gtime_t te)  { (void)ts; (void)te; }
extern "C" void settime (gtime_t time)            { (void)time; }

int main()
{
    /* the C++ standard library must still work after including the header */
    std::vector<std::string> v;
    v.push_back("linkage");

    rtk_t rtk;
    prcopt_t opt = prcopt_default;
    rtkinit(&rtk, &opt);

    const int en = fgo_enabled();
    const int rc = fgo_init(&rtk, &opt);
    const int ep = fgo_process_epoch(&rtk, nullptr, 0, 0, nullptr);
    fgo_free(&rtk);
    rtkfree(&rtk);

    const bool ok = (en == 0) && (rc == FGO_ERR_DISABLED) &&
                    (ep == FGO_ERR_DISABLED) && (v.size() == 1);
    std::printf("  %-42s %s  enabled=%d init=%d epoch=%d\n",
                "C++ links against the C stub", ok ? "PASS" : "FAIL",
                en, rc, ep);
    return ok ? 0 : 1;
}
