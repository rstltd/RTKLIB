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
#else
#define EXPORT __declspec(dllimport)
#endif
#else
#define EXPORT
#endif

#if (__STDC_VERSION__ >= 201710L)
#define THREADLOCAL _Thread_local
#elif defined(__GNUC__)
#define THREADLOCAL __thread
#elif defined(_MSC_VER)
#define THREADLOCAL __declspec(__thread)
#else
#define THREADLOCAL
#endif

/* constants -----------------------------------------------------------------*/

#define VER_RTKLIB  "EX"             /* library version */

#define PATCH_LEVEL "2.5.0"               /* patch level */

/* --- domain fragments, assembled here in original order (umbrella header).
 *     Split from the historical single-file rtklib.h; behaviour-preserving. --- */
#include "rtklib_const.h"
#include "rtklib_types.h"
#include "rtklib_api.h"
#ifdef __cplusplus
}
#endif
#endif /* RTKLIB_H */
