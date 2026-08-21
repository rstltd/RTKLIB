/*------------------------------------------------------------------------------
* fgo_config.h : internal configuration and variable naming for the FGO backend
*
*          Copyright (C) 2026 by RST Ltd., All rights reserved.
*
* C++17, internal to src/fgo/.  No C translation unit includes this; the only
* contract with the core is rtklib_fgo_api.h (plan.md 3.6).
*
* Two things live here:
*
*   the key schema   which GTSAM variable a quantity maps to (plan.md 4.1.2)
*   fgo::Config      prcopt_t's flat fgo_* fields turned into something
*                    structured, converted in exactly one place so the rest of
*                    the backend never spells a prcopt_t field name and an ABI
*                    change to that struct stops here (plan.md 7.2)
*
* version : $Revision:$ $Date:$
* history : 2026/08/21 1.0 new
*-----------------------------------------------------------------------------*/
#ifndef FGO_CONFIG_H
#define FGO_CONFIG_H

#include "rtklib.h"

#include <gtsam/inference/Symbol.h>

#include <cstdint>
#include <stdexcept>
#include <string>

namespace fgo {

/*---------------------------------------------------------------------------
 * Key schema (plan.md 4.1.2)
 *
 * A gtsam::Key is 8 bits of character and 56 bits of index.  GTSAM 4.2 throws
 * std::invalid_argument when an index overflows that, which is far better than
 * corrupting the character silently -- but the message names only the number,
 * not which variable produced it.  The helpers below check first and say what
 * was being built, because by the time this fires the interesting question is
 * which schema ran out of room, not that one did.
 *
 * Headroom, for the record: keyIono() is the densest, at epoch*MAXSAT.  With
 * MAXSAT at 208 that allows about 3.4e14 epochs, so a 1 Hz receiver would need
 * roughly ten million years to reach it.  The check is for programming errors,
 * not for operation.
 *-------------------------------------------------------------------------*/
constexpr std::uint64_t kMaxKeyIndex = (std::uint64_t(1) << 56) - 1;

inline gtsam::Key makeKey(unsigned char c, std::uint64_t j, const char *what)
{
    if (j > kMaxKeyIndex) {
        throw std::overflow_error(std::string("fgo: key index overflow for ") +
                                  what + " (" + std::to_string(j) + ")");
    }
    return gtsam::Symbol(c, j);
}

/* receiver position, velocity and acceleration at epoch k */
inline gtsam::Key keyPos(std::uint64_t k) { return makeKey('p', k, "position"); }
inline gtsam::Key keyVel(std::uint64_t k) { return makeKey('v', k, "velocity"); }
inline gtsam::Key keyAcc(std::uint64_t k) { return makeKey('a', k, "acceleration"); }

/* zenith tropospheric delay, per receiver (0 rover, 1 base) */
inline gtsam::Key keyTrop(std::uint64_t k, int rcv)
{
    return makeKey('t', k * 2 + static_cast<std::uint64_t>(rcv), "troposphere");
}

/* slant ionospheric delay, per satellite */
inline gtsam::Key keyIono(std::uint64_t k, int sat)
{
    return makeKey('i', k * MAXSAT + static_cast<std::uint64_t>(sat - 1),
                   "ionosphere");
}

/* GLONASS inter-channel bias, one per frequency for the whole run */
inline gtsam::Key keyGloIcb(int f)
{
    return makeKey('g', static_cast<std::uint64_t>(f), "GLONASS ICB");
}

/* Carrier phase bias, keyed by ARC rather than by (satellite, frequency).
   A cycle slip must start a NEW variable: the graph is cumulative, so resetting
   an existing node the way udbias() resets a filter state would retroactively
   change the meaning of every factor already attached to it (plan.md 4.1.3). */
using ArcId = std::uint64_t;
inline gtsam::Key keyBias(ArcId arc) { return makeKey('b', arc, "phase bias"); }

/*---------------------------------------------------------------------------
 * Configuration
 *-------------------------------------------------------------------------*/
struct RobustConfig {
    int    type     = FGO_ROBUST_HUBER;
    double huberD   = FGO_DEF_HUBER_D;
    double cauchyC  = FGO_DEF_CAUCHY_C;
    double tukeyC   = FGO_DEF_TUKEY_C;
    bool   scaleEst = true;
    double scaleMin = FGO_DEF_SCALEMIN;
    double scaleMax = FGO_DEF_SCALEMAX;
};

struct Config {
    int          solver     = FGO_SOLVER_EKF;
    double       windowSec  = FGO_DEF_WINDOW;
    int          maxIter    = FGO_DEF_MAXITER;
    int          ddCovMode  = FGO_DDCOV_BLOCK;
    int          elwModel   = FGO_ELW_RTKLIB;
    bool         enableTdcp = true;
    bool         mpAdaptive = false;
    bool         arcStitch  = false;
    bool         jerkConst  = false;
    bool         async      = false;
    int          timeoutMs  = FGO_DEF_TIMEOUT;
    double       innoScale  = FGO_DEF_INNOSCALE;
    double       relinThres = FGO_DEF_RELINTHRES;
    RobustConfig robust;
    std::string  siteFile, insightFile, mpMapFile;

    /* The single conversion point.  Values are clamped rather than trusted:
       a configuration file can carry anything, and an option set before these
       fields existed leaves them at whatever resetsysopts() supplied -- so a
       maxIter of zero or a negative window has to become something workable
       here rather than confusing an optimizer later. */
    static Config fromPrcopt(const prcopt_t &opt);

    /* Human-readable summary, for the trace log at init. */
    std::string describe() const;
};

}  // namespace fgo

#endif /* FGO_CONFIG_H */
