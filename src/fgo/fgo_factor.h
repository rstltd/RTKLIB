/*------------------------------------------------------------------------------
* fgo_factor.h : GTSAM factors backed by RTKLIB's observation model
*
*          Copyright (C) 2026 by RST Ltd., All rights reserved.
*
* C++17, internal to src/fgo/.
*
* These are the factors of plan.md 4.2.  What distinguishes them from the
* linearised architecture A that 1.2 rejects is that evaluateError() calls back
* into RTKLIB at whatever state the optimizer hands it, rather than replaying a
* Jacobian captured once: the geometry, elevation, troposphere mapping and
* antenna correction are all recomputed there (4.2.1, requirement 2).
*
* version : $Revision:$ $Date:$
* history : 2026/08/21 1.0 new
*-----------------------------------------------------------------------------*/
#ifndef FGO_FACTOR_H
#define FGO_FACTOR_H

#include "fgo_rtklib.h"
#include "fgo_config.h"

#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>
#include <vector>

namespace fgo {

/* GTSAM 4.2 hands out boost::shared_ptr and 4.3 hands out std::shared_ptr.
   Rebinding GTSAM's own alias onto another type follows whichever it is,
   without this code naming either library -- which is what
   docs/fgo/build_environment.md asks for and what a graph's push_back()
   requires, since it accepts only the flavour GTSAM itself uses. */
template <class T>
using SharedFactor = typename std::pointer_traits<
    gtsam::NonlinearFactor::shared_ptr>::template rebind<T>;

/*---------------------------------------------------------------------------
 * GnssDDFactor : double-differenced pseudorange and carrier phase
 *
 * Vector-valued rather than one factor per double difference, because
 * ddcov() produces a block-dense covariance -- every DD in a block shares the
 * reference satellite's error -- and splitting them would discard that
 * correlation (plan.md 4.2.1, decision D7 mode (a)).
 *
 * Connects the receiver position only.  The remaining states -- phase biases,
 * troposphere, ionosphere, GLONASS inter-channel bias -- are held at the
 * values of a baseline vector supplied at construction.
 *
 * That makes this factor CORRECT AND COMPLETE for a code-only configuration
 * (opt->mode <= PMODE_DGPS), where ddres_core() emits no carrier phase rows
 * and there are no bias states to hold.
 *
 * For a carrier phase configuration it is only as good as the baseline
 * biases.  Held at zero -- which is what rtkinit() leaves -- the phase
 * residuals are of order 1e7 m and an optimizer will happily move the
 * position by thousands of kilometres to fit them; that was observed, not
 * theorised.  create() therefore warns when it is built for such a
 * configuration.  Promoting the biases to graph variables is P3.9/P3.10 and
 * needs arc management first, because a cycle slip must start a new variable
 * rather than reset an existing one (4.1.3).
 *
 * Sign convention (plan.md 3.4).  RTKLIB gives v = z - h(x) and stores H as
 * dh/dx with one COLUMN per measurement; GTSAM wants e = h(x) - z and one ROW
 * per measurement.  So error = -v, and the Jacobian is H's column transposed.
 * Getting this backwards produces a Jacobian of the right magnitude and the
 * wrong sign, which 6.11 identifies as the hardest FGO bug to see.
 *-------------------------------------------------------------------------*/
class GnssDDFactor : public gtsam::NoiseModelFactorN<gtsam::Point3> {
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Point3>;

    /* ctx must outlive the factor, and must already be frozen: a factor's
       dimension is fixed at construction, and only freezing makes the row set
       independent of the state (4.2.1). */
    static SharedFactor<GnssDDFactor> create(gtsam::Key posKey,
                                             const fgo_dd_ctx_t *ctx,
                                             const double *xBase, int nx,
                                             const Config &cfg);

    gtsam::Vector evaluateError(
        const gtsam::Point3 &p,
        boost::optional<gtsam::Matrix &> Hp = boost::none) const override;

    /* rows the factor carries; fixed for its lifetime */
    int rows() const { return dim_; }

    /* how many of those rows exceeded the innovation threshold when the graph
       was built -- diagnostic, and the hook the robust kernel work of 5.5
       will use */
    int rejectedRows() const;

private:
    GnssDDFactor(const gtsam::SharedNoiseModel &model, gtsam::Key posKey,
                 const fgo_dd_ctx_t *ctx, const double *xBase, int nx, int dim);

    /* Evaluate at a state, into the preallocated buffers.  Returns rows, or a
       negative FGO_ERR_*.  const because the buffers are scratch, not state --
       see the mutable declarations below. */
    int evalAt(const gtsam::Point3 &p) const;

    const fgo_dd_ctx_t *ctx_;
    int nx_;
    int dim_;

    /* Baseline state: every element except the position, which the optimizer
       supplies.  Copied, not referenced -- the factor outlives the epoch. */
    std::vector<double> xBase_;

    /* evaluateError() is the hot path, called once per factor per
       linearisation, so nothing here is allocated during it (plan.md 7.2). */
    mutable std::vector<double> x_, ws_, v_, H_, R_;
    mutable std::vector<int> vflg_;
    mutable std::unique_ptr<ddres_stat_t> st_;
};

}  // namespace fgo

#endif /* FGO_FACTOR_H */
