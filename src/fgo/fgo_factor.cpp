/*------------------------------------------------------------------------------
* fgo_factor.cpp : GTSAM factors backed by RTKLIB's observation model
*
*          Copyright (C) 2026 by RST Ltd., All rights reserved.
*
* See fgo_factor.h for the design, and in particular for the sign convention.
*
* version : $Revision:$ $Date:$
* history : 2026/08/21 1.0 new
*-----------------------------------------------------------------------------*/
#include "fgo_factor.h"

#include <gtsam/base/Matrix.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace fgo {

bool GnssDDFactor::supports(const fgo_dd_ctx_t *ctx, std::string *why)
{
    const prcopt_t *opt = ctx ? fgo_dd_opt(ctx) : nullptr;
    if (!opt) {
        if (why) *why = "no context";
        return false;
    }
    /* Every one of these puts a state OTHER than the position into the
       residual that ddres_core() computes.  Holding such a state at its
       baseline value does not make the factor approximate -- it makes it
       wrong in a way that lands entirely in the position, because the
       position is the only thing the optimizer can move.  Refusing is the
       honest answer until those states become graph variables (P3.9/P3.10). */
    if (opt->mode > PMODE_DGPS) {
        if (why) *why = "carrier phase mode needs phase bias variables";
        return false;
    }
    if (opt->ionoopt == IONOOPT_EST) {
        if (why) *why = "estimated ionosphere needs iono variables";
        return false;
    }
    if (opt->tropopt >= TROPOPT_EST) {
        if (why) *why = "estimated troposphere needs tropo variables";
        return false;
    }
    if (opt->glomodear == GLO_ARMODE_AUTOCAL) {
        if (why) *why = "GLONASS ICB autocal needs an ICB variable";
        return false;
    }
    return true;
}

GnssDDFactor::GnssDDFactor(const gtsam::SharedNoiseModel &model,
                           gtsam::Key posKey, const fgo_dd_ctx_t *ctx,
                           const double *xBase, int nx, int dim,
                           const ddres_stat_t &st0)
    : Base(model, posKey), ctx_(ctx), nx_(nx), dim_(dim),
      xBase_(xBase, xBase + nx)
{
    const int nvmax = fgo_dd_nvmax(ctx);
    x_.resize(nx);
    ws_.resize(fgo_dd_ws_size(ctx));
    v_.resize(nvmax);
    H_.resize(static_cast<std::size_t>(nvmax) * nx);
    R_.resize(static_cast<std::size_t>(nvmax) * nvmax);
    vflg_.resize(nvmax);
    st_.reset(new ddres_stat_t(st0));
}

gtsam::NonlinearFactor::shared_ptr GnssDDFactor::clone() const
{
    /* Scratch is per-instance, so the copy gets its own: two clones must be
       evaluable independently, including from different threads.  The context
       is shared by design -- it is immutable once frozen. */
    return gtsam::NonlinearFactor::shared_ptr(new GnssDDFactor(
        noiseModel(), keys().front(), ctx_, xBase_.data(), nx_, dim_, *st_));
}

int GnssDDFactor::evalAt(const gtsam::Point3 &p) const
{
    std::copy(xBase_.begin(), xBase_.end(), x_.begin());
    x_[0] = p.x();
    x_[1] = p.y();
    x_[2] = p.z();
    return fgo_dd_eval(ctx_, x_.data(), nx_, ws_.data(), v_.data(), H_.data(),
                       R_.data(), vflg_.data(), st_.get());
}

SharedFactor<GnssDDFactor> GnssDDFactor::create(gtsam::Key posKey,
                                               const fgo_dd_ctx_t *ctx,
                                               const double *xBase, int nx,
                                               const Config &cfg)
{
    if (!ctx || !xBase || nx <= 0) {
        throw std::invalid_argument("fgo: GnssDDFactor::create bad argument");
    }
    /* Refuse rather than warn.  With an unmodelled state held at its baseline
       value the error lands entirely in the position -- observed with phase
       biases at zero, where the optimizer moved the position 9500 km to fit
       residuals of order 1e7 m.  A caller that gets a solution back has to be
       able to trust it. */
    std::string why;
    if (!supports(ctx, &why)) {
        fgo_trace(2, "fgo: GnssDDFactor unsupported configuration: %s\n",
                  why.c_str());
        throw std::invalid_argument("fgo: GnssDDFactor unsupported "
                                    "configuration: " + why);
    }

    /* Build once at the baseline state to learn the dimension and the
       covariance.  Both are fixed for the factor's life: the dimension because
       GTSAM requires it, the covariance because it is a property of the
       observations and their geometry, not of the iterate. */
    const int nvmax = fgo_dd_nvmax(ctx);
    std::vector<double> x(xBase, xBase + nx), ws(fgo_dd_ws_size(ctx));
    std::vector<double> v(nvmax), H(static_cast<std::size_t>(nvmax) * nx);
    std::vector<double> R(static_cast<std::size_t>(nvmax) * nvmax);
    std::vector<int> vflg(nvmax);
    ddres_stat_t st;

    const int nv = fgo_dd_eval(ctx, x.data(), nx, ws.data(), v.data(), H.data(),
                               R.data(), vflg.data(), &st);
    if (nv < 0) {
        throw std::runtime_error("fgo: GnssDDFactor::create eval failed (" +
                                 std::to_string(nv) + ")");
    }
    if (nv == 0) return nullptr;   /* nothing to constrain; caller skips it */

    /* R comes back as nv x nv in RTKLIB's column-major layout.  It is block
       dense by construction -- within a block every DD shares the reference
       satellite's variance -- so it is kept whole rather than reduced to a
       diagonal (plan.md 4.2.1, decision D7 mode (a)). */
    gtsam::Matrix cov(nv, nv);
    for (int i = 0; i < nv; i++) {
        for (int j = 0; j < nv; j++) cov(i, j) = R[i + static_cast<std::size_t>(j) * nv];
    }
    /* Robust kernels are deliberately not applied yet: 4.2.1 notes that
       noiseModel::Robust needs an isotropic or diagonal base, which a block
       covariance is not, and 5.6.2 treats reconciling the two as its own
       piece of work. */
    (void)cfg;
    gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Covariance(cov);

    return SharedFactor<GnssDDFactor>(
        new GnssDDFactor(model, posKey, ctx, xBase, nx, nv, st));
}

gtsam::Vector GnssDDFactor::evaluateError(
    const gtsam::Point3 &p, boost::optional<gtsam::Matrix &> Hp) const
{
    const int nv = evalAt(p);

    if (nv != dim_) {
        /* The row set is frozen, so this cannot happen for a well-formed
           context -- and if it ever does, returning zeros would tell the
           optimizer the factor is perfectly satisfied and let a real
           inconsistency pass as convergence.  Say so instead. */
        throw std::runtime_error("fgo: GnssDDFactor dimension changed (" +
                                 std::to_string(dim_) + " -> " +
                                 std::to_string(nv) + ")");
    }

    /* error = -v : RTKLIB's innovation is z - h(x), GTSAM wants h(x) - z */
    gtsam::Vector e(dim_);
    for (int i = 0; i < dim_; i++) e(i) = -v_[i];

    if (Hp) {
        /* RTKLIB stores H with one COLUMN per measurement, nx entries apart,
           and that column is a row of dh/dx.  GTSAM wants one ROW per
           measurement.  The position block is the first three entries. */
        gtsam::Matrix &J = *Hp;
        J.resize(dim_, 3);
        for (int i = 0; i < dim_; i++) {
            for (int k = 0; k < 3; k++) {
                J(i, k) = H_[k + static_cast<std::size_t>(i) * nx_];
            }
        }
    }
    return e;
}

int GnssDDFactor::rejectedRows() const
{
    int n = 0;
    for (int i = 0; i < dim_ && i < DDRES_MAXROW; i++) {
        if (st_->rowrej[i]) n++;
    }
    return n;
}

}  // namespace fgo
