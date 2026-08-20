/*------------------------------------------------------------------------------
* gtsam_smoke.cpp : GTSAM capability probe for the RTKLIB FGO extension
*
*          Copyright (C) 2026 by RST Ltd., All rights reserved.
*
* This program does NOT test RTKLIB. It asserts that the pinned GTSAM build
* actually provides every backend capability the FGO design document (plan.md)
* depends on, so that a broken or mis-featured GTSAM is caught at environment
* setup time rather than deep inside src/fgo/ development.
*
* probes (each maps to a section of plan.md):
*
*   1 batch LM optimizer            plan.md 4.5.2  (FGO_SOLVER_BATCH)
*   2 incremental fixed-lag smoother plan.md 4.5.4 (FGO_SOLVER_ISAM2, decision D5)
*   3 batch fixed-lag smoother      plan.md 4.5.3  (FGO_SOLVER_SLIDING)
*   4 robust noise models           plan.md 5.5    (Huber/Cauchy/Tukey kernels)
*   4b GNC optimizer               plan.md 5.5.3  (graduated non-convexity)
*   5 full covariance noise model   plan.md 4.2.1  (DD block covariance, D7 mode a)
*   6 marginal covariance recovery  plan.md 4.4.2  (LAMBDA integration)
*   7 numerical derivative helper   plan.md 6.11 G4 (Jacobian verification)
*
* version : $Revision:$ $Date:$
* history : 2026/08/20 1.0 new
*-----------------------------------------------------------------------------*/
#include <cstdio>
#include <functional>
#include <cmath>
#include <string>
#include <vector>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

/* the two fixed-lag smoothers live in gtsam_unstable, not core gtsam -- the
   FGO target must therefore link gtsam_unstable as well as gtsam            */
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using namespace gtsam;

static int nfail = 0;

static void check(const char *name, bool ok, const std::string &detail)
{
    printf("  %-34s %s%s%s\n", name, ok ? "PASS" : "FAIL",
           detail.empty() ? "" : "  ", detail.c_str());
    if (!ok) nfail++;
}

/* 1. batch Levenberg-Marquardt (plan.md 4.5.2) ------------------------------*/
static void probe_batch_lm(void)
{
    NonlinearFactorGraph g;
    Key k0 = Symbol('p', 0), k1 = Symbol('p', 1);

    g.addPrior(k0, Point3(0, 0, 0), noiseModel::Isotropic::Sigma(3, 0.01));
    g.add(BetweenFactor<Point3>(k0, k1, Point3(1, 2, 3),
                                noiseModel::Isotropic::Sigma(3, 0.05)));
    Values init;
    init.insert(k0, Point3(0, 0, 0));
    init.insert(k1, Point3(0, 0, 0));

    Values r = LevenbergMarquardtOptimizer(g, init).optimize();
    Point3 p = r.at<Point3>(k1);
    double err = (p - Point3(1, 2, 3)).norm();
    check("batch LM optimizer", err < 1e-6,
          "|dx|=" + std::to_string(err));
}

/* 2+3. fixed-lag smoothers (plan.md 4.5.3 / 4.5.4) --------------------------*/
template <class Smoother> static double run_fixedlag(double lag)
{
    Smoother sm(lag);
    Point3 truth(0, 0, 0);

    for (int k = 0; k < 8; k++) {
        NonlinearFactorGraph g;
        Values v;
        FixedLagSmootherKeyTimestampMap ts;
        Key kk = Symbol('p', k);

        if (k == 0) {
            g.addPrior(kk, truth, noiseModel::Isotropic::Sigma(3, 0.01));
        } else {
            g.add(BetweenFactor<Point3>(Symbol('p', k - 1), kk, Point3(0, 0, 0),
                                        noiseModel::Isotropic::Sigma(3, 0.02)));
        }
        v.insert(kk, Point3(0.5, 0.5, 0.5));   /* deliberately wrong start */
        ts[kk] = (double)k;
        sm.update(g, v, ts);
    }
    Values est = sm.calculateEstimate();
    return (est.at<Point3>(Symbol('p', 7)) - truth).norm();
}

/* 4. robust kernels (plan.md 5.5) -------------------------------------------*/
static void probe_robust(void)
{
    auto base = noiseModel::Isotropic::Sigma(1, 1.0);
    bool ok = true;
    std::string detail;

    /* The tuning constants and the reference weights below are taken straight
       from the weight table in plan.md 5.5.2.  Asserting them pins down the
       one property the design actually relies on: Huber and Cauchy never let
       an outlier go, whereas Tukey rejects it outright past c.               */
    const double huber_d  = 1.345;
    const double cauchy_c = 2.3849;
    const double tukey_c  = 4.6851;

    auto huber  = noiseModel::mEstimator::Huber::Create(huber_d);
    auto cauchy = noiseModel::mEstimator::Cauchy::Create(cauchy_c);
    auto tukey  = noiseModel::mEstimator::Tukey::Create(tukey_c);

    struct { const char *n; double got, want; } cases[] = {
        /* Huber: w = d/|u| beyond d, strictly positive for every residual   */
        {"huber(2)",   huber->weight(2.0),   huber_d / 2.0},
        {"huber(10)",  huber->weight(10.0),  huber_d / 10.0},
        /* Cauchy: w = 1/(1+(u/c)^2), asymptotically zero but never zero     */
        {"cauchy(10)", cauchy->weight(10.0), 1.0 / (1.0 + (10.0 / cauchy_c) *
                                                         (10.0 / cauchy_c))},
        /* Tukey: w = (1-(u/c)^2)^2 inside c, exactly zero outside           */
        {"tukey(1)",   tukey->weight(1.0),
                       (1.0 - (1.0 / tukey_c) * (1.0 / tukey_c)) *
                       (1.0 - (1.0 / tukey_c) * (1.0 / tukey_c))},
        {"tukey(10)",  tukey->weight(10.0),  0.0},
    };
    for (auto &c : cases) {
        if (std::fabs(c.got - c.want) > 1e-9) {
            ok = false;
            detail += std::string(c.n) + "=" + std::to_string(c.got) +
                      "!=" + std::to_string(c.want) + " ";
        }
    }
    /* redescending vs not -- the distinction plan.md 5.5.2 turns on */
    if (!(huber->weight(1e3) > 0.0))  { ok = false; detail += "huber-redescends "; }
    if (!(cauchy->weight(1e3) > 0.0)) { ok = false; detail += "cauchy-redescends "; }
    if (tukey->weight(1e3) != 0.0)    { ok = false; detail += "tukey-not-redescending "; }

    /* all three must be wrappable as a factor noise model.  Note the shared
       pointer type: GTSAM 4.2 still uses boost::shared_ptr, 4.3 moves to
       std::shared_ptr -- src/fgo/ must not hardcode either spelling.        */
    const std::vector<noiseModel::mEstimator::Base::shared_ptr> ms =
        {huber, cauchy, tukey};
    for (const auto &m : ms) {
        if (!noiseModel::Robust::Create(m, base)) {
            ok = false;
            detail += "robust-create-null ";
        }
    }
    check("robust kernels huber/cauchy/tukey", ok, detail);
}

/* 4b. GNC optimizer (plan.md 5.5.3 -- graduated non-convexity) --------------*/
static void probe_gnc(void)
{
    NonlinearFactorGraph g;
    Key k = Symbol('p', 0);
    auto noise = noiseModel::Isotropic::Sigma(3, 0.01);

    /* three consistent measurements plus one gross outlier; GNC must reject
       the outlier and land on the inlier consensus                          */
    for (int i = 0; i < 3; i++) g.addPrior(k, Point3(1, 2, 3), noise);
    g.addPrior(k, Point3(100, 100, 100), noise);

    Values init;
    init.insert(k, Point3(1, 2, 3));

    using Params = GncParams<LevenbergMarquardtParams>;
    Params p;
    p.setLossType(GncLossType::TLS);
    p.setVerbosityGNC(Params::Verbosity::SILENT);

    GncOptimizer<Params> opt(g, init, p);
    Values r = opt.optimize();
    double err = (r.at<Point3>(k) - Point3(1, 2, 3)).norm();
    check("GNC optimizer (TLS)", err < 1e-3, "|dx|=" + std::to_string(err));
}

/* 5. full covariance noise model (plan.md 4.2.1 covariance, mode (a)) -------*/
static void probe_block_covariance(void)
{
    /* diagonal + rank-1 structure produced by ddcov():
       R = sigma_ref^2 * 1*1' + diag(sigma_j^2)                              */
    const int nb = 4;
    const double sref2 = 0.02;
    Matrix R = Matrix::Constant(nb, nb, sref2);
    for (int i = 0; i < nb; i++) R(i, i) += 0.01 * (i + 1);

    auto m = noiseModel::Gaussian::Covariance(R);
    Vector e = Vector::Ones(nb);
    /* Mahalanobis distance via GTSAM must match the closed form e' R^-1 e   */
    double d_gtsam = 2.0 * m->loss(m->squaredMahalanobisDistance(e));
    double d_ref   = e.transpose() * R.inverse() * e;
    check("full-covariance gaussian model",
          std::fabs(d_gtsam - d_ref) < 1e-9,
          "d=" + std::to_string(d_gtsam) + " ref=" + std::to_string(d_ref));
}

/* 6. marginal covariance recovery (plan.md 4.4.2) ---------------------------*/
static void probe_marginals(void)
{
    NonlinearFactorGraph g;
    Key k = Symbol('p', 0);
    const double sigma = 0.03;

    g.addPrior(k, Point3(0, 0, 0), noiseModel::Isotropic::Sigma(3, sigma));
    Values v;
    v.insert(k, Point3(0, 0, 0));

    Marginals marg(g, v);
    Matrix cov = marg.marginalCovariance(k);
    check("marginal covariance recovery",
          std::fabs(cov(0, 0) - sigma * sigma) < 1e-12,
          "var=" + std::to_string(cov(0, 0)));
}

/* 7. numericalDerivative (plan.md 6.11 G4) ----------------------------------*/
static void probe_numerical_derivative(void)
{
    /* f(p) = |p| ; analytic gradient is p/|p|                               */
    std::function<Vector1(const Point3 &)> f =
        [](const Point3 &p) { return Vector1(p.norm()); };
    Point3 p(1.0, 2.0, 2.0);              /* |p| = 3 */

    Matrix num = numericalDerivative11<Vector1, Point3>(f, p);
    Matrix ana(1, 3);
    ana << p.x() / 3.0, p.y() / 3.0, p.z() / 3.0;

    double rel = (num - ana).norm() / ana.norm();
    check("numericalDerivative11", rel < 1e-6, "rel=" + std::to_string(rel));
}

int main(void)
{
    printf("GTSAM capability probe for RTKLIB FGO (plan.md)\n");
    printf("  gtsam version                      %d.%d.%d\n",
           GTSAM_VERSION_MAJOR, GTSAM_VERSION_MINOR, GTSAM_VERSION_PATCH);

    probe_batch_lm();

    double e_inc = run_fixedlag<IncrementalFixedLagSmoother>(3.0);
    check("incremental fixed-lag smoother", e_inc < 1e-3,
          "|dx|=" + std::to_string(e_inc));

    double e_bat = run_fixedlag<BatchFixedLagSmoother>(3.0);
    check("batch fixed-lag smoother", e_bat < 1e-3,
          "|dx|=" + std::to_string(e_bat));

    probe_robust();
    probe_gnc();
    probe_block_covariance();
    probe_marginals();
    probe_numerical_derivative();

    if (nfail) {
        printf("\n%d probe(s) FAILED -- this GTSAM build cannot host src/fgo/\n",
               nfail);
        return 1;
    }
    printf("\nall probes passed\n");
    return 0;
}
