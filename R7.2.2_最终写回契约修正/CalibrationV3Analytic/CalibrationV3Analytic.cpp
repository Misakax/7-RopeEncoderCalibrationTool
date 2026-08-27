#include "CalibrationV3Analytic.h"

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace
{
using Eigen::JacobiSVD;
using Eigen::Matrix3Xd;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

constexpr double kSvdRelTol = 1e-10;   // close in spirit to V1's 1e-10 rank threshold
constexpr double kTinyLength = 1e-12;
constexpr int kMaxAxisCount = 32;
constexpr int kMaxSampleCount = 100000;

struct Model
{
    VectorXd a;
    VectorXd d;
    VectorXd alpha;
    VectorXd theta;
    VectorXd beta;
    VectorXd q0;
};

struct State
{
    Matrix3Xd points;     // 3 x M
    Vector3d anchor;
    VectorXd residual;    // measured - (predicted geometry + ropeBiasM)
    double ropeBiasM = 0.0;
    double mae = std::numeric_limits<double>::infinity();
    double maxAbs = std::numeric_limits<double>::infinity();
    int anchorRank = 0;
};

thread_local CalibrationV3Trace g_lastTrace = {};

void ResetTrace(int axis)
{
    std::memset(&g_lastTrace, 0, sizeof(g_lastTrace));
    g_lastTrace.axis = axis;
}

void AppendTrace(int iteration, double stepScale, const Model& model,
                 const State& state, const VectorXd& appliedDelta)
{
    if (g_lastTrace.count >= CALV3_TRACE_MAX_ITERATIONS ||
        model.a.size() > CALV3_TRACE_MAX_AXIS)
        return;
    CalibrationV3IterationRecord& rec = g_lastTrace.record[g_lastTrace.count++];
    std::memset(&rec, 0, sizeof(rec));
    rec.iteration = iteration;
    rec.stepScale = stepScale;
    rec.maeM = state.mae;
    rec.maxAbsM = state.maxAbs;
    rec.ropeBiasM = state.ropeBiasM;
    for (int xyz = 0; xyz < 3; ++xyz) rec.anchor[xyz] = state.anchor(xyz);
    const int axis = static_cast<int>(model.a.size());
    for (int joint = 0; joint < axis; ++joint)
    {
        rec.a[joint] = model.a(joint);
        rec.d[joint] = model.d(joint);
        rec.alpha[joint] = model.alpha(joint);
        rec.q0[joint] = model.q0(joint);
    }
    const int deltaCount = std::min<int>(static_cast<int>(appliedDelta.size()),
                                         4 * CALV3_TRACE_MAX_AXIS);
    for (int i = 0; i < deltaCount; ++i) rec.appliedDelta4N[i] = appliedDelta(i);
}

struct LinearSolveInfo
{
    int rank = 0;
    double condition = std::numeric_limits<double>::infinity();
};

inline bool IsFinite(double v)
{
    return std::isfinite(v) != 0;
}

bool AllFinite(const VectorXd& v)
{
    return v.array().isFinite().all();
}

bool AllFinite(const MatrixXd& m)
{
    return m.array().isFinite().all();
}

Matrix4d JointTransform(double q, double a, double d, double alpha, double beta)
{
    const double cq = std::cos(q);
    const double sq = std::sin(q);
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);
    const double cb = std::cos(beta);
    const double sb = std::sin(beta);

    Matrix4d T = Matrix4d::Zero();
    T(0, 0) = cq * cb;
    T(0, 1) = -sq;
    T(0, 2) = cq * sb;
    T(0, 3) = a;

    T(1, 0) = sq * ca * cb + sa * sb;
    T(1, 1) = cq * ca;
    T(1, 2) = sq * ca * sb - sa * cb;
    T(1, 3) = -d * sa;

    T(2, 0) = sq * sa * cb - ca * sb;
    T(2, 1) = cq * sa;
    T(2, 2) = sq * sa * sb + ca * cb;
    T(2, 3) = d * ca;

    T(3, 3) = 1.0;
    return T;
}

// Exact partial dT/da.
Matrix4d DTransformDa()
{
    Matrix4d D = Matrix4d::Zero();
    D(0, 3) = 1.0;
    return D;
}

// Exact partial dT/dd.
Matrix4d DTransformDd(double alpha)
{
    Matrix4d D = Matrix4d::Zero();
    D(1, 3) = -std::sin(alpha);
    D(2, 3) =  std::cos(alpha);
    return D;
}

// Exact partial dT/dalpha for the exact V1 GetJ2JTrans convention.
Matrix4d DTransformDAlpha(double q, double d, double alpha, double beta)
{
    const double cq = std::cos(q);
    const double sq = std::sin(q);
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);
    const double cb = std::cos(beta);
    const double sb = std::sin(beta);

    Matrix4d D = Matrix4d::Zero();

    D(1, 0) = -sq * sa * cb + ca * sb;
    D(1, 1) = -cq * sa;
    D(1, 2) = -sq * sa * sb - ca * cb;
    D(1, 3) = -d * ca;

    D(2, 0) =  sq * ca * cb + sa * sb;
    D(2, 1) =  cq * ca;
    D(2, 2) =  sq * ca * sb - sa * cb;
    D(2, 3) = -d * sa;

    return D;
}

// Exact partial dT/dq. Because q_eff = q_measured + q0 + thetaFixed,
// dT/dq0 is identical to dT/dq.
Matrix4d DTransformDq(double q, double alpha, double beta)
{
    const double cq = std::cos(q);
    const double sq = std::sin(q);
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);
    const double cb = std::cos(beta);
    const double sb = std::sin(beta);

    Matrix4d D = Matrix4d::Zero();

    D(0, 0) = -sq * cb;
    D(0, 1) = -cq;
    D(0, 2) = -sq * sb;

    D(1, 0) =  cq * ca * cb;
    D(1, 1) = -sq * ca;
    D(1, 2) =  cq * ca * sb;

    D(2, 0) =  cq * sa * cb;
    D(2, 1) = -sq * sa;
    D(2, 2) =  cq * sa * sb;

    return D;
}

Matrix4d FixtureTransform(const double fixtureLocalXYZ[3])
{
    Matrix4d T = Matrix4d::Identity();
    T(0, 3) = fixtureLocalXYZ[0];
    T(1, 3) = fixtureLocalXYZ[1];
    T(2, 3) = fixtureLocalXYZ[2];
    return T;
}

// Computes measurement point and, optionally, the exact 3 x (4N) position Jacobian.
// Parameter layout: [a | d | alpha | q0].
void MeasurementPointAndJacobian(
    const Model& model,
    const double* qMeasured,
    const double fixtureLocalXYZ[3],
    Vector3d& point,
    MatrixXd* positionJacobian)
{
    const int N = static_cast<int>(model.a.size());

    std::vector<Matrix4d, Eigen::aligned_allocator<Matrix4d>> T(static_cast<size_t>(N));
    std::vector<double> qEff(static_cast<size_t>(N));

    for (int i = 0; i < N; ++i)
    {
        qEff[static_cast<size_t>(i)] = qMeasured[i] + model.q0(i) + model.theta(i);
        T[static_cast<size_t>(i)] = JointTransform(
            qEff[static_cast<size_t>(i)],
            model.a(i), model.d(i), model.alpha(i), model.beta(i));
    }

    const Matrix4d Tfixture = FixtureTransform(fixtureLocalXYZ);

    // prefix[i] = T1 * ... * Ti, with prefix[0] = I and prefix[i] before T_i in derivative use.
    std::vector<Matrix4d, Eigen::aligned_allocator<Matrix4d>> prefix(static_cast<size_t>(N + 1));
    prefix[0] = Matrix4d::Identity();
    for (int i = 0; i < N; ++i)
        prefix[static_cast<size_t>(i + 1)] = prefix[static_cast<size_t>(i)] * T[static_cast<size_t>(i)];

    // suffix[i] = Ti * ... * TN * Tfixture; suffix[N] = Tfixture.
    std::vector<Matrix4d, Eigen::aligned_allocator<Matrix4d>> suffix(static_cast<size_t>(N + 1));
    suffix[static_cast<size_t>(N)] = Tfixture;
    for (int i = N - 1; i >= 0; --i)
        suffix[static_cast<size_t>(i)] = T[static_cast<size_t>(i)] * suffix[static_cast<size_t>(i + 1)];

    const Vector4d origin(0.0, 0.0, 0.0, 1.0);
    const Vector4d p4 = prefix[static_cast<size_t>(N)] * Tfixture * origin;
    point = p4.head<3>();

    if (positionJacobian == nullptr)
        return;

    positionJacobian->resize(3, 4 * N);
    positionJacobian->setZero();

    for (int i = 0; i < N; ++i)
    {
        const Matrix4d left  = prefix[static_cast<size_t>(i)];
        const Matrix4d right = suffix[static_cast<size_t>(i + 1)];

        const Matrix4d Da = DTransformDa();
        const Matrix4d Dd = DTransformDd(model.alpha(i));
        const Matrix4d Dal = DTransformDAlpha(
            qEff[static_cast<size_t>(i)], model.d(i), model.alpha(i), model.beta(i));
        const Matrix4d Dq = DTransformDq(
            qEff[static_cast<size_t>(i)], model.alpha(i), model.beta(i));

        const Vector3d dPda = (left * Da  * right * origin).head<3>();
        const Vector3d dPdd = (left * Dd  * right * origin).head<3>();
        const Vector3d dPdal= (left * Dal * right * origin).head<3>();
        const Vector3d dPdq = (left * Dq  * right * origin).head<3>();

        positionJacobian->col(i)         = dPda;
        positionJacobian->col(N + i)     = dPdd;
        positionJacobian->col(2 * N + i) = dPdal;
        positionJacobian->col(3 * N + i) = dPdq;
    }
}

int NumericalRankAndCondition(const VectorXd& singularValues, int rows, int cols,
                              int& rank, double& condition)
{
    rank = 0;
    condition = std::numeric_limits<double>::infinity();
    if (singularValues.size() == 0)
        return CALV3_ERR_NONFINITE;

    const double sMax = singularValues(0);
    if (!IsFinite(sMax) || sMax <= 0.0)
        return CALV3_ERR_NONFINITE;

    const double tol = kSvdRelTol * std::max(rows, cols) * sMax;
    double sMinKept = std::numeric_limits<double>::infinity();
    for (int i = 0; i < singularValues.size(); ++i)
    {
        const double s = singularValues(i);
        if (s > tol)
        {
            ++rank;
            sMinKept = std::min(sMinKept, s);
        }
    }

    if (rank > 0 && IsFinite(sMinKept) && sMinKept > 0.0)
        condition = sMax / sMinKept;

    return CALV3_OK;
}

VectorXd PseudoInverseSolve(const MatrixXd& A, const VectorXd& b, LinearSolveInfo& info)
{
    JacobiSVD<MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const VectorXd s = svd.singularValues();
    NumericalRankAndCondition(s, A.rows(), A.cols(), info.rank, info.condition);

    VectorXd invS = VectorXd::Zero(s.size());
    if (s.size() > 0)
    {
        const double sMax = s(0);
        const double tol = kSvdRelTol * std::max(A.rows(), A.cols()) * sMax;
        for (int i = 0; i < s.size(); ++i)
            if (s(i) > tol)
                invS(i) = 1.0 / s(i);
    }

    return svd.matrixV() * invS.asDiagonal() * svd.matrixU().transpose() * b;
}

int SolveAnchorV1Adjacent(const Matrix3Xd& points, const VectorXd& measured,
                          Vector3d& anchor, int& rank)
{
    const int M = static_cast<int>(points.cols());
    if (M < 4)
        return CALV3_ERR_BAD_ARGUMENT;

    MatrixXd A(M - 1, 3);
    VectorXd b(M - 1);

    for (int i = 0; i < M - 1; ++i)
    {
        const Vector3d p0 = points.col(i);
        const Vector3d p1 = points.col(i + 1);
        A.row(i) = (2.0 * (p0 - p1)).transpose();
        b(i) = measured(i + 1) * measured(i + 1)
             - measured(i) * measured(i)
             + p0.squaredNorm() - p1.squaredNorm();
    }

    LinearSolveInfo info;
    const VectorXd x = PseudoInverseSolve(A, b, info);
    rank = info.rank;
    if (rank < 3 || x.size() != 3 || !AllFinite(x))
        return CALV3_ERR_ANCHOR_RANK;

    anchor = x.head<3>();

    return CALV3_OK;
}

int EvaluateState(const Model& model,
                  int sampleCount,
                  const double* jointRad,
                  const VectorXd& measured,
                  const double fixtureLocalXYZ[3],
                  State& state)
{
    const int N = static_cast<int>(model.a.size());
    state.points.resize(3, sampleCount);
    state.ropeBiasM = 0.0;

    for (int s = 0; s < sampleCount; ++s)
    {
        Vector3d P;
        MeasurementPointAndJacobian(model, jointRad + s * N, fixtureLocalXYZ, P, nullptr);
        if (!P.array().isFinite().all())
            return CALV3_ERR_NONFINITE;
        state.points.col(s) = P;
    }

    int status = SolveAnchorV1Adjacent(state.points, measured, state.anchor, state.anchorRank);
    if (status != CALV3_OK)
        return status;

    state.residual.resize(sampleCount);
    double sumAbs = 0.0;
    double maxAbs = 0.0;

    for (int s = 0; s < sampleCount; ++s)
    {
        const double predicted = (state.points.col(s) - state.anchor).norm();
        if (!IsFinite(predicted) || predicted <= kTinyLength)
            return CALV3_ERR_INVALID_LENGTH;

        const double r = measured(s) - predicted;
        if (!IsFinite(r))
            return CALV3_ERR_NONFINITE;
        state.residual(s) = r;
        sumAbs += std::abs(r);
        maxAbs = std::max(maxAbs, std::abs(r));
    }

    state.mae = sumAbs / static_cast<double>(sampleCount);
    state.maxAbs = maxAbs;
    return CALV3_OK;
}

int EvaluateStateAtAnchor(const Model& model,
                          int sampleCount,
                          const double* jointRad,
                          const VectorXd& measured,
                          const double fixtureLocalXYZ[3],
                          const Vector3d& anchor,
                          double ropeBiasM,
                          State& state)
{
    const int N = static_cast<int>(model.a.size());
    state.points.resize(3, sampleCount);
    state.anchor = anchor;
    state.anchorRank = 3;
    state.ropeBiasM = ropeBiasM;
    state.residual.resize(sampleCount);

    double sumAbs = 0.0;
    double maxAbs = 0.0;
    for (int s = 0; s < sampleCount; ++s)
    {
        Vector3d point;
        MeasurementPointAndJacobian(model, jointRad + s * N,
                                    fixtureLocalXYZ, point, nullptr);
        if (!point.array().isFinite().all())
            return CALV3_ERR_NONFINITE;
        state.points.col(s) = point;

        const double predicted = (point - anchor).norm();
        if (!IsFinite(predicted) || predicted <= kTinyLength)
            return CALV3_ERR_INVALID_LENGTH;
        const double residual = measured(s) - predicted - ropeBiasM;
        if (!IsFinite(residual))
            return CALV3_ERR_NONFINITE;
        state.residual(s) = residual;
        sumAbs += std::abs(residual);
        maxAbs = std::max(maxAbs, std::abs(residual));
    }

    state.mae = sumAbs / static_cast<double>(sampleCount);
    state.maxAbs = maxAbs;
    return CALV3_OK;
}

void ApplyDelta(Model& model, const VectorXd& fullDelta)
{
    const int N = static_cast<int>(model.a.size());
    model.a     += fullDelta.segment(0, N);
    model.d     += fullDelta.segment(N, N);
    model.alpha += fullDelta.segment(2 * N, N);
    model.q0    += fullDelta.segment(3 * N, N);
}

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kDegToRad = kPi / 180.0;

// R7.2 HE3 laser-population prior.
// Source: the eight machines in "弓叶HE3激光跟踪仪校准后的DH和零点参数.xlsx".
// The policy is enabled only when the incoming model matches the public HE3
// nominal Craig MDH. Other callers keep R7.1's generic weak-prior behaviour.
struct He3LaserPrior
{
    bool enabled = false;
    VectorXd targetDelta;   // target - public-input parameter
    VectorXd sigma;         // sample standard deviation
    VectorXd lowerDelta;    // mean - 3 sigma - public input
    VectorXd upperDelta;    // mean + 3 sigma - public input
};

bool LooksLikePublicHe3(const Model& initial)
{
    if (initial.a.size() != 7) return false;

    const double dNominal[7] = {0.0, 0.049, 0.493, -0.120, 0.317, 0.0, 0.142};
    const double alphaNominalDeg[7] = {0.0, 90.0, -90.0, 90.0, -90.0, 90.0, -90.0};
    for (int i = 0; i < 7; ++i)
    {
        if (std::abs(initial.a(i)) > 0.002) return false;
        if (std::abs(initial.d(i) - dNominal[i]) > 0.005) return false;
        if (std::abs(initial.alpha(i) - alphaNominalDeg[i] * kDegToRad) > 2.0 * kDegToRad)
            return false;
        if (std::abs(initial.q0(i)) > 2.0 * kDegToRad) return false;
    }
    return true;
}

He3LaserPrior BuildHe3LaserPrior(const Model& initial)
{
    const int axis = static_cast<int>(initial.a.size());
    He3LaserPrior p;
    p.targetDelta = VectorXd::Zero(4 * axis);
    p.sigma = VectorXd::Constant(4 * axis, std::numeric_limits<double>::infinity());
    p.lowerDelta = VectorXd::Constant(4 * axis, -std::numeric_limits<double>::infinity());
    p.upperDelta = VectorXd::Constant(4 * axis,  std::numeric_limits<double>::infinity());

    if (!LooksLikePublicHe3(initial)) return p;
    p.enabled = true;

    // D2..D5, mean/sample-sigma from eight laser-calibrated machines [mm].
    const double dMeanMm[4] = {48.476025, 492.60525, -120.345875, 317.0595};
    const double dSigmaMm[4] = {0.2654127877, 0.2669492996, 0.5993695348, 0.5020122366};
    for (int j = 0; j < 4; ++j)
    {
        const int slot = 1 + j;
        const int idx = axis + slot;
        const double mean = dMeanMm[j] * 1e-3;
        const double sigma = dSigmaMm[j] * 1e-3;
        p.targetDelta(idx) = mean - initial.d(slot);
        p.sigma(idx) = std::max(sigma, 0.00020); // never claim <0.2 mm population certainty
        p.lowerDelta(idx) = (mean - 3.0 * sigma) - initial.d(slot);
        p.upperDelta(idx) = (mean + 3.0 * sigma) - initial.d(slot);
    }

    // Craig slots 1..6 correspond to physical Alpha1..Alpha6 [deg].
    const double alphaMeanDeg[6] = {
        90.132725, -90.055725, 89.97261625, -89.8612625, 89.9082, -89.8848125
    };
    const double alphaSigmaDeg[6] = {
        0.1014841543, 0.1854501146, 0.06552157637,
        0.1859791074, 0.2700252475, 0.2798704008
    };
    for (int j = 0; j < 6; ++j)
    {
        const int slot = 1 + j;
        const int idx = 2 * axis + slot;
        const double mean = alphaMeanDeg[j] * kDegToRad;
        const double sigma = alphaSigmaDeg[j] * kDegToRad;
        p.targetDelta(idx) = mean - initial.alpha(slot);
        p.sigma(idx) = std::max(sigma, 0.05 * kDegToRad);
        p.lowerDelta(idx) = (mean - 3.0 * sigma) - initial.alpha(slot);
        p.upperDelta(idx) = (mean + 3.0 * sigma) - initial.alpha(slot);
    }

    // q0 slots 1..5 are q2..q6. The workbook states that its zero-compensation
    // angle is the opposite sign of the theta/q0 compensation used by the model.
    const double q0MeanDeg[5] = {
        -0.20794723, 0.15649, -0.1583614, -0.068745875, 0.0961529875
    };
    const double q0SigmaDeg[5] = {
        0.2036069439, 0.2329085981, 0.1279960884, 0.2995546803, 0.3288645275
    };
    for (int j = 0; j < 5; ++j)
    {
        const int slot = 1 + j;
        const int idx = 3 * axis + slot;
        const double mean = q0MeanDeg[j] * kDegToRad;
        const double sigma = q0SigmaDeg[j] * kDegToRad;
        p.targetDelta(idx) = mean - initial.q0(slot);
        p.sigma(idx) = std::max(sigma, 0.10 * kDegToRad);
        p.lowerDelta(idx) = (mean - 3.0 * sigma) - initial.q0(slot);
        p.upperDelta(idx) = (mean + 3.0 * sigma) - initial.q0(slot);
    }

    return p;
}

double GenericPriorSigma(int fullIndex, int axis)
{
    // R7.1 fallback for non-HE3/synthetic callers.
    if (fullIndex < 2 * axis) return 0.100;
    return 30.0 * kDegToRad;
}

double GenericHardLimit(int fullIndex, int axis)
{
    if (fullIndex < 2 * axis) return 0.003;
    if (fullIndex < 3 * axis) return 1.5 * kDegToRad;
    return 2.0 * kDegToRad;
}

bool InsidePhysicalEnvelope(const VectorXd& accumulated,
                            const std::vector<int>& activeIndex,
                            const He3LaserPrior* laserPrior)
{
    const int axis = static_cast<int>(accumulated.size()) / 4;
    for (int k = 0; k < static_cast<int>(activeIndex.size()); ++k)
    {
        const int index = activeIndex[static_cast<size_t>(k)];
        if (laserPrior && laserPrior->enabled &&
            IsFinite(laserPrior->lowerDelta(index)) && IsFinite(laserPrior->upperDelta(index)))
        {
            if (accumulated(index) < laserPrior->lowerDelta(index) ||
                accumulated(index) > laserPrior->upperDelta(index))
                return false;
        }
        else if (std::abs(accumulated(index)) > GenericHardLimit(index, axis))
        {
            return false;
        }
    }
    return true;
}

int BuildRopeJacobian(const Model& model,
                      int sampleCount,
                      const double* jointRad,
                      const VectorXd& measured,
                      const double fixtureLocalXYZ[3],
                      const State& state,
                      const std::vector<int>& activeIndex,
                      MatrixXd& Jactive,
                      LinearSolveInfo& info)
{
    const int N = static_cast<int>(model.a.size());
    const int K = static_cast<int>(activeIndex.size());
    (void)measured;
    // The last four columns are nuisance variables: three fixed-anchor
    // corrections followed by the independent pull-wire bias bL. The exact
    // measurement model is L_measured = |P-lambda| + bL, so dbL has a constant
    // +1 column. bL is deliberately outside the 4N robot parameter vector.
    Jactive.resize(sampleCount, K + 4);

    std::vector<MatrixXd> positionJacobians(static_cast<size_t>(sampleCount));
    std::vector<Vector3d> ropeDirections(static_cast<size_t>(sampleCount));
    for (int s = 0; s < sampleCount; ++s)
    {
        Vector3d P;
        MeasurementPointAndJacobian(model, jointRad + s * N, fixtureLocalXYZ,
                                    P, &positionJacobians[static_cast<size_t>(s)]);

        const Vector3d ropeVec = P - state.anchor;
        const double len = ropeVec.norm();
        if (!IsFinite(len) || len <= kTinyLength)
            return CALV3_ERR_INVALID_LENGTH;

        ropeDirections[static_cast<size_t>(s)] = ropeVec / len;
    }

    for (int s = 0; s < sampleCount; ++s)
    {
        const Vector3d& direction = ropeDirections[static_cast<size_t>(s)];
        for (int k = 0; k < K; ++k)
        {
            const int parameterColumn = activeIndex[static_cast<size_t>(k)];
            Jactive(s, k) = direction.dot(
                positionJacobians[static_cast<size_t>(s)].col(parameterColumn));
        }
        Jactive.block<1, 3>(s, K) = -direction.transpose();
        Jactive(s, K + 3) = 1.0;
    }

    if (!AllFinite(Jactive))
        return CALV3_ERR_NONFINITE;

    JacobiSVD<MatrixXd> svd(Jactive, Eigen::ComputeThinU | Eigen::ComputeThinV);
    NumericalRankAndCondition(svd.singularValues(), Jactive.rows(), Jactive.cols(), info.rank, info.condition);
    return CALV3_OK;
}

int BuildRobotOnlyJacobian(const Model& model,
                           int sampleCount,
                           const double* jointRad,
                           const double fixtureLocalXYZ[3],
                           const State& state,
                           const std::vector<int>& activeIndex,
                           MatrixXd& Jactive,
                           LinearSolveInfo& info)
{
    const int N = static_cast<int>(model.a.size());
    const int K = static_cast<int>(activeIndex.size());
    Jactive.resize(sampleCount, K);

    for (int s = 0; s < sampleCount; ++s)
    {
        Vector3d P;
        MatrixXd positionJacobian;
        MeasurementPointAndJacobian(model, jointRad + s * N, fixtureLocalXYZ,
                                    P, &positionJacobian);
        const Vector3d ropeVec = P - state.anchor;
        const double len = ropeVec.norm();
        if (!IsFinite(len) || len <= kTinyLength) return CALV3_ERR_INVALID_LENGTH;
        const Vector3d direction = ropeVec / len;

        for (int k = 0; k < K; ++k)
        {
            const int parameterColumn = activeIndex[static_cast<size_t>(k)];
            Jactive(s, k) = direction.dot(positionJacobian.col(parameterColumn));
        }
    }

    if (!AllFinite(Jactive)) return CALV3_ERR_NONFINITE;
    JacobiSVD<MatrixXd> svd(Jactive, Eigen::ComputeThinU | Eigen::ComputeThinV);
    NumericalRankAndCondition(svd.singularValues(), Jactive.rows(), Jactive.cols(),
                              info.rank, info.condition);
    return CALV3_OK;
}

void CopyModelToOutput(const Model& model,
                       double* outA, double* outD, double* outAlpha, double* outQ0)
{
    const int N = static_cast<int>(model.a.size());
    for (int i = 0; i < N; ++i)
    {
        outA[i] = model.a(i);
        outD[i] = model.d(i);
        outAlpha[i] = model.alpha(i);
        outQ0[i] = model.q0(i);
    }
}

} // namespace

extern "C" CALV3_API const char* CALV3_CALL CalibrationV3BuildId()
{
    return "HE3-V3-CRAIG-MDH-ANALYTIC-TWOSTAGE-LASER-PRIOR-FINAL-WRITEBACK-20260826-R7.2.2";
}

extern "C" CALV3_API int CALV3_CALL CalibrationV3GetLastTrace(
    CalibrationV3Trace* outTrace)
{
    if (!outTrace) return CALV3_ERR_BAD_ARGUMENT;
    std::memcpy(outTrace, &g_lastTrace, sizeof(g_lastTrace));
    return CALV3_OK;
}

extern "C" CALV3_API int CALV3_CALL CalibrationV3EvaluatePointAndJacobian(
    int axis,
    const double* a,
    const double* d,
    const double* alpha,
    const double* thetaFixed,
    const double* beta,
    const double* q0,
    const double* jointRadOnePose,
    const double fixtureLocalXYZ[3],
    double outPoint3[3],
    double* outJacobian3x4N)
{
    if (axis <= 0 || axis > kMaxAxisCount || !a || !d || !alpha || !thetaFixed || !beta || !q0 ||
        !jointRadOnePose || !fixtureLocalXYZ || !outPoint3 || !outJacobian3x4N)
        return CALV3_ERR_BAD_ARGUMENT;

    try
    {
        for (int i = 0; i < axis; ++i)
            if (!IsFinite(a[i]) || !IsFinite(d[i]) || !IsFinite(alpha[i]) ||
                !IsFinite(thetaFixed[i]) || !IsFinite(beta[i]) || !IsFinite(q0[i]) ||
                !IsFinite(jointRadOnePose[i]))
                return CALV3_ERR_NONFINITE;
        for (int i = 0; i < 3; ++i)
            if (!IsFinite(fixtureLocalXYZ[i]))
                return CALV3_ERR_NONFINITE;

        Model model;
        model.a.resize(axis);
        model.d.resize(axis);
        model.alpha.resize(axis);
        model.theta.resize(axis);
        model.beta.resize(axis);
        model.q0.resize(axis);
        for (int i = 0; i < axis; ++i)
        {
            model.a(i) = a[i];
            model.d(i) = d[i];
            model.alpha(i) = alpha[i];
            model.theta(i) = thetaFixed[i];
            model.beta(i) = beta[i];
            model.q0(i) = q0[i];
        }

        Vector3d P;
        MatrixXd J;
        MeasurementPointAndJacobian(model, jointRadOnePose, fixtureLocalXYZ, P, &J);
        if (!P.array().isFinite().all() || !AllFinite(J))
            return CALV3_ERR_NONFINITE;

        for (int r = 0; r < 3; ++r)
            outPoint3[r] = P(r);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4 * axis; ++c)
                outJacobian3x4N[r * (4 * axis) + c] = J(r, c);

        return CALV3_OK;
    }
    catch (...)
    {
        return CALV3_ERR_INTERNAL;
    }
}

extern "C" CALV3_API int CALV3_CALL CalibrationV3Analytic(
    int axis,
    int sampleCount,
    const double* a,
    const double* d,
    const double* alpha,
    const double* thetaFixed,
    const double* beta,
    const double* q0Initial,
    const double* jointRad,
    const double* ropeRawM,
    double ropeFixedOffsetM,
    const double fixtureLocalXYZ[3],
    const int* activeMask4N,
    int maxIterations,
    double minMaeImprovementM,
    int requireFullColumnRank,
    double maxCondition,
    double* outA,
    double* outD,
    double* outAlpha,
    double* outQ0,
    double* outDelta4N,
    double* residualBeforeM,
    double* residualAfterM,
    CalibrationV3Report* report)
{
    if (axis <= 0 || axis > kMaxAxisCount || sampleCount < 4 || sampleCount > kMaxSampleCount ||
        !a || !d || !alpha || !thetaFixed || !beta ||
        !q0Initial || !jointRad || !ropeRawM || !fixtureLocalXYZ || !activeMask4N ||
        !outA || !outD || !outAlpha || !outQ0 || !outDelta4N ||
        !residualBeforeM || !residualAfterM || !report || maxIterations < 0 ||
        maxIterations > 1000 || !IsFinite(ropeFixedOffsetM) ||
        !IsFinite(minMaeImprovementM) || minMaeImprovementM < 0.0 ||
        !IsFinite(maxCondition))
        return CALV3_ERR_BAD_ARGUMENT;

    std::memset(report, 0, sizeof(*report));
    report->status = CALV3_ERR_BAD_ARGUMENT;
    ResetTrace(axis);

    try
    {
    for (int i = 0; i < axis; ++i)
        if (!IsFinite(a[i]) || !IsFinite(d[i]) || !IsFinite(alpha[i]) ||
            !IsFinite(thetaFixed[i]) || !IsFinite(beta[i]) || !IsFinite(q0Initial[i]))
        {
            report->status = CALV3_ERR_NONFINITE;
            return report->status;
        }
    for (int i = 0; i < 3; ++i)
        if (!IsFinite(fixtureLocalXYZ[i]))
        {
            report->status = CALV3_ERR_NONFINITE;
            return report->status;
        }
    for (int i = 0; i < axis * sampleCount; ++i)
        if (!IsFinite(jointRad[i]))
        {
            report->status = CALV3_ERR_NONFINITE;
            return report->status;
        }

    Model initial;
    initial.a.resize(axis);
    initial.d.resize(axis);
    initial.alpha.resize(axis);
    initial.theta.resize(axis);
    initial.beta.resize(axis);
    initial.q0.resize(axis);

    for (int i = 0; i < axis; ++i)
    {
        initial.a(i) = a[i];
        initial.d(i) = d[i];
        initial.alpha(i) = alpha[i];
        initial.theta(i) = thetaFixed[i];
        initial.beta(i) = beta[i];
        initial.q0(i) = q0Initial[i];
    }

    const He3LaserPrior laserPrior = BuildHe3LaserPrior(initial);

    VectorXd measured(sampleCount);
    for (int s = 0; s < sampleCount; ++s)
    {
        measured(s) = ropeRawM[s] + ropeFixedOffsetM;
        if (!IsFinite(measured(s)) || measured(s) <= kTinyLength)
        {
            report->status = CALV3_ERR_INVALID_LENGTH;
            return report->status;
        }
    }

    std::vector<int> activeIndex;
    activeIndex.reserve(static_cast<size_t>(4 * axis));
    for (int i = 0; i < 4 * axis; ++i)
        if (activeMask4N[i] != 0)
            activeIndex.push_back(i);

    report->activeCount = static_cast<int>(activeIndex.size());
    if (activeIndex.empty())
    {
        report->status = CALV3_ERR_BAD_ARGUMENT;
        return report->status;
    }

    State before;
    int status = EvaluateState(initial, sampleCount, jointRad, measured, fixtureLocalXYZ, before);
    if (status != CALV3_OK)
    {
        report->status = status;
        return status;
    }

    report->maeBeforeM = before.mae;
    report->maxAbsBeforeM = before.maxAbs;
    report->anchorRank = before.anchorRank;
    for (int i = 0; i < 3; ++i)
        report->anchorBefore[i] = before.anchor(i);
    for (int s = 0; s < sampleCount; ++s)
        residualBeforeM[s] = before.residual(s);

    // R7.1: separate the common pull-wire absolute-length zero offset from robot
    // geometry. The raw before-calibration metrics above intentionally use bL=0.
    // Initialize bL by the least-squares constant offset, then jointly refine
    // robot parameters, anchor and bL.
    const double initialRopeBiasM = before.residual.mean();
    Model current = initial;
    State currentState;
    status = EvaluateStateAtAnchor(current, sampleCount, jointRad, measured,
                                   fixtureLocalXYZ, before.anchor,
                                   initialRopeBiasM, currentState);
    if (status != CALV3_OK)
    {
        report->status = status;
        return status;
    }
    VectorXd accumulatedDelta = VectorXd::Zero(4 * axis);
    AppendTrace(0, 0.0, current, currentState, VectorXd::Zero(4 * axis));

    // R7.2 HE3 path: Stage 1 is now complete. The sphere-derived anchor and the
    // scalar bL are frozen from here on. Stage 2 identifies only robot parameters
    // and uses the eight-machine laser population as a real statistical prior.
    if (laserPrior.enabled)
    {
        const Vector3d fixedAnchor = currentState.anchor;
        const double fixedRopeBiasM = currentState.ropeBiasM;
        int he3FinalRank = 0;
        double he3FinalCondition = std::numeric_limits<double>::infinity();
        int he3AcceptedIterations = 0;
        int he3StopStatus = CALV3_OK;

        for (int iter = 0; iter < maxIterations; ++iter)
        {
            MatrixXd Jactive;
            LinearSolveInfo jInfo;
            status = BuildRobotOnlyJacobian(current, sampleCount, jointRad, fixtureLocalXYZ,
                                            currentState, activeIndex, Jactive, jInfo);
            if (status != CALV3_OK)
            {
                report->status = status;
                return status;
            }

            he3FinalRank = jInfo.rank;
            he3FinalCondition = jInfo.condition;
            const int K = static_cast<int>(activeIndex.size());
            if (requireFullColumnRank != 0 && jInfo.rank < K)
            {
                report->jacobianRank = jInfo.rank;
                report->jacobianCondition = jInfo.condition;
                report->anchorRank = currentState.anchorRank;
                report->maeAfterM = currentState.mae;
                report->maxAbsAfterM = currentState.maxAbs;
                report->ropeBiasM = currentState.ropeBiasM;
                for (int i = 0; i < 3; ++i) report->anchorAfter[i] = currentState.anchor(i);
                report->status = CALV3_ERR_JACOBIAN_RANK;
                CopyModelToOutput(current, outA, outD, outAlpha, outQ0);
                for (int i = 0; i < 4 * axis; ++i) outDelta4N[i] = accumulatedDelta(i);
                for (int s = 0; s < sampleCount; ++s) residualAfterM[s] = currentState.residual(s);
                return report->status;
            }
            if (maxCondition > 0.0 && (!IsFinite(jInfo.condition) || jInfo.condition > maxCondition))
            {
                report->jacobianRank = jInfo.rank;
                report->jacobianCondition = jInfo.condition;
                report->anchorRank = currentState.anchorRank;
                report->maeAfterM = currentState.mae;
                report->maxAbsAfterM = currentState.maxAbs;
                report->ropeBiasM = currentState.ropeBiasM;
                for (int i = 0; i < 3; ++i) report->anchorAfter[i] = currentState.anchor(i);
                report->status = CALV3_ERR_JACOBIAN_RANK;
                CopyModelToOutput(current, outA, outD, outAlpha, outQ0);
                for (int i = 0; i < 4 * axis; ++i) outDelta4N[i] = accumulatedDelta(i);
                for (int s = 0; s < sampleCount; ++s) residualAfterM[s] = currentState.residual(s);
                return report->status;
            }

            const double measurementSigmaM = 0.0015;
            const double huberM = 0.005;
            MatrixXd Jsolve = MatrixXd::Zero(sampleCount + K, K);
            VectorXd rsolve = VectorXd::Zero(sampleCount + K);
            for (int s = 0; s < sampleCount; ++s)
            {
                const double absResidual = std::abs(currentState.residual(s));
                const double huberWeight = (absResidual <= huberM || absResidual <= kTinyLength)
                    ? 1.0 : std::sqrt(huberM / absResidual);
                const double rowScale = huberWeight / measurementSigmaM;
                Jsolve.row(s) = rowScale * Jactive.row(s);
                rsolve(s) = rowScale * currentState.residual(s);
            }
            for (int k = 0; k < K; ++k)
            {
                const int fullIndex = activeIndex[static_cast<size_t>(k)];
                const double sigma = IsFinite(laserPrior.sigma(fullIndex))
                    ? laserPrior.sigma(fullIndex) : GenericPriorSigma(fullIndex, axis);
                const double target = IsFinite(laserPrior.sigma(fullIndex))
                    ? laserPrior.targetDelta(fullIndex) : 0.0;
                Jsolve(sampleCount + k, k) = 1.0 / sigma;
                rsolve(sampleCount + k) = (target - accumulatedDelta(fullIndex)) / sigma;
            }

            LinearSolveInfo solveInfo;
            const VectorXd deltaJoint = PseudoInverseSolve(Jsolve, rsolve, solveInfo);
            if (!AllFinite(deltaJoint))
            {
                report->status = CALV3_ERR_NONFINITE;
                return report->status;
            }

            VectorXd deltaFull = VectorXd::Zero(4 * axis);
            for (int k = 0; k < K; ++k)
                deltaFull(activeIndex[static_cast<size_t>(k)]) = deltaJoint(k);

            bool accepted = false;
            double acceptedScale = 0.0;
            Model acceptedModel;
            State acceptedState;
            VectorXd acceptedDelta = VectorXd::Zero(4 * axis);
            for (double scale = 1.0; scale >= 1.0 / 1024.0; scale *= 0.5)
            {
                const VectorXd scaledDelta = scale * deltaFull;
                if (!InsidePhysicalEnvelope(accumulatedDelta + scaledDelta, activeIndex, &laserPrior))
                    continue;

                Model candidate = current;
                ApplyDelta(candidate, scaledDelta);
                State candidateState;
                status = EvaluateStateAtAnchor(candidate, sampleCount, jointRad, measured,
                                               fixtureLocalXYZ, fixedAnchor, fixedRopeBiasM,
                                               candidateState);
                if (status != CALV3_OK) continue;

                if (currentState.mae - candidateState.mae > minMaeImprovementM)
                {
                    accepted = true;
                    acceptedModel = candidate;
                    acceptedState = candidateState;
                    acceptedDelta = scaledDelta;
                    acceptedScale = scale;
                    break;
                }
            }

            if (!accepted)
            {
                he3StopStatus = CALV3_STOP_NO_IMPROVEMENT;
                break;
            }

            current = acceptedModel;
            currentState = acceptedState;
            accumulatedDelta += acceptedDelta;
            ++he3AcceptedIterations;
            AppendTrace(he3AcceptedIterations, acceptedScale, current, currentState, acceptedDelta);
        }

        {
            MatrixXd Jactive;
            LinearSolveInfo jInfo;
            if (BuildRobotOnlyJacobian(current, sampleCount, jointRad, fixtureLocalXYZ,
                                       currentState, activeIndex, Jactive, jInfo) == CALV3_OK)
            {
                he3FinalRank = jInfo.rank;
                he3FinalCondition = jInfo.condition;
            }
        }

        CopyModelToOutput(current, outA, outD, outAlpha, outQ0);
        for (int i = 0; i < 4 * axis; ++i) outDelta4N[i] = accumulatedDelta(i);
        for (int s = 0; s < sampleCount; ++s) residualAfterM[s] = currentState.residual(s);

        report->iterations = he3AcceptedIterations;
        report->jacobianRank = he3FinalRank;
        report->jacobianCondition = he3FinalCondition;
        report->anchorRank = currentState.anchorRank;
        report->maeAfterM = currentState.mae;
        report->maxAbsAfterM = currentState.maxAbs;
        report->ropeBiasM = fixedRopeBiasM;
        for (int i = 0; i < 3; ++i) report->anchorAfter[i] = fixedAnchor(i);
        report->status = he3StopStatus;
        return report->status;
    }

    int finalRank = 0;
    double finalCondition = std::numeric_limits<double>::infinity();
    int acceptedIterations = 0;
    int stopStatus = CALV3_OK;

    for (int iter = 0; iter < maxIterations; ++iter)
    {
        MatrixXd Jactive;
        LinearSolveInfo jInfo;
        status = BuildRopeJacobian(current, sampleCount, jointRad, measured, fixtureLocalXYZ,
                                   currentState, activeIndex, Jactive, jInfo);
        if (status != CALV3_OK)
        {
            report->status = status;
            return status;
        }

        finalRank = std::max(0, jInfo.rank - 4);
        finalCondition = jInfo.condition;

        const int requiredJointRank = static_cast<int>(activeIndex.size()) + 4;
        if (requireFullColumnRank != 0 && jInfo.rank < requiredJointRank)
        {
            report->jacobianRank = std::max(0, jInfo.rank - 4);
            report->jacobianCondition = jInfo.condition;
            report->anchorRank = currentState.anchorRank;
            report->maeAfterM = currentState.mae;
            report->maxAbsAfterM = currentState.maxAbs;
            report->ropeBiasM = currentState.ropeBiasM;
            for (int i = 0; i < 3; ++i) report->anchorAfter[i] = currentState.anchor(i);
            report->status = CALV3_ERR_JACOBIAN_RANK;
            CopyModelToOutput(current, outA, outD, outAlpha, outQ0);
            for (int i = 0; i < 4 * axis; ++i) outDelta4N[i] = accumulatedDelta(i);
            for (int s = 0; s < sampleCount; ++s) residualAfterM[s] = currentState.residual(s);
            return report->status;
        }

        if (maxCondition > 0.0 && (!IsFinite(jInfo.condition) || jInfo.condition > maxCondition))
        {
            report->jacobianRank = std::max(0, jInfo.rank - 4);
            report->jacobianCondition = jInfo.condition;
            report->anchorRank = currentState.anchorRank;
            report->maeAfterM = currentState.mae;
            report->maxAbsAfterM = currentState.maxAbs;
            report->ropeBiasM = currentState.ropeBiasM;
            for (int i = 0; i < 3; ++i) report->anchorAfter[i] = currentState.anchor(i);
            report->status = CALV3_ERR_JACOBIAN_RANK;
            CopyModelToOutput(current, outA, outD, outAlpha, outQ0);
            for (int i = 0; i < 4 * axis; ++i) outDelta4N[i] = accumulatedDelta(i);
            for (int s = 0; s < sampleCount; ++s) residualAfterM[s] = currentState.residual(s);
            return report->status;
        }

        // R6 keeps V1's exact analytic Jacobian direction, but adds two
        // production safeguards learned from eight laser-calibrated HE3s:
        // (1) Huber weighting prevents one bad pose from steering all MDH;
        // (2) weak physical priors stop an unobservable rope direction from
        //     becoming a fictitious 100 mm link or a 40 degree zero error.
        const int K = static_cast<int>(activeIndex.size());
        const double measurementSigmaM = 0.0015;
        const double huberM = 0.005;
        MatrixXd Jsolve = MatrixXd::Zero(sampleCount + K, K + 4);
        VectorXd rsolve = VectorXd::Zero(sampleCount + K);
        for (int s = 0; s < sampleCount; ++s)
        {
            const double absResidual = std::abs(currentState.residual(s));
            const double huberWeight = (absResidual <= huberM || absResidual <= kTinyLength)
                ? 1.0 : std::sqrt(huberM / absResidual);
            const double rowScale = huberWeight / measurementSigmaM;
            Jsolve.row(s) = rowScale * Jactive.row(s);
            rsolve(s) = rowScale * currentState.residual(s);
        }
        for (int k = 0; k < K; ++k)
        {
            const int fullIndex = activeIndex[static_cast<size_t>(k)];
            const double sigma = GenericPriorSigma(fullIndex, axis);
            Jsolve(sampleCount + k, k) = 1.0 / sigma;
            rsolve(sampleCount + k) = -accumulatedDelta(fullIndex) / sigma;
        }

        // V1 core solve: analytic linearization followed by pseudoinverse.
        LinearSolveInfo solveInfo;
        const VectorXd deltaJoint = PseudoInverseSolve(Jsolve, rsolve, solveInfo);
        if (!AllFinite(deltaJoint))
        {
            report->status = CALV3_ERR_NONFINITE;
            return report->status;
        }

        VectorXd deltaFull = VectorXd::Zero(4 * axis);
        for (int k = 0; k < static_cast<int>(activeIndex.size()); ++k)
            deltaFull(activeIndex[static_cast<size_t>(k)]) = deltaJoint(k);
        const Vector3d deltaAnchor = deltaJoint.segment(K, 3);
        const double deltaRopeBiasM = deltaJoint(K + 3);

        // Keep V1's analytic pseudoinverse direction.  A short scalar
        // backtracking search only prevents a full linearized step from
        // overshooting after the anchor is re-estimated; it does not use a
        // finite-difference parameter Jacobian or a different optimizer.
        bool accepted = false;
        double acceptedScale = 0.0;
        Model acceptedModel;
        State acceptedState;
        VectorXd acceptedDelta = VectorXd::Zero(4 * axis);
        for (double scale = 1.0; scale >= 1.0 / 1024.0; scale *= 0.5)
        {
            const VectorXd scaledDelta = scale * deltaFull;
            if (!InsidePhysicalEnvelope(accumulatedDelta + scaledDelta, activeIndex, nullptr))
                continue;
            Model candidate = current;
            ApplyDelta(candidate, scaledDelta);
            State candidateState;
            status = EvaluateStateAtAnchor(candidate, sampleCount, jointRad, measured,
                                           fixtureLocalXYZ,
                                           currentState.anchor + scale * deltaAnchor,
                                           currentState.ropeBiasM + scale * deltaRopeBiasM,
                                           candidateState);
            if (status != CALV3_OK)
                continue;

            const double improvement = currentState.mae - candidateState.mae;
            if (improvement > minMaeImprovementM)
            {
                accepted = true;
                acceptedModel = candidate;
                acceptedState = candidateState;
                acceptedDelta = scaledDelta;
                acceptedScale = scale;
                break;
            }
        }

        if (!accepted)
        {
            stopStatus = CALV3_STOP_NO_IMPROVEMENT;
            break;
        }

        current = acceptedModel;
        currentState = acceptedState;
        accumulatedDelta += acceptedDelta;
        ++acceptedIterations;
        AppendTrace(acceptedIterations, acceptedScale, current, currentState, acceptedDelta);
    }

    // Rebuild final J for final rank/condition report.
    {
        MatrixXd Jactive;
        LinearSolveInfo jInfo;
        if (BuildRopeJacobian(current, sampleCount, jointRad, measured, fixtureLocalXYZ,
                              currentState, activeIndex, Jactive, jInfo) == CALV3_OK)
        {
            finalRank = std::max(0, jInfo.rank - 4);
            finalCondition = jInfo.condition;
        }
    }

    CopyModelToOutput(current, outA, outD, outAlpha, outQ0);
    for (int i = 0; i < 4 * axis; ++i)
        outDelta4N[i] = accumulatedDelta(i);
    for (int s = 0; s < sampleCount; ++s)
        residualAfterM[s] = currentState.residual(s);

    report->iterations = acceptedIterations;
    report->jacobianRank = finalRank;
    report->jacobianCondition = finalCondition;
    report->anchorRank = currentState.anchorRank;
    report->maeAfterM = currentState.mae;
    report->maxAbsAfterM = currentState.maxAbs;
    report->ropeBiasM = currentState.ropeBiasM;
    for (int i = 0; i < 3; ++i)
        report->anchorAfter[i] = currentState.anchor(i);

    report->status = stopStatus;
    return report->status;
    }
    catch (...)
    {
        report->status = CALV3_ERR_INTERNAL;
        return report->status;
    }
}
