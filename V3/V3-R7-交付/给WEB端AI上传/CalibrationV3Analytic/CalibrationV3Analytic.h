#pragma once

// HE3 / general N-axis pull-wire calibration using V1-style exact analytic Jacobian.
// Public ABI uses only POD/scalar arrays. Eigen is used internally only.

#ifdef _WIN32
#  ifdef MATHEMATICALDLL_EXPORTS
#    define CALV3_API __declspec(dllexport)
#  else
#    define CALV3_API __declspec(dllimport)
#  endif
#  define CALV3_CALL __stdcall
#else
#  define CALV3_API
#  define CALV3_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum CalibrationV3Status
{
    CALV3_OK = 0,
    CALV3_STOP_NO_IMPROVEMENT = 1,

    CALV3_ERR_BAD_ARGUMENT = -1,
    CALV3_ERR_ANCHOR_RANK = -2,
    CALV3_ERR_JACOBIAN_RANK = -3,
    CALV3_ERR_NONFINITE = -4,
    CALV3_ERR_INVALID_LENGTH = -5,
    CALV3_ERR_CORRECTION_LIMIT = -6, // retained for ABI compatibility; not emitted by R2
    CALV3_ERR_INTERNAL = -7
};

// Build identity is exported so the GUI/log can prove which DLL was loaded.
CALV3_API const char* CALV3_CALL CalibrationV3BuildId();

// Parameter layout for activeMask4N / delta4N:
// [ a1..aN | d1..dN | alpha1..alphaN | q01..q0N ]
// Units: a,d = m; alpha,q0 = rad.
struct CalibrationV3Report
{
    int status;
    int iterations;
    int activeCount;
    int jacobianRank;
    int anchorRank;

    double jacobianCondition;
    double maeBeforeM;
    double maeAfterM;
    double maxAbsBeforeM;
    double maxAbsAfterM;

    double anchorBefore[3];
    double anchorAfter[3];
};

enum { CALV3_TRACE_MAX_AXIS = 8, CALV3_TRACE_MAX_ITERATIONS = 32 };

// R6.1 trace record. It is intentionally POD-only so the MFC GUI can read it
// without sharing Eigen/STL objects across the DLL boundary.
struct CalibrationV3IterationRecord
{
    int iteration;       // 0 = input model, 1..N = accepted updates
    double stepScale;
    double maeM;
    double maxAbsM;
    double anchor[3];
    double a[CALV3_TRACE_MAX_AXIS];
    double d[CALV3_TRACE_MAX_AXIS];
    double alpha[CALV3_TRACE_MAX_AXIS];
    double q0[CALV3_TRACE_MAX_AXIS];
    double appliedDelta4N[4 * CALV3_TRACE_MAX_AXIS];
};

struct CalibrationV3Trace
{
    int axis;
    int count;
    CalibrationV3IterationRecord record[CALV3_TRACE_MAX_ITERATIONS];
};

// Returns the trace produced by the most recent call on the current thread.
CALV3_API int CALV3_CALL CalibrationV3GetLastTrace(CalibrationV3Trace* outTrace);

// IMPORTANT INPUT CONVENTIONS
// 1) jointRad is sample-major: jointRad[sampleIndex * axis + jointIndex].
// 2) V1 is the authority for the HE3 production measurement convention:
//        d[6] = robot d7 + ToolOffset ~= 0.0927026 + 0.049 = 0.1417026 m
//        fixtureLocalXYZ = {0, 0, 0}
//    fixtureLocalXYZ exists only for algebra/fixture experiments.  Do not add
//    the 49 mm again when the V1-folded d7 value is supplied.
// 3) ropeRawM is the encoder-derived rope length before the fixed rope dead-length correction.
//    measured length used by V3 = ropeRawM + ropeFixedOffsetM.
//    The 49 mm fixture geometry is NOT added to ropeFixedOffsetM in this model.
// 4) fixtureLocalXYZ is a rigid point offset expressed in frame 7 after T7.
// 5) activeMask4N is 0/1. The solver uses V1's analytic-Jacobian and
//    pseudoinverse direction. The fixed rope anchor is initialized by V1's
//    adjacent-sphere equations, then solved jointly as three analytic nuisance
//    variables. No finite-difference Jacobian is used by the DLL.
// 6) maxCondition <= 0 disables the condition-number rejection.
// 7) requireFullColumnRank != 0 rejects rank(J_active) < activeCount.
// 8) V3 does not impose an artificial parameter-correction clamp.  V1 did not
//    have one either.  The caller decides acceptance from the returned residual,
//    rank, condition number and parameter corrections.

// Diagnostic API for development/gradient checking.
// outJacobian3x4N is row-major: row * (4*axis) + col.
CALV3_API int CALV3_CALL CalibrationV3EvaluatePointAndJacobian(
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
    double* outJacobian3x4N);

CALV3_API int CALV3_CALL CalibrationV3Analytic(
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
    CalibrationV3Report* report);

#ifdef __cplusplus
}
#endif
