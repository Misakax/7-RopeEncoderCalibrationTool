#include "CalibrationV3Analytic.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

namespace
{
constexpr int kAxis = 7;
constexpr int kSamples = 96;
constexpr double kPi = 3.1415926535897932384626433832795;
}

int main()
{
    const double a[kAxis] = {0, 0, 0, 0, 0, 0, 0};
    const double dNominal[kAxis] = {0.0000006, 0.0505064, 0.4911552, -0.1186827,
                                     0.3178316, -0.0007692, 0.1417026};
    double dTrue[kAxis];
    for (int i = 0; i < kAxis; ++i) dTrue[i] = dNominal[i];
    // Known-truth regression: the synthetic robot has d5 longer by 0.8 mm.
    // V3 starts from V1 nominal data and must recover that injected error.
    constexpr double kInjectedD5M = 0.0008;
    dTrue[4] += kInjectedD5M;

    const double alpha[kAxis] = {kPi / 2, -kPi / 2, kPi / 2, -kPi / 2,
                                 kPi / 2, -kPi / 2, 0};
    const double zero[kAxis] = {};
    const double q0True[kAxis] = {};
    const double fixture[3] = {0, 0, 0};
    const double anchor[3] = {0.85, -0.42, 0.73};
    const double ropeOffset = 0.0625;

    std::vector<double> joints(kSamples * kAxis);
    for (int s = 0; s < kSamples; ++s)
    {
        const double t = 2.0 * kPi * s / kSamples;
        joints[s * kAxis + 0] = 0.8 * std::sin(t);
        joints[s * kAxis + 1] = 0.45 * std::sin(2.0 * t + 0.2);
        joints[s * kAxis + 2] = 0.65 * std::cos(1.5 * t - 0.1);
        joints[s * kAxis + 3] = -kPi / 2 + 0.40 * std::sin(2.5 * t + 0.4);
        joints[s * kAxis + 4] = 0.9 * std::cos(1.2 * t);
        joints[s * kAxis + 5] = 0.55 * std::sin(3.0 * t - 0.3);
        joints[s * kAxis + 6] = 1.4 * std::cos(0.7 * t + 0.5);
    }

    std::vector<double> ropeRaw(kSamples);
    std::vector<double> jacobian(3 * 4 * kAxis);
    for (int s = 0; s < kSamples; ++s)
    {
        double point[3] = {};
        const int status = CalibrationV3EvaluatePointAndJacobian(
            kAxis, a, dTrue, alpha, zero, zero, q0True,
            joints.data() + s * kAxis, fixture, point, jacobian.data());
        if (status != CALV3_OK) return 2;
        const double dx = point[0] - anchor[0];
        const double dy = point[1] - anchor[1];
        const double dz = point[2] - anchor[2];
        ropeRaw[s] = std::sqrt(dx * dx + dy * dy + dz * dz) - ropeOffset;
    }

    int active[4 * kAxis] = {};
    active[kAxis + 4] = 1;      // d5

    std::vector<double> outA(kAxis), outD(kAxis), outAlpha(kAxis), outQ0(kAxis);
    std::vector<double> delta(4 * kAxis), before(kSamples), after(kSamples);
    CalibrationV3Report report = {};
    const int status = CalibrationV3Analytic(
        kAxis, kSamples, a, dNominal, alpha, zero, zero, zero,
        joints.data(), ropeRaw.data(), ropeOffset, fixture, active,
        20, 1e-12, 1, 1e8,
        outA.data(), outD.data(), outAlpha.data(), outQ0.data(), delta.data(),
        before.data(), after.data(), &report);

    const double recoveredD5Mm = delta[kAxis + 4] * 1000.0;
    std::cout << std::setprecision(12)
              << "build=" << CalibrationV3BuildId() << " status=" << status << "\n"
              << "MAE(mm): " << report.maeBeforeM * 1000.0 << " -> "
              << report.maeAfterM * 1000.0 << "\n"
              << "d5 expected/recovered(mm): " << kInjectedD5M * 1000.0
              << " / " << recoveredD5Mm << "\n"
              << "rank=" << report.jacobianRank << "/" << report.activeCount
              << " condition=" << report.jacobianCondition << "\n";

    const bool successStatus = status == CALV3_OK || status == CALV3_STOP_NO_IMPROVEMENT;
    const bool passD5 = successStatus && report.maeAfterM < 1e-7 &&
        std::abs(recoveredD5Mm - kInjectedD5M * 1000.0) < 0.01;
    std::cout << "d5_test=" << (passD5 ? "PASS" : "FAIL") << "\n";

    // A rotational parameter that is visible to the rope model must also be
    // recoverable. q5 is used here to prove that R6.1 does not freeze all
    // rotational zero offsets.
    constexpr double kInjectedQ5Deg = 0.5;
    double q0TrueQ5[kAxis] = {};
    q0TrueQ5[4] = kInjectedQ5Deg * kPi / 180.0;
    std::vector<double> ropeRawQ5(kSamples);
    for (int s = 0; s < kSamples; ++s)
    {
        double point[3] = {};
        const int pointStatus = CalibrationV3EvaluatePointAndJacobian(
            kAxis, a, dNominal, alpha, zero, zero, q0TrueQ5,
            joints.data() + s * kAxis, fixture, point, jacobian.data());
        if (pointStatus != CALV3_OK) return 3;
        const double dx = point[0] - anchor[0];
        const double dy = point[1] - anchor[1];
        const double dz = point[2] - anchor[2];
        ropeRawQ5[s] = std::sqrt(dx * dx + dy * dy + dz * dz) - ropeOffset;
    }

    int activeQ5[4 * kAxis] = {};
    activeQ5[3 * kAxis + 4] = 1;
    std::vector<double> outAQ5(kAxis), outDQ5(kAxis), outAlphaQ5(kAxis), outQ0Q5(kAxis);
    std::vector<double> deltaQ5(4 * kAxis), beforeQ5(kSamples), afterQ5(kSamples);
    CalibrationV3Report reportQ5 = {};
    const int statusQ5 = CalibrationV3Analytic(
        kAxis, kSamples, a, dNominal, alpha, zero, zero, zero,
        joints.data(), ropeRawQ5.data(), ropeOffset, fixture, activeQ5,
        30, 1e-12, 1, 1e8,
        outAQ5.data(), outDQ5.data(), outAlphaQ5.data(), outQ0Q5.data(), deltaQ5.data(),
        beforeQ5.data(), afterQ5.data(), &reportQ5);
    const double recoveredQ5Deg = deltaQ5[3 * kAxis + 4] * 180.0 / kPi;
    const bool q5SuccessStatus = statusQ5 == CALV3_OK || statusQ5 == CALV3_STOP_NO_IMPROVEMENT;
    const bool passQ5 = q5SuccessStatus && reportQ5.maeAfterM < 1e-7 &&
        std::abs(recoveredQ5Deg - kInjectedQ5Deg) < 0.01;
    std::cout << "q5 status=" << statusQ5
              << " MAE(mm)=" << reportQ5.maeBeforeM * 1000.0 << " -> "
              << reportQ5.maeAfterM * 1000.0 << "\n"
              << "q5 expected/recovered(deg): " << kInjectedQ5Deg
              << " / " << recoveredQ5Deg << "\n"
              << "q5 rank=" << reportQ5.jacobianRank << "/" << reportQ5.activeCount
              << " condition=" << reportQ5.jacobianCondition << "\n"
              << "q5_test=" << (passQ5 ? "PASS" : "FAIL") << "\n";

    // R7.1 regression: inject only a common pull-wire absolute-length bias.
    // The solver must recover it as bL while leaving the robot d5 correction
    // essentially zero. This directly guards against hiding a tape/encoder zero
    // error inside MDH.
    constexpr double kInjectedRopeBiasM = 0.00275;
    std::vector<double> ropeRawBias(kSamples);
    for (int s = 0; s < kSamples; ++s)
    {
        double point[3] = {};
        const int pointStatus = CalibrationV3EvaluatePointAndJacobian(
            kAxis, a, dNominal, alpha, zero, zero, zero,
            joints.data() + s * kAxis, fixture, point, jacobian.data());
        if (pointStatus != CALV3_OK) return 5;
        const double dx = point[0] - anchor[0];
        const double dy = point[1] - anchor[1];
        const double dz = point[2] - anchor[2];
        ropeRawBias[s] = std::sqrt(dx * dx + dy * dy + dz * dz)
            + kInjectedRopeBiasM - ropeOffset;
    }

    int activeBias[4 * kAxis] = {};
    activeBias[kAxis + 4] = 1; // keep one robot column observable; truth correction is zero
    std::vector<double> outABias(kAxis), outDBias(kAxis), outAlphaBias(kAxis), outQ0Bias(kAxis);
    std::vector<double> deltaBias(4 * kAxis), beforeBias(kSamples), afterBias(kSamples);
    CalibrationV3Report reportBias = {};
    const int statusBias = CalibrationV3Analytic(
        kAxis, kSamples, a, dNominal, alpha, zero, zero, zero,
        joints.data(), ropeRawBias.data(), ropeOffset, fixture, activeBias,
        30, 1e-12, 1, 1e8,
        outABias.data(), outDBias.data(), outAlphaBias.data(), outQ0Bias.data(), deltaBias.data(),
        beforeBias.data(), afterBias.data(), &reportBias);
    const double recoveredBiasMm = reportBias.ropeBiasM * 1000.0;
    const double falseD5Mm = deltaBias[kAxis + 4] * 1000.0;
    const bool biasSuccessStatus = statusBias == CALV3_OK || statusBias == CALV3_STOP_NO_IMPROVEMENT;
    const bool passBias = biasSuccessStatus && reportBias.maeAfterM < 1e-7 &&
        std::abs(recoveredBiasMm - kInjectedRopeBiasM * 1000.0) < 0.01 &&
        std::abs(falseD5Mm) < 0.01;
    std::cout << "bL status=" << statusBias
              << " raw/geometric MAE(mm)=" << reportBias.maeBeforeM * 1000.0 << " -> "
              << reportBias.maeAfterM * 1000.0 << "\n"
              << "bL expected/recovered(mm): " << kInjectedRopeBiasM * 1000.0
              << " / " << recoveredBiasMm << "\n"
              << "d5 false correction(mm): " << falseD5Mm << "\n"
              << "bL_test=" << (passBias ? "PASS" : "FAIL") << "\n";

    // Exact analytic point-Jacobian evidence for the current MDH/fixture.
    // The measurement point lies on both terminal rotation axes in this model,
    // so q6 and q7 must be zero columns.  Forcing either one into least squares
    // would create rank deficiency, not additional calibration information.
    double q6ColumnNorm2 = 0.0;
    double q7ColumnNorm2 = 0.0;
    for (int s = 0; s < kSamples; ++s)
    {
        double point[3] = {};
        const int pointStatus = CalibrationV3EvaluatePointAndJacobian(
            kAxis, a, dNominal, alpha, zero, zero, zero,
            joints.data() + s * kAxis, fixture, point, jacobian.data());
        if (pointStatus != CALV3_OK) return 4;
        for (int row = 0; row < 3; ++row)
        {
            const double q6Value = jacobian[row * 4 * kAxis + 3 * kAxis + 5];
            const double q7Value = jacobian[row * 4 * kAxis + 3 * kAxis + 6];
            q6ColumnNorm2 += q6Value * q6Value;
            q7ColumnNorm2 += q7Value * q7Value;
        }
    }
    const double q6ColumnNorm = std::sqrt(q6ColumnNorm2);
    const double q7ColumnNorm = std::sqrt(q7ColumnNorm2);
    const bool passQ6StructuralZero = q6ColumnNorm < 1e-10;
    const bool passQ7StructuralZero = q7ColumnNorm < 1e-10;
    std::cout << "q6 point-Jacobian column norm=" << q6ColumnNorm
              << " structural_zero=" << (passQ6StructuralZero ? "PASS" : "FAIL") << "\n"
              << "q7 point-Jacobian column norm=" << q7ColumnNorm
              << " structural_zero=" << (passQ7StructuralZero ? "PASS" : "FAIL") << "\n";

    const bool pass = passD5 && passQ5 && passBias && passQ6StructuralZero && passQ7StructuralZero;
    std::cout << "overall=" << (pass ? "PASS" : "FAIL") << "\n";

    std::cout << "\nPress Enter to close...";
    std::cin.get();

    return pass ? 0 : 1;
}
