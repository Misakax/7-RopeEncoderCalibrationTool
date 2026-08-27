#include "CalibrationV3Analytic.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace
{
constexpr int kAxis = 7;
constexpr double kPi = 3.1415926535897932384626433832795;

struct Candidate
{
    const char* name;
    double d7;
    double fixtureZ;
    double ropeOffset;
};

bool LoadNumbers(const std::string& path, std::vector<double>& out)
{
    std::ifstream input(path);
    if (!input) return false;
    double value = 0.0;
    while (input >> value) out.push_back(value);
    return !out.empty();
}

double DegToRad(double degree) { return degree * kPi / 180.0; }

void PrintJointCoverage(const std::vector<double>& qDeg, int sampleCount)
{
    std::cout << "joint coverage(deg):\n";
    for (int joint = 0; joint < kAxis; ++joint)
    {
        double minValue = std::numeric_limits<double>::infinity();
        double maxValue = -std::numeric_limits<double>::infinity();
        double sum = 0.0;
        for (int sample = 0; sample < sampleCount; ++sample)
        {
            const double value = qDeg[sample * kAxis + joint];
            minValue = std::min(minValue, value);
            maxValue = std::max(maxValue, value);
            sum += value;
        }
        const double mean = sum / sampleCount;
        double variance = 0.0;
        for (int sample = 0; sample < sampleCount; ++sample)
        {
            const double diff = qDeg[sample * kAxis + joint] - mean;
            variance += diff * diff;
        }
        variance /= sampleCount;
        std::cout << "  J" << (joint + 1)
                  << " min=" << minValue << " max=" << maxValue
                  << " span=" << (maxValue - minValue)
                  << " std=" << std::sqrt(variance) << "\n";
    }
}

int EvaluateCandidate(const Candidate& candidate,
                      const std::vector<double>& jointRad,
                      const std::vector<double>& ropeRawM,
                      int sampleCount)
{
    const double a[kAxis] = {0, 0, 0, 0, 0, 0, 0};
    double d[kAxis] = {0.0000006, 0.0505064, 0.4911552, -0.1186827,
                       0.3178316, -0.0007692, candidate.d7};
    const double alpha[kAxis] = {kPi / 2, -kPi / 2, kPi / 2, -kPi / 2,
                                 kPi / 2, -kPi / 2, 0};
    const double zero[kAxis] = {0, 0, 0, 0, 0, 0, 0};
    const double fixture[3] = {0, 0, candidate.fixtureZ};

    // V1-authoritative seven-axis extension: all non-zero d columns and J1-J6
    // zero offsets.  J7 q0 is deliberately excluded because an axial/free-
    // rotating rope point gives an exactly zero q7 column.
    int active[4 * kAxis] = {};
    for (int joint = 0; joint < kAxis; ++joint) active[kAxis + joint] = 1;
    for (int joint = 0; joint < kAxis - 1; ++joint) active[3 * kAxis + joint] = 1;

    std::vector<double> outA(kAxis), outD(kAxis), outAlpha(kAxis), outQ0(kAxis);
    std::vector<double> delta(4 * kAxis), before(sampleCount), after(sampleCount);
    CalibrationV3Report report = {};
    const int status = CalibrationV3Analytic(
        kAxis, sampleCount, a, d, alpha, zero, zero, zero,
        jointRad.data(), ropeRawM.data(), candidate.ropeOffset, fixture, active,
        0, 0.0, 0, 0.0,
        outA.data(), outD.data(), outAlpha.data(), outQ0.data(), delta.data(),
        before.data(), after.data(), &report);

    std::cout << std::left << std::setw(29) << candidate.name
              << " status=" << std::setw(3) << status
              << " MAE(mm)=" << std::setw(11) << report.maeBeforeM * 1000.0
              << " MAX(mm)=" << std::setw(11) << report.maxAbsBeforeM * 1000.0
              << " rank=" << report.jacobianRank << "/" << report.activeCount
              << " cond=" << report.jacobianCondition
              << " anchor=[" << report.anchorBefore[0] << ","
              << report.anchorBefore[1] << "," << report.anchorBefore[2] << "]\n";
    return status;
}

void PrintV1ObservabilityDetails(const std::vector<double>& jointRad,
                                 const std::vector<double>& ropeRawM,
                                 int sampleCount)
{
    const double a[kAxis] = {0, 0, 0, 0, 0, 0, 0};
    const double d[kAxis] = {0.0000006, 0.0505064, 0.4911552, -0.1186827,
                             0.3178316, -0.0007692, 0.1417026};
    const double alpha[kAxis] = {kPi / 2, -kPi / 2, kPi / 2, -kPi / 2,
                                 kPi / 2, -kPi / 2, 0};
    const double zero[kAxis] = {};
    const double fixture[3] = {};
    int activeMask[4 * kAxis] = {};
    std::vector<int> activeIndex;
    std::vector<std::string> names;
    for (int joint = 0; joint < kAxis; ++joint)
    {
        activeMask[kAxis + joint] = 1;
        activeIndex.push_back(kAxis + joint);
        names.push_back("d" + std::to_string(joint + 1));
    }
    for (int joint = 0; joint < kAxis - 1; ++joint)
    {
        activeMask[3 * kAxis + joint] = 1;
        activeIndex.push_back(3 * kAxis + joint);
        names.push_back("q0" + std::to_string(joint + 1));
    }

    std::vector<double> outA(kAxis), outD(kAxis), outAlpha(kAxis), outQ0(kAxis);
    std::vector<double> delta(4 * kAxis), before(sampleCount), after(sampleCount);
    CalibrationV3Report report = {};
    const int status = CalibrationV3Analytic(
        kAxis, sampleCount, a, d, alpha, zero, zero, zero,
        jointRad.data(), ropeRawM.data(), 0.0625, fixture, activeMask,
        0, 0.0, 0, 0.0,
        outA.data(), outD.data(), outAlpha.data(), outQ0.data(), delta.data(),
        before.data(), after.data(), &report);
    if (status != CALV3_OK) return;

    names.push_back("anchorX");
    names.push_back("anchorY");
    names.push_back("anchorZ");
    const int robotColumnCount = static_cast<int>(activeIndex.size());
    Eigen::MatrixXd ropeJ(sampleCount, robotColumnCount + 3);
    std::vector<double> pointJ(3 * 4 * kAxis);
    for (int sample = 0; sample < sampleCount; ++sample)
    {
        double point[3] = {};
        CalibrationV3EvaluatePointAndJacobian(
            kAxis, a, d, alpha, zero, zero, zero,
            jointRad.data() + sample * kAxis, fixture, point, pointJ.data());
        Eigen::Vector3d ropeVector(point[0] - report.anchorBefore[0],
                                   point[1] - report.anchorBefore[1],
                                   point[2] - report.anchorBefore[2]);
        const Eigen::Vector3d direction = ropeVector.normalized();
        for (size_t column = 0; column < activeIndex.size(); ++column)
        {
            const int sourceColumn = activeIndex[column];
            double value = 0.0;
            for (int row = 0; row < 3; ++row)
                value += direction(row) * pointJ[row * (4 * kAxis) + sourceColumn];
            ropeJ(sample, static_cast<int>(column)) = value;
        }
        ropeJ.block<1, 3>(sample, robotColumnCount) = -direction.transpose();
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(ropeJ, Eigen::ComputeThinV);
    std::cout << "\nV3 joint robot+anchor singular values:\n  ";
    for (int i = 0; i < svd.singularValues().size(); ++i)
        std::cout << svd.singularValues()(i) << (i + 1 == svd.singularValues().size() ? '\n' : ' ');

    const double tolerance = 1e-10 * std::max(ropeJ.rows(), ropeJ.cols()) *
                             svd.singularValues()(0);
    int fullRank = 0;
    for (int i = 0; i < svd.singularValues().size(); ++i)
        if (svd.singularValues()(i) > tolerance) ++fullRank;
    const Eigen::MatrixXd V = svd.matrixV();
    const int nullCount = ropeJ.cols() - fullRank;
    for (int mode = 0; mode < nullCount; ++mode)
    {
        const int column = static_cast<int>(activeIndex.size()) - 1 - mode;
        std::cout << "  null mode " << (mode + 1) << ":";
        for (int row = 0; row < V.rows(); ++row)
            if (std::abs(V(row, column)) >= 0.15)
                std::cout << " " << names[static_cast<size_t>(row)]
                          << "=" << V(row, column);
        std::cout << "\n";
    }
}
}

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: V3MeasurementAudit joint_deg.txt rope_raw_mm.txt\n";
        return 2;
    }

    std::vector<double> qDeg;
    std::vector<double> ropeMm;
    if (!LoadNumbers(argv[1], qDeg) || !LoadNumbers(argv[2], ropeMm) ||
        qDeg.size() % kAxis != 0)
    {
        std::cerr << "Invalid input file.\n";
        return 3;
    }
    const int sampleCount = static_cast<int>(qDeg.size() / kAxis);
    if (static_cast<int>(ropeMm.size()) != sampleCount)
    {
        std::cerr << "Joint/rope sample count mismatch.\n";
        return 4;
    }

    std::vector<double> jointRad(qDeg.size());
    std::vector<double> ropeRawM(ropeMm.size());
    for (size_t i = 0; i < qDeg.size(); ++i) jointRad[i] = DegToRad(qDeg[i]);
    for (size_t i = 0; i < ropeMm.size(); ++i) ropeRawM[i] = ropeMm[i] / 1000.0;

    std::cout << std::setprecision(10);
    std::cout << "build=" << CalibrationV3BuildId() << " samples=" << sampleCount << "\n";
    PrintJointCoverage(qDeg, sampleCount);
    std::cout << "\nmeasurement-model comparison (zero optimization iterations):\n";

    const Candidate candidates[] = {
        {"V1 d7=141.7026 + L62.5", 0.1417026, 0.0,       0.0625},
        {"explicit +49Z + L62.5",  0.0927026, 0.049,     0.0625},
        {"center d7 + L62.5",      0.0927026, 0.0,       0.0625},
        {"center d7 + L111.5",     0.0927026, 0.0,       0.1115},
        {"V1 d7=141.7026 + L0",    0.1417026, 0.0,       0.0}
    };

    for (const Candidate& candidate : candidates)
        EvaluateCandidate(candidate, jointRad, ropeRawM, sampleCount);

    PrintV1ObservabilityDetails(jointRad, ropeRawM, sampleCount);

    // Algebra check: for alpha7=0, d7=92.7026 + local Z=49 mm must be
    // numerically identical to d7=141.7026 + local Z=0.  If not, the FK or
    // fixture convention has changed.
    const double a[kAxis] = {0, 0, 0, 0, 0, 0, 0};
    double dA[kAxis] = {0.0000006, 0.0505064, 0.4911552, -0.1186827,
                        0.3178316, -0.0007692, 0.1417026};
    double dB[kAxis] = {0.0000006, 0.0505064, 0.4911552, -0.1186827,
                        0.3178316, -0.0007692, 0.0927026};
    const double alpha[kAxis] = {kPi / 2, -kPi / 2, kPi / 2, -kPi / 2,
                                 kPi / 2, -kPi / 2, 0};
    const double zero[kAxis] = {};
    const double fixtureA[3] = {0, 0, 0};
    const double fixtureB[3] = {0, 0, 0.049};
    double pointA[3] = {}, pointB[3] = {};
    std::vector<double> jacobian(3 * 4 * kAxis);
    CalibrationV3EvaluatePointAndJacobian(kAxis, a, dA, alpha, zero, zero, zero,
        jointRad.data(), fixtureA, pointA, jacobian.data());
    CalibrationV3EvaluatePointAndJacobian(kAxis, a, dB, alpha, zero, zero, zero,
        jointRad.data(), fixtureB, pointB, jacobian.data());
    const double pointDiffMm = 1000.0 * std::sqrt(
        (pointA[0] - pointB[0]) * (pointA[0] - pointB[0]) +
        (pointA[1] - pointB[1]) * (pointA[1] - pointB[1]) +
        (pointA[2] - pointB[2]) * (pointA[2] - pointB[2]));
    std::cout << "\nV1 folded d7 vs explicit +49Z point difference(mm)=" << pointDiffMm << "\n";

    return 0;
}
