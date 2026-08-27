#include "MathematicalDLL.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

namespace
{
constexpr double kPi = 3.1415926535897932384626433832795;

bool LoadNumbers(const char* path, std::vector<double>& values)
{
    std::ifstream input(path);
    double value = 0.0;
    while (input >> value) values.push_back(value);
    return !values.empty();
}
}

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: V1BinaryReplay joint_deg.txt rope_raw_mm.txt\n";
        return 2;
    }

    constexpr int axis = 7;
    std::vector<double> jointDeg;
    std::vector<double> ropeMm;
    if (!LoadNumbers(argv[1], jointDeg) || !LoadNumbers(argv[2], ropeMm) ||
        jointDeg.size() % axis != 0)
    {
        std::cerr << "Invalid input data.\n";
        return 3;
    }
    const int sampleCount = static_cast<int>(jointDeg.size() / axis);
    if (static_cast<int>(ropeMm.size()) != sampleCount)
    {
        std::cerr << "Joint/rope count mismatch.\n";
        return 4;
    }

    MatrixXd dh(7, axis);
    dh.setZero();
    const double d[axis] = {0.0000006, 0.0505064, 0.4911552, -0.1186827,
                            0.3178316, -0.0007692, 0.1417026};
    const double alpha[axis] = {kPi/2, -kPi/2, kPi/2, -kPi/2,
                                kPi/2, -kPi/2, 0.0};
    for (int joint = 0; joint < axis; ++joint)
    {
        dh(1, joint) = d[joint];
        dh(2, joint) = alpha[joint];
    }

    MatrixXd theta(axis, sampleCount);
    for (int sample = 0; sample < sampleCount; ++sample)
        for (int joint = 0; joint < axis; ++joint)
            theta(joint, sample) = jointDeg[sample * axis + joint] * kPi / 180.0;

    VectorXd rope(sampleCount);
    for (int sample = 0; sample < sampleCount; ++sample)
        rope(sample) = ropeMm[sample] * 0.001 + 0.0625;

    double result[12] = {};
    double* oldError = nullptr;
    double* nowError = nullptr;
    Calibration(3, axis, dh, theta, rope, result, &oldError, &nowError);

    double oldMae = 0.0;
    double newMae = 0.0;
    for (int sample = 0; sample < sampleCount; ++sample)
    {
        oldMae += std::abs(oldError[sample]);
        newMae += std::abs(nowError[sample]);
    }
    oldMae = oldMae * 1000.0 / sampleCount;
    newMae = newMae * 1000.0 / sampleCount;

    std::cout << std::setprecision(12)
              << "V1 binary MAE before(mm)=" << oldMae
              << " after(mm)=" << newMae << "\nresult:";
    for (double value : result) std::cout << ' ' << value;
    std::cout << '\n';
    return 0;
}
