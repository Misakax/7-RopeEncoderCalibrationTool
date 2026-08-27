#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "MathematicalDLL.h"

namespace
{
	const double kPi = 3.14159265358979323846;
	int failures = 0;

	void Check(bool condition, const std::string& message)
	{
		if (condition)
			std::cout << "[PASS] " << message << "\n";
		else
		{
			std::cerr << "[FAIL] " << message << "\n";
			++failures;
		}
	}

	std::vector<double> He3Mdh()
	{
		const int n = 7;
		std::vector<double> mdh(5 * n, 0.0);
		const double d[n] = { 0.0000006, 0.0505064, 0.4911552, -0.1186827, 0.3178316, -0.0007692, 0.0927026 };
		const double alpha[n] = { kPi / 2, -kPi / 2, kPi / 2, -kPi / 2, kPi / 2, -kPi / 2, 0.0 };
		for (int i = 0; i < n; ++i)
		{
			mdh[n + i] = d[i];
			mdh[2 * n + i] = alpha[i];
		}
		return mdh;
	}

	std::vector<double> ExcitingTrajectory(int samples)
	{
		const int n = 7;
		std::vector<double> theta(n * samples, 0.0);
		for (int sample = 0; sample < samples; ++sample)
		{
			const double t = 2.0 * kPi * sample / samples;
			theta[0 * samples + sample] = 0.55 * std::sin(t) + 0.14 * std::cos(3.0 * t);
			theta[1 * samples + sample] = 0.42 * std::sin(2.0 * t + 0.2);
			theta[2 * samples + sample] = 0.65 * std::cos(t - 0.3) + 0.12 * std::sin(4.0 * t);
			theta[3 * samples + sample] = -kPi / 2.0 + 0.45 * std::sin(3.0 * t + 0.5);
			theta[4 * samples + sample] = 0.60 * std::cos(2.0 * t + 0.9);
			theta[5 * samples + sample] = 0.50 * std::sin(4.0 * t - 0.4);
			theta[6 * samples + sample] = 0.75 * std::cos(3.0 * t + 0.1);
		}
		return theta;
	}

	Matrix4d JointTransform(double q, double a, double d, double alpha, double beta)
	{
		Matrix4d transform = Matrix4d::Zero();
		transform(0, 0) = std::cos(q) * std::cos(beta);
		transform(0, 1) = -std::sin(q);
		transform(0, 2) = std::cos(q) * std::sin(beta);
		transform(0, 3) = a;
		transform(1, 0) = std::sin(q) * std::cos(alpha) * std::cos(beta) + std::sin(alpha) * std::sin(beta);
		transform(1, 1) = std::cos(q) * std::cos(alpha);
		transform(1, 2) = std::sin(q) * std::cos(alpha) * std::sin(beta) - std::sin(alpha) * std::cos(beta);
		transform(1, 3) = -d * std::sin(alpha);
		transform(2, 0) = std::sin(q) * std::sin(alpha) * std::cos(beta) - std::cos(alpha) * std::sin(beta);
		transform(2, 1) = std::cos(q) * std::sin(alpha);
		transform(2, 2) = std::sin(q) * std::sin(alpha) * std::sin(beta) + std::cos(alpha) * std::cos(beta);
		transform(2, 3) = d * std::cos(alpha);
		transform(3, 3) = 1.0;
		return transform;
	}

	std::vector<double> GenerateLengths(const std::vector<double>& mdh, const std::vector<double>& theta,
		const std::vector<double>& q0, int samples, const Vector3d& anchor, double toolZ)
	{
		const int n = static_cast<int>(q0.size());
		std::vector<double> lengths(samples, 0.0);
		for (int sample = 0; sample < samples; ++sample)
		{
			Matrix4d transform = Matrix4d::Identity();
			for (int axis = 0; axis < n; ++axis)
			{
				const double q = theta[axis * samples + sample] + q0[axis] + mdh[3 * n + axis];
				transform *= JointTransform(q, mdh[axis], mdh[n + axis], mdh[2 * n + axis], mdh[4 * n + axis]);
			}
			const Vector3d position = (transform * Vector4d(0.0, 0.0, toolZ, 1.0)).head<3>();
			lengths[sample] = (position - anchor).norm();
		}
		return lengths;
	}

	CalibrationV2Options Options(bool validationSplit)
	{
		CalibrationV2Options options = {};
		options.structSize = sizeof(options);
		options.maxIterations = 50;
		options.toolOffset[2] = 0.049;
		options.rankTolerance = 1.0e-8;
		options.maxConditionNumber = 1.0e4;
		options.huberDelta = 0.002;
		options.initialDamping = 1.0e-3;
		options.lengthParameterScale = 0.001;
		options.angleParameterScale = 0.001;
		options.maxLengthCorrection = 0.005;
		options.maxAngleCorrection = kPi / 180.0;
		options.useValidationSplit = validationSplit ? 1 : 0;
		return options;
	}

	std::vector<int> He3Mask()
	{
		const int n = 7;
		std::vector<int> mask(4 * n, 0);
		const int dAxes[] = { 1, 2, 3, 4, 6 };
		const int qAxes[] = { 1, 2, 3, 4 };
		for (int axis : dAxes) mask[n + axis] = 1;
		for (int axis : qAxes) mask[3 * n + axis] = 1;
		return mask;
	}

	struct RunOutput
	{
		int status;
		CalibrationV2Result result;
		std::vector<double> updated;
		std::vector<double> delta;
		std::vector<int> state;
		std::vector<double> q0;
		std::vector<double> before;
		std::vector<double> after;
	};

	RunOutput Run(const std::vector<double>& mdh, const std::vector<double>& theta,
		const std::vector<double>& lengths, const std::vector<int>& mask, CalibrationV2Options options)
	{
		const int n = static_cast<int>(mdh.size() / 5);
		const int samples = static_cast<int>(lengths.size());
		RunOutput output = {};
		output.updated.resize(5 * n);
		output.delta.resize(4 * n);
		output.state.resize(4 * n);
		output.q0.resize(n);
		output.before.resize(samples);
		output.after.resize(samples);
		output.result.structSize = sizeof(output.result);
		output.status = CalibrationV2(3, n, samples, mdh.data(), theta.data(), lengths.data(), mask.data(), &options,
			output.updated.data(), output.delta.data(), output.state.data(), output.q0.data(),
			output.before.data(), output.after.data(), &output.result);
		return output;
	}

	void SyntheticTests()
	{
		const int n = 7;
		const int samples = 80;
		const std::vector<double> nominal = He3Mdh();
		const std::vector<double> theta = ExcitingTrajectory(samples);
		const Vector3d anchor(-1.4, -1.1, -0.03);
		const std::vector<double> noZero(n, 0.0);
		const std::vector<int> mask = He3Mask();
		const std::vector<double> exactLengths = GenerateLengths(nominal, theta, noZero, samples, anchor, 0.049);
		RunOutput exact = Run(nominal, theta, exactLengths, mask, Options(false));
		Check(exact.status == CALIBRATION_V2_OK, "zero-error synthetic data converges");
		Check(exact.result.beforeMae < 1.0e-8 && exact.result.afterMae < 1.0e-8,
			"zero-error synthetic residual remains near zero");

		std::vector<double> truth = nominal;
		truth[n + 1] += 0.0005;
		truth[n + 2] -= 0.0007;
		truth[n + 3] += 0.0004;
		truth[n + 4] -= 0.0003;
		truth[n + 6] += 0.0020;
		std::vector<double> trueQ0(n, 0.0);
		trueQ0[1] = 0.10 * kPi / 180.0;
		trueQ0[2] = -0.08 * kPi / 180.0;
		trueQ0[3] = 0.05 * kPi / 180.0;
		trueQ0[4] = -0.06 * kPi / 180.0;
		const std::vector<double> shiftedLengths = GenerateLengths(truth, theta, trueQ0, samples, anchor, 0.049);
		RunOutput shifted = Run(nominal, theta, shiftedLengths, mask, Options(false));
		Check(shifted.status == CALIBRATION_V2_OK, "injected HE3 parameters converge");
		Check(shifted.result.afterMae < 1.0e-5, "injected HE3 residual is recovered below 0.01 mm");
		Check(std::abs(shifted.delta[n + 6] - 0.0020) < 2.0e-4, "d7 +2 mm is recovered");
		Check(std::abs(shifted.q0[1] - trueQ0[1]) < 2.0e-4, "q2 zero offset is recovered");
		RunOutput shiftedValidation = Run(nominal, theta, shiftedLengths, mask, Options(true));
		Check(shiftedValidation.status == CALIBRATION_V2_OK,
			"HE3 validation-split workflow accepts consistent seven-axis data");
		Check(shiftedValidation.result.afterMae < 0.001
			&& shiftedValidation.result.validationAfterMae < 0.001,
			"HE3 training and held-out rope MAE both pass the 1 mm gate");

		std::vector<int> wristMask(4 * n, 0);
		wristMask[3 * n + 5] = wristMask[3 * n + 6] = 1;
		RunOutput wrist = Run(nominal, theta, exactLengths, wristMask, Options(false));
		Check(wrist.status == CALIBRATION_V2_NO_OBSERVABLE_PARAMETERS, "q6/q7-only request is rejected as unobservable");
		Check(wrist.state[3 * n + 5] == CALIBRATION_PARAMETER_UNOBSERVABLE
			&& wrist.state[3 * n + 6] == CALIBRATION_PARAMETER_UNOBSERVABLE, "q6 and q7 states explain the rejection");

		std::vector<int> duplicateLengthMask(4 * n, 0);
		duplicateLengthMask[n + 5] = duplicateLengthMask[n + 6] = 1;
		RunOutput duplicate = Run(nominal, theta, exactLengths, duplicateLengthMask, Options(false));
		const int activePair = (duplicate.state[n + 5] == CALIBRATION_PARAMETER_ACTIVE ? 1 : 0)
			+ (duplicate.state[n + 6] == CALIBRATION_PARAMETER_ACTIVE ? 1 : 0);
		Check(activePair == 1, "d6/d7 duplicate sensitivity keeps exactly one active parameter");

		std::vector<double> invalidLengths(5, 1.0);
		std::vector<double> invalidTheta(n * 5, 0.0);
		RunOutput insufficient = Run(nominal, invalidTheta, invalidLengths, mask, Options(false));
		Check(insufficient.status == CALIBRATION_V2_INSUFFICIENT_SAMPLES, "insufficient samples return an error code");
	}

	bool ReadNumbers(const std::string& path, std::vector<double>& values)
	{
		std::ifstream input(path.c_str());
		if (!input) return false;
		double value = 0.0;
		while (input >> value) values.push_back(value);
		return true;
	}

	void RealReplayTest()
	{
		const int n = 7;
		const int samples = 60;
		std::vector<double> jointRows;
		std::vector<double> rawMillimetres;
		const std::string jointPath = "RopeEncoderCalibration/Joint/HE3_GY_Joint_2026_6_9_16_48.txt";
		const std::string dataPath = "RopeEncoderCalibration/data/HE3_GY_Data_2026_6_9_16_48.txt";
		Check(ReadNumbers(jointPath, jointRows), "real HE3_GY joint file is readable");
		Check(ReadNumbers(dataPath, rawMillimetres), "real HE3_GY rope file is readable");
		if (jointRows.size() < n * samples || rawMillimetres.size() < samples) return;
		std::vector<double> theta(n * samples, 0.0);
		for (int sample = 0; sample < samples; ++sample)
			for (int axis = 0; axis < n; ++axis)
				theta[axis * samples + sample] = jointRows[sample * n + axis] * kPi / 180.0;
		std::vector<double> lengths(samples, 0.0);
		for (int i = 0; i < samples; ++i) lengths[i] = rawMillimetres[i] / 1000.0;
		CalibrationV2Options baselineOptions = Options(false);
		baselineOptions.ropeLengthOffset = 0.0625;
		RunOutput baseline = Run(He3Mdh(), theta, lengths, He3Mask(), baselineOptions);
		Check(std::abs(baseline.result.beforeMae * 1000.0 - 16.7541) < 0.2,
			"real HE3_GY refined-anchor baseline reproduces approximately 16.75 mm MAE");

		CalibrationV2Options options = Options(true);
		options.ropeLengthOffset = 0.0625;
		RunOutput replay = Run(He3Mdh(), theta, lengths, He3Mask(), options);
		std::cout << "[INFO] real replay status=" << replay.status << " MAE(mm)="
			<< replay.result.beforeMae * 1000.0 << " -> " << replay.result.afterMae * 1000.0
			<< " validation(mm)=" << replay.result.validationBeforeMae * 1000.0 << " -> "
			<< replay.result.validationAfterMae * 1000.0 << "\n";
		Check(std::isfinite(replay.result.afterMae) && replay.result.afterMae <= replay.result.beforeMae,
			"real replay is finite and does not worsen all-point MAE");
		Check(replay.result.writeBackAllowed == 0, "real replay never enables write-back");
		Check(replay.status == CALIBRATION_V2_CORRECTION_OUT_OF_RANGE,
			"legacy mismatched HE3_GY replay is rejected by correction safety bounds");
		Check(replay.result.afterMae > 0.001 || replay.result.validationAfterMae > 0.001,
			"legacy mismatched replay cannot be mistaken for a sub-millimetre accepted run");
	}
}

int main()
{
	std::cout << "Build: " << CalibrationBuildId() << "\n";
	SyntheticTests();
	RealReplayTest();
	std::cout << (failures == 0 ? "ALL TESTS PASSED\n" : "TEST FAILURES: " + std::to_string(failures) + "\n");
	return failures == 0 ? 0 : 1;
}
