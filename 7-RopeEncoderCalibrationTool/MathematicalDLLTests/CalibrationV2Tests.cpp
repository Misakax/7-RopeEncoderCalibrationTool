#include <cmath>
#include <cstdlib>
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
	double DiagnosticMae(const std::vector<Vector3d>& positions,
		const std::vector<double>& rawLengths, double scale, double offset, Vector3d* finalAnchor);

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

	std::vector<double> He3RoundedRunMdh()
	{
		const int n = 7;
		std::vector<double> mdh(5 * n, 0.0);
		const double d[n] = { 0.0, 0.049, 0.493, -0.120, 0.317, 0.0, 0.093 };
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

	std::vector<Vector3d> DiagnosticPositions(const std::vector<double>& mdh,
		const std::vector<double>& theta, int samples, double toolZ)
	{
		const int n = static_cast<int>(mdh.size() / 5);
		std::vector<Vector3d> positions(samples);
		for (int sample = 0; sample < samples; ++sample)
		{
			Matrix4d transform = Matrix4d::Identity();
			for (int axis = 0; axis < n; ++axis)
			{
				const double q = theta[axis * samples + sample] + mdh[3 * n + axis];
				transform *= JointTransform(q, mdh[axis], mdh[n + axis], mdh[2 * n + axis], mdh[4 * n + axis]);
			}
			positions[sample] = (transform * Vector4d(0.0, 0.0, toolZ, 1.0)).head<3>();
		}
		return positions;
	}

	std::vector<Matrix4d> DiagnosticTransforms(const std::vector<double>& mdh,
		const std::vector<double>& theta, int samples)
	{
		const int n = static_cast<int>(mdh.size() / 5);
		std::vector<Matrix4d> transforms(samples, Matrix4d::Identity());
		for (int sample = 0; sample < samples; ++sample)
			for (int axis = 0; axis < n; ++axis)
			{
				const double q = theta[axis * samples + sample] + mdh[3 * n + axis];
				transforms[sample] *= JointTransform(q, mdh[axis], mdh[n + axis], mdh[2 * n + axis], mdh[4 * n + axis]);
			}
		return transforms;
	}

	double FitToolPoint(const std::vector<Matrix4d>& transforms,
		const std::vector<double>& rawLengths, bool fitScaleOffset,
		Vector3d& tool, Vector3d& anchor, double& scale, double& offset)
	{
		const int samples = static_cast<int>(transforms.size());
		std::vector<Vector3d> positions(samples);
		for (int i = 0; i < samples; ++i)
			positions[i] = (transforms[i] * Vector4d(tool.x(), tool.y(), tool.z(), 1.0)).head<3>();
		DiagnosticMae(positions, rawLengths, scale, offset, &anchor);
		double bestCost = std::numeric_limits<double>::infinity();
		double damping = 1.0e-5;
		for (int iteration = 0; iteration < 150; ++iteration)
		{
			const int columns = fitScaleOffset ? 8 : 6;
			MatrixXd J(samples, columns);
			VectorXd residual(samples);
			double cost = 0.0;
			for (int i = 0; i < samples; ++i)
			{
				const Matrix3d rotation = transforms[i].block<3, 3>(0, 0);
				const Vector3d position = (transforms[i] * Vector4d(tool.x(), tool.y(), tool.z(), 1.0)).head<3>();
				const Vector3d difference = position - anchor;
				const double distance = difference.norm();
				const Vector3d direction = difference / distance;
				residual(i) = rawLengths[i] * scale + offset - distance;
				cost += residual(i) * residual(i);
				J.block<1, 3>(i, 0) = direction.transpose();
				J.block<1, 3>(i, 3) = -(direction.transpose() * rotation);
				if (fitScaleOffset)
				{
					J(i, 6) = rawLengths[i];
					J(i, 7) = 1.0;
				}
			}
			if (cost < bestCost) bestCost = cost;
			MatrixXd normal = J.transpose() * J;
			normal.diagonal().array() += damping;
			const VectorXd step = normal.ldlt().solve(-J.transpose() * residual);
			if (!step.allFinite() || step.norm() < 1.0e-12) break;
			const Vector3d candidateAnchor = anchor + step.segment<3>(0);
			const Vector3d candidateTool = tool + step.segment<3>(3);
			const double candidateScale = fitScaleOffset ? scale + step(6) : scale;
			const double candidateOffset = fitScaleOffset ? offset + step(7) : offset;
			if (candidateTool.norm() > 0.5 || std::abs(candidateScale) < 0.5 || std::abs(candidateScale) > 1.5
				|| candidateOffset < -1.0 || candidateOffset > 3.0)
			{
				damping *= 10.0;
				continue;
			}
			double candidateCost = 0.0;
			for (int i = 0; i < samples; ++i)
			{
				const Vector3d position = (transforms[i] * Vector4d(candidateTool.x(), candidateTool.y(), candidateTool.z(), 1.0)).head<3>();
				const double r = rawLengths[i] * candidateScale + candidateOffset - (position - candidateAnchor).norm();
				candidateCost += r * r;
			}
			if (candidateCost < bestCost)
			{
				anchor = candidateAnchor;
				tool = candidateTool;
				scale = candidateScale;
				offset = candidateOffset;
				bestCost = candidateCost;
				damping = (std::max)(1.0e-12, damping / 3.0);
			}
			else damping *= 10.0;
		}
		double mae = 0.0;
		for (int i = 0; i < samples; ++i)
		{
			const Vector3d position = (transforms[i] * Vector4d(tool.x(), tool.y(), tool.z(), 1.0)).head<3>();
			mae += std::abs(rawLengths[i] * scale + offset - (position - anchor).norm());
		}
		return mae / samples;
	}

	// Rejected diagnostic hypothesis (kept only as a record, never used by the
	// production DLL or standard tests).  The operator later confirmed that the
	// rope attachment remains at a constant radius from the J7 output centre, so
	// the white-pin angle is not an independent calibration variable.
	// Earlier passive U-bracket model seen on the HE3 fixture:
	//   - pivotLocal is the white hinge pin, expressed in the J7/flange frame;
	//   - axisLocal is the hinge axis, fixed in the flange frame;
	//   - the red rope point moves on a circle of radius armRadius;
	//   - rope tension makes the arm align with the projection of the anchor
	//     direction onto the hinge plane.
	// This eliminates the unmeasured passive hinge angle analytically.
	double PassiveSwivelDistance(const Matrix4d& flange, const Vector3d& anchor,
		const Vector3d& pivotLocal, const Vector3d& axisLocal, double armRadius)
	{
		const Matrix3d rotation = flange.block<3, 3>(0, 0);
		const Vector3d pivotWorld = (flange * Vector4d(
			pivotLocal.x(), pivotLocal.y(), pivotLocal.z(), 1.0)).head<3>();
		Vector3d axisWorld = rotation * axisLocal;
		if (axisWorld.norm() <= 1.0e-12)
			return std::numeric_limits<double>::quiet_NaN();
		axisWorld.normalize();
		const Vector3d toAnchor = anchor - pivotWorld;
		const double axial = toAnchor.dot(axisWorld);
		const Vector3d radialVector = toAnchor - axial * axisWorld;
		const double radial = radialVector.norm();
		if (radial <= 1.0e-12)
			return std::abs(axial);
		const Vector3d ropePoint = pivotWorld + armRadius * radialVector / radial;
		return (anchor - ropePoint).norm();
	}

	struct PassiveSwivelFit
	{
		double trainMae;
		double validationMae;
		double allMae;
		Vector3d anchor;
		Vector3d pivotLocal;
		Vector3d axisLocal;
		double armRadius;
	};

	double PassiveCost(const std::vector<Matrix4d>& transforms,
		const std::vector<double>& lengths, const std::vector<int>& rows,
		const VectorXd& parameters, const Vector3d& axisLocal)
	{
		const Vector3d anchor = parameters.segment<3>(0);
		const Vector3d pivot = parameters.segment<3>(3);
		const double radius = parameters(6);
		double cost = 0.0;
		for (size_t i = 0; i < rows.size(); ++i)
		{
			const int sample = rows[i];
			const double predicted = PassiveSwivelDistance(
				transforms[sample], anchor, pivot, axisLocal, radius);
			const double residual = lengths[sample] - predicted;
			// Huber loss at 3 mm keeps one sticky/hysteretic bracket pose from
			// steering all fixture parameters.
			const double absolute = std::abs(residual);
			const double delta = 0.003;
			cost += absolute <= delta ? 0.5 * residual * residual
				: delta * (absolute - 0.5 * delta);
		}
		return cost;
	}

	double PassiveMae(const std::vector<Matrix4d>& transforms,
		const std::vector<double>& lengths, const std::vector<int>& rows,
		const VectorXd& parameters, const Vector3d& axisLocal)
	{
		double mae = 0.0;
		for (size_t i = 0; i < rows.size(); ++i)
		{
			const int sample = rows[i];
			mae += std::abs(lengths[sample] - PassiveSwivelDistance(
				transforms[sample], parameters.segment<3>(0),
				parameters.segment<3>(3), axisLocal, parameters(6)));
		}
		return rows.empty() ? 0.0 : mae / static_cast<double>(rows.size());
	}

	PassiveSwivelFit FitPassiveSwivel(const std::vector<Matrix4d>& transforms,
		const std::vector<double>& lengths, const Vector3d& axisLocal)
	{
		const int samples = static_cast<int>(transforms.size());
		std::vector<int> trainingRows, validationRows, allRows(samples);
		for (int i = 0; i < samples; ++i)
		{
			allRows[i] = i;
			if ((i + 1) % 4 == 0) validationRows.push_back(i);
			else trainingRows.push_back(i);
		}

		PassiveSwivelFit best = { std::numeric_limits<double>::infinity(),
			std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
			Vector3d::Zero(), Vector3d::Zero(), axisLocal, 0.0 };
		const double pivotZStarts[] = { -0.14, -0.10, -0.05, 0.0, 0.05 };
		const double radiusStarts[] = { 0.04, 0.08, 0.12, 0.16 };
		for (double pivotZ : pivotZStarts)
			for (double radiusStart : radiusStarts)
			{
				VectorXd parameters(7);
				parameters.setZero();
				parameters.segment<3>(3) = Vector3d(0.0, 0.0, pivotZ);
				parameters(6) = radiusStart;

				// A point-at-pivot sphere fit is a stable anchor initializer. The
				// nonlinear stage below then uses the actual circular endpoint model.
				std::vector<Vector3d> pivotPositions(samples);
				for (int i = 0; i < samples; ++i)
					pivotPositions[i] = (transforms[i] * Vector4d(0.0, 0.0, pivotZ, 1.0)).head<3>();
				Vector3d initialAnchor;
				DiagnosticMae(pivotPositions, lengths, 1.0, -radiusStart, &initialAnchor);
				parameters.segment<3>(0) = initialAnchor;

				double damping = 1.0e-5;
				double currentCost = PassiveCost(transforms, lengths, trainingRows, parameters, axisLocal);
				for (int iteration = 0; iteration < 250; ++iteration)
				{
					MatrixXd jacobian(static_cast<int>(trainingRows.size()), 7);
					VectorXd residual(static_cast<int>(trainingRows.size()));
					for (size_t row = 0; row < trainingRows.size(); ++row)
					{
						const int sample = trainingRows[row];
						residual(static_cast<int>(row)) = lengths[sample] - PassiveSwivelDistance(
							transforms[sample], parameters.segment<3>(0),
							parameters.segment<3>(3), axisLocal, parameters(6));
					}
					for (int column = 0; column < 7; ++column)
					{
						const double stepSize = 1.0e-5;
						VectorXd plus = parameters, minus = parameters;
						plus(column) += stepSize;
						minus(column) -= stepSize;
						for (size_t row = 0; row < trainingRows.size(); ++row)
						{
							const int sample = trainingRows[row];
							const double plusResidual = lengths[sample] - PassiveSwivelDistance(
								transforms[sample], plus.segment<3>(0), plus.segment<3>(3), axisLocal, plus(6));
							const double minusResidual = lengths[sample] - PassiveSwivelDistance(
								transforms[sample], minus.segment<3>(0), minus.segment<3>(3), axisLocal, minus(6));
							jacobian(static_cast<int>(row), column) =
								(plusResidual - minusResidual) / (2.0 * stepSize);
						}
					}
					for (int row = 0; row < residual.size(); ++row)
					{
						const double absolute = std::abs(residual(row));
						const double weight = absolute <= 0.003 ? 1.0 : 0.003 / absolute;
						const double rootWeight = std::sqrt(weight);
						residual(row) *= rootWeight;
						jacobian.row(row) *= rootWeight;
					}
					MatrixXd normal = jacobian.transpose() * jacobian;
					normal.diagonal().array() += damping;
					VectorXd step = normal.ldlt().solve(-jacobian.transpose() * residual);
					if (!step.allFinite() || step.norm() < 1.0e-11) break;
					for (int i = 0; i < step.size(); ++i)
						step(i) = (std::max)(-0.02, (std::min)(0.02, step(i)));
					VectorXd candidate = parameters + step;
					if (candidate.segment<3>(3).norm() > 0.30 || candidate(6) < 0.01 || candidate(6) > 0.25)
					{
						damping *= 10.0;
						continue;
					}
					const double candidateCost = PassiveCost(transforms, lengths, trainingRows, candidate, axisLocal);
					if (candidateCost < currentCost)
					{
						parameters = candidate;
						if (currentCost - candidateCost < 1.0e-14) { currentCost = candidateCost; break; }
						currentCost = candidateCost;
						damping = (std::max)(1.0e-12, damping / 3.0);
					}
					else damping *= 10.0;
				}

				const double validationMae = PassiveMae(transforms, lengths, validationRows, parameters, axisLocal);
				if (validationMae < best.validationMae)
				{
					best.trainMae = PassiveMae(transforms, lengths, trainingRows, parameters, axisLocal);
					best.validationMae = validationMae;
					best.allMae = PassiveMae(transforms, lengths, allRows, parameters, axisLocal);
					best.anchor = parameters.segment<3>(0);
					best.pivotLocal = parameters.segment<3>(3);
					best.armRadius = parameters(6);
				}
			}
		return best;
	}

	struct AxisCenterFit
	{
		double trainMae;
		double validationMae;
		double allMae;
		Vector3d anchor;
		double centerZ;
		double lengthOffset;
	};

	AxisCenterFit FitAxisCenterModel(const std::vector<Matrix4d>& transforms,
		const std::vector<double>& lengths)
	{
		const int samples = static_cast<int>(transforms.size());
		std::vector<int> trainingRows, validationRows, allRows(samples);
		for (int i = 0; i < samples; ++i)
		{
			allRows[i] = i;
			if ((i + 1) % 4 == 0) validationRows.push_back(i);
			else trainingRows.push_back(i);
		}
		auto mae = [&](const VectorXd& p, const std::vector<int>& rows)
		{
			double sum = 0.0;
			for (int sample : rows)
			{
				const Vector3d position = (transforms[sample] * Vector4d(0.0, 0.0, p(3), 1.0)).head<3>();
				sum += std::abs(lengths[sample] + p(4) - (position - p.head<3>()).norm());
			}
			return rows.empty() ? 0.0 : sum / static_cast<double>(rows.size());
		};
		auto cost = [&](const VectorXd& p)
		{
			double sum = 0.0;
			for (int sample : trainingRows)
			{
				const Vector3d position = (transforms[sample] * Vector4d(0.0, 0.0, p(3), 1.0)).head<3>();
				const double r = lengths[sample] + p(4) - (position - p.head<3>()).norm();
				const double a = std::abs(r), d = 0.003;
				sum += a <= d ? 0.5 * r * r : d * (a - 0.5 * d);
			}
			return sum;
		};

		AxisCenterFit best = { std::numeric_limits<double>::infinity(),
			std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
			Vector3d::Zero(), 0.0, 0.0 };
		const double zStarts[] = { -0.15, -0.10, 0.0, 0.05 };
		const double offsetStarts[] = { 0.0 };
		for (double zStart : zStarts)
			for (double offsetStart : offsetStarts)
			{
				VectorXd p(5);
				p.setZero(); p(3) = zStart; p(4) = offsetStart;
				std::vector<Vector3d> positions(samples);
				for (int i = 0; i < samples; ++i)
					positions[i] = (transforms[i] * Vector4d(0.0, 0.0, zStart, 1.0)).head<3>();
				Vector3d initialAnchor;
				DiagnosticMae(positions, lengths, 1.0, offsetStart, &initialAnchor);
				p.head<3>() = initialAnchor;
				double currentCost = cost(p), damping = 1.0e-5;
				for (int iteration = 0; iteration < 100; ++iteration)
				{
					MatrixXd J(static_cast<int>(trainingRows.size()), 5);
					VectorXd residual(static_cast<int>(trainingRows.size()));
					for (size_t row = 0; row < trainingRows.size(); ++row)
					{
						const int sample = trainingRows[row];
						const Matrix3d rotation = transforms[sample].block<3, 3>(0, 0);
						const Vector3d position = (transforms[sample] * Vector4d(0.0, 0.0, p(3), 1.0)).head<3>();
						const Vector3d difference = position - p.head<3>();
						const double distance = difference.norm();
						const Vector3d direction = difference / distance;
						residual(static_cast<int>(row)) = lengths[sample] + p(4) - distance;
						J.block<1, 3>(static_cast<int>(row), 0) = direction.transpose();
						J(static_cast<int>(row), 3) = -direction.dot(rotation.col(2));
						J(static_cast<int>(row), 4) = 1.0;
						const double absolute = std::abs(residual(static_cast<int>(row)));
						const double weight = absolute <= 0.003 ? 1.0 : 0.003 / absolute;
						const double rootWeight = std::sqrt(weight);
						residual(static_cast<int>(row)) *= rootWeight;
						J.row(static_cast<int>(row)) *= rootWeight;
					}
					MatrixXd normal = J.transpose() * J;
					normal.diagonal().array() += damping;
					VectorXd step = normal.ldlt().solve(-J.transpose() * residual);
					if (!step.allFinite() || step.norm() < 1.0e-11) break;
					for (int i = 0; i < step.size(); ++i)
						step(i) = (std::max)(-0.02, (std::min)(0.02, step(i)));
					VectorXd candidate = p + step;
					if (candidate(3) < -0.30 || candidate(3) > 0.30
						|| candidate(4) < -0.50 || candidate(4) > 0.50)
					{ damping *= 10.0; continue; }
					const double candidateCost = cost(candidate);
					if (candidateCost < currentCost)
					{
						p = candidate;
						if (currentCost - candidateCost < 1.0e-14) { currentCost = candidateCost; break; }
						currentCost = candidateCost;
						damping = (std::max)(1.0e-12, damping / 3.0);
					}
					else damping *= 10.0;
				}
				const double validation = mae(p, validationRows);
				if (validation < best.validationMae)
				{
					best.trainMae = mae(p, trainingRows);
					best.validationMae = validation;
					best.allMae = mae(p, allRows);
					best.anchor = p.head<3>();
					best.centerZ = p(3);
					best.lengthOffset = p(4);
				}
			}
		return best;
	}

	struct JointCenterFit
	{
		double trainMae;
		double validationMae;
		double allMae;
		VectorXd parameters;
	};

	JointCenterFit FitJointCenterAndDq(const std::vector<double>& mdh,
		const std::vector<double>& theta, const std::vector<double>& lengths,
		const AxisCenterFit& initial)
	{
		const int n = 7, samples = static_cast<int>(lengths.size());
		std::vector<int> trainingRows, validationRows, allRows(samples);
		for (int i = 0; i < samples; ++i)
		{
			allRows[i] = i;
			if ((i + 1) % 4 == 0) validationRows.push_back(i); else trainingRows.push_back(i);
		}
		// [anchor xyz, centerZ, rope constant, delta d1..d7, delta q1..q7]
		VectorXd p = VectorXd::Zero(5 + 2 * n);
		p.head<3>() = initial.anchor;
		p(3) = initial.centerZ;
		p(4) = initial.lengthOffset;
		auto sampleResidual = [&](const VectorXd& x, int sample)
		{
			Matrix4d transform = Matrix4d::Identity();
			for (int axis = 0; axis < n; ++axis)
			{
				const double q = theta[axis * samples + sample] + mdh[3 * n + axis] + x(5 + n + axis);
				transform *= JointTransform(q, mdh[axis], mdh[n + axis] + x(5 + axis),
					mdh[2 * n + axis], mdh[4 * n + axis]);
			}
			const Vector3d center = (transform * Vector4d(0.0, 0.0, x(3), 1.0)).head<3>();
			return lengths[sample] + x(4) - (center - x.head<3>()).norm();
		};
		auto mae = [&](const VectorXd& x, const std::vector<int>& rows)
		{
			double sum = 0.0;
			for (int sample : rows) sum += std::abs(sampleResidual(x, sample));
			return rows.empty() ? 0.0 : sum / static_cast<double>(rows.size());
		};
		auto robustCost = [&](const VectorXd& x)
		{
			double sum = 0.0;
			for (int sample : trainingRows)
			{
				const double r = sampleResidual(x, sample), a = std::abs(r), d = 0.002;
				sum += a <= d ? 0.5 * r * r : d * (a - 0.5 * d);
			}
			return sum;
		};

		double currentCost = robustCost(p), damping = 1.0e-4;
		for (int iteration = 0; iteration < 160; ++iteration)
		{
			MatrixXd J(static_cast<int>(trainingRows.size()), p.size());
			VectorXd residual(static_cast<int>(trainingRows.size()));
			for (size_t row = 0; row < trainingRows.size(); ++row)
				residual(static_cast<int>(row)) = sampleResidual(p, trainingRows[row]);
			for (int column = 0; column < p.size(); ++column)
			{
				const bool angle = column >= 5 + n;
				const double h = angle ? 1.0e-7 : 1.0e-6;
				VectorXd plus = p, minus = p;
				plus(column) += h; minus(column) -= h;
				for (size_t row = 0; row < trainingRows.size(); ++row)
					J(static_cast<int>(row), column) =
						(sampleResidual(plus, trainingRows[row]) - sampleResidual(minus, trainingRows[row])) / (2.0 * h);
			}
			for (int row = 0; row < residual.size(); ++row)
			{
				const double a = std::abs(residual(row));
				const double w = a <= 0.002 ? 1.0 : 0.002 / a;
				const double rw = std::sqrt(w);
				residual(row) *= rw; J.row(row) *= rw;
			}
			MatrixXd normal = J.transpose() * J;
			// Weak physical priors prevent unobservable parameter combinations
			// from wandering while leaving millimetre-scale corrections free.
			for (int i = 5; i < 5 + n; ++i) normal(i, i) += 1.0e-4;
			for (int i = 5 + n; i < p.size(); ++i) normal(i, i) += 1.0e-5;
			normal.diagonal().array() += damping;
			VectorXd step = normal.ldlt().solve(-J.transpose() * residual);
			if (!step.allFinite() || step.norm() < 1.0e-10) break;
			for (int i = 0; i < step.size(); ++i)
				step(i) = (std::max)(-0.01, (std::min)(0.01, step(i)));
			VectorXd candidate = p + step;
			bool valid = candidate(3) >= -0.30 && candidate(3) <= 0.30
				&& candidate(4) >= -0.50 && candidate(4) <= 0.70;
			for (int axis = 0; axis < n; ++axis)
				valid = valid && std::abs(candidate(5 + axis)) <= 0.05
					&& std::abs(candidate(5 + n + axis)) <= 5.0 * kPi / 180.0;
			if (!valid) { damping *= 10.0; continue; }
			const double candidateCost = robustCost(candidate);
			if (candidateCost < currentCost)
			{
				p = candidate;
				if (currentCost - candidateCost < 1.0e-14) { currentCost = candidateCost; break; }
				currentCost = candidateCost;
				damping = (std::max)(1.0e-12, damping / 3.0);
			}
			else damping *= 10.0;
		}
		JointCenterFit result;
		result.trainMae = mae(p, trainingRows);
		result.validationMae = mae(p, validationRows);
		result.allMae = mae(p, allRows);
		result.parameters = p;
		return result;
	}

	double DiagnosticMae(const std::vector<Vector3d>& positions,
		const std::vector<double>& rawLengths, double scale, double offset, Vector3d* finalAnchor = NULL)
	{
		const int samples = static_cast<int>(positions.size());
		MatrixXd A(samples - 1, 3);
		VectorXd b(samples - 1);
		for (int i = 1; i < samples; ++i)
		{
			const double previousLength = rawLengths[i - 1] * scale + offset;
			const double currentLength = rawLengths[i] * scale + offset;
			A.row(i - 1) = 2.0 * (positions[i - 1] - positions[i]).transpose();
			b(i - 1) = currentLength * currentLength - previousLength * previousLength
				+ positions[i - 1].squaredNorm() - positions[i].squaredNorm();
		}
		Vector3d anchor = A.colPivHouseholderQr().solve(b);
		for (int iteration = 0; iteration < 50; ++iteration)
		{
			MatrixXd J(samples, 3);
			VectorXd residual(samples);
			for (int i = 0; i < samples; ++i)
			{
				const Vector3d difference = positions[i] - anchor;
				const double distance = difference.norm();
				residual(i) = rawLengths[i] * scale + offset - distance;
				J.row(i) = (difference / distance).transpose();
			}
			Matrix3d normal = J.transpose() * J;
			normal.diagonal().array() += 1.0e-9;
			const Vector3d step = normal.ldlt().solve(-J.transpose() * residual);
			if (!step.allFinite() || step.norm() < 1.0e-13) break;
			anchor += step;
		}
		double mae = 0.0;
		for (int i = 0; i < samples; ++i)
			mae += std::abs(rawLengths[i] * scale + offset - (positions[i] - anchor).norm());
		if (finalAnchor != NULL) *finalAnchor = anchor;
		return mae / samples;
	}

	void FitLengthScaleOffset(const std::vector<Vector3d>& positions,
		const std::vector<double>& rawLengths, double& scale, double& offset,
		Vector3d& anchor, double& mae)
	{
		const int samples = static_cast<int>(positions.size());
		mae = DiagnosticMae(positions, rawLengths, scale, offset, &anchor);
		double damping = 1.0e-6;
		for (int iteration = 0; iteration < 80; ++iteration)
		{
			MatrixXd J(samples, 5);
			VectorXd residual(samples);
			for (int i = 0; i < samples; ++i)
			{
				const Vector3d difference = positions[i] - anchor;
				const double distance = difference.norm();
				residual(i) = rawLengths[i] * scale + offset - distance;
				J.block<1, 3>(i, 0) = (difference / distance).transpose();
				J(i, 3) = rawLengths[i];
				J(i, 4) = 1.0;
			}
			MatrixXd normal = J.transpose() * J;
			normal.diagonal().array() += damping;
			const VectorXd step = normal.ldlt().solve(-J.transpose() * residual);
			if (!step.allFinite()) break;
			const Vector3d candidateAnchor = anchor + step.head<3>();
			const double candidateScale = scale + step(3);
			const double candidateOffset = offset + step(4);
			if (candidateScale < 0.5 || candidateScale > 1.5 || candidateOffset < -1.0 || candidateOffset > 1.0)
			{
				damping *= 10.0;
				continue;
			}
			double candidateMae = 0.0;
			for (int i = 0; i < samples; ++i)
				candidateMae += std::abs(rawLengths[i] * candidateScale + candidateOffset
					- (positions[i] - candidateAnchor).norm());
			candidateMae /= samples;
			if (candidateMae < mae)
			{
				anchor = candidateAnchor;
				scale = candidateScale;
				offset = candidateOffset;
				if (mae - candidateMae < 1.0e-12) { mae = candidateMae; break; }
				mae = candidateMae;
				damping = (std::max)(1.0e-12, damping / 3.0);
			}
			else damping *= 10.0;
		}
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

	void ActualPoseSyntheticTest()
	{
		const int n = 7;
		const int samples = 60;
		std::vector<double> jointRows;
		const std::string jointPath = "RopeEncoderCalibration/Joint/HE3_GY_Joint_2026_6_9_16_48.txt";
		Check(ReadNumbers(jointPath, jointRows), "actual 60-pose HE3_GY trajectory is readable");
		if (jointRows.size() < n * samples) return;

		std::vector<double> theta(n * samples, 0.0);
		for (int sample = 0; sample < samples; ++sample)
			for (int axis = 0; axis < n; ++axis)
				theta[axis * samples + sample] = jointRows[sample * n + axis] * kPi / 180.0;

		const std::vector<double> nominal = He3Mdh();
		std::vector<double> truth = nominal;
		truth[n + 1] += 0.00030;
		truth[n + 2] -= 0.00040;
		truth[n + 3] += 0.00020;
		truth[n + 4] -= 0.00025;
		truth[n + 6] += 0.00050;
		std::vector<double> trueQ0(n, 0.0);
		trueQ0[1] = 0.05 * kPi / 180.0;
		trueQ0[2] = -0.04 * kPi / 180.0;
		trueQ0[3] = 0.03 * kPi / 180.0;
		trueQ0[4] = -0.03 * kPi / 180.0;

		const Vector3d anchor(-1.4, -1.1, -0.03);
		const std::vector<double> lengths = GenerateLengths(truth, theta, trueQ0, samples, anchor, 0.049);
		RunOutput recovered = Run(nominal, theta, lengths, He3Mask(), Options(true));
		std::cout << "[INFO] actual-pose synthetic status=" << recovered.status
			<< " active=" << recovered.result.activeParameterCount
			<< " rank=" << recovered.result.numericalRank
			<< " condition=" << recovered.result.conditionNumber
			<< " MAE(mm)=" << recovered.result.beforeMae * 1000.0 << " -> "
			<< recovered.result.afterMae * 1000.0
			<< " validation(mm)=" << recovered.result.validationBeforeMae * 1000.0 << " -> "
			<< recovered.result.validationAfterMae * 1000.0 << "\n";
		Check(recovered.status == CALIBRATION_V2_OK,
			"actual HE3 60-pose geometry accepts a consistent seven-axis data set");
		Check(recovered.result.activeParameterCount > 0
			&& recovered.result.numericalRank == recovered.result.activeParameterCount + 3,
			"actual HE3 60-pose calibration Jacobian (anchor plus active parameters) has full column rank");
		Check(std::isfinite(recovered.result.conditionNumber)
			&& recovered.result.conditionNumber <= 1.0e4,
			"actual HE3 60-pose calibration Jacobian passes the condition-number gate");
		Check(recovered.result.afterMae < 0.001
			&& recovered.result.validationAfterMae < 0.001,
			"actual HE3 60-pose training and held-out rope MAE both pass 1 mm");
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

	void LatestRunDiagnostics()
	{
		const int n = 7;
		const int samples = 60;
		std::vector<double> jointRows;
		std::vector<double> rawMillimetres;
		const std::string jointPath = "RopeEncoderCalibration/Joint/HE3_GY_Joint_2026_8_14_15_38.txt";
		const std::string dataPath = "RopeEncoderCalibration/data/HE3_GY_Data_2026_8_14_15_38.txt";
		if (!ReadNumbers(jointPath, jointRows) || !ReadNumbers(dataPath, rawMillimetres)
			|| jointRows.size() < n * samples || rawMillimetres.size() < samples)
		{
			std::cout << "[DIAG] latest run files are unavailable; diagnostic skipped\n";
			return;
		}
		std::vector<double> baseTheta(n * samples, 0.0);
		std::vector<double> rawLengths(samples, 0.0);
		for (int sample = 0; sample < samples; ++sample)
		{
			for (int axis = 0; axis < n; ++axis)
				baseTheta[axis * samples + sample] = jointRows[sample * n + axis] * kPi / 180.0;
			rawLengths[sample] = rawMillimetres[sample] / 1000.0;
		}

		struct Candidate
		{
			double mae;
			int model;
			int signMask;
			double scale;
			double offset;
			Vector3d anchor;
			std::vector<double> theta;
		};
		Candidate best = { std::numeric_limits<double>::infinity(), 0, 0, 1.0, 0.0, Vector3d::Zero(), std::vector<double>() };
		const std::vector<double> models[2] = { He3RoundedRunMdh(), He3Mdh() };
		const int reportedMasks[] = { 0, 8, 32, 40, 15 };
		for (int model = 0; model < 2; ++model)
		{
			for (int maskIndex = 0; maskIndex < 5; ++maskIndex)
			{
				const int signMask = reportedMasks[maskIndex];
				std::vector<double> theta = baseTheta;
				for (int axis = 0; axis < n; ++axis)
					if ((signMask & (1 << axis)) != 0)
						for (int sample = 0; sample < samples; ++sample)
							theta[axis * samples + sample] = -theta[axis * samples + sample];
				const std::vector<Vector3d> positions = DiagnosticPositions(models[model], theta, samples, 0.049);
				std::cout << "[DIAG] convention model=" << (model == 0 ? "rounded" : "precise")
					<< " signMask=" << signMask << " MAE(mm)="
					<< DiagnosticMae(positions, rawLengths, 1.0, 0.0) * 1000.0 << "\n";
			}
		}
		for (int model = 0; model < 2; ++model)
		{
			for (int signMask = 0; signMask < (1 << n); ++signMask)
			{
				std::vector<double> theta = baseTheta;
				for (int axis = 0; axis < n; ++axis)
					if ((signMask & (1 << axis)) != 0)
						for (int sample = 0; sample < samples; ++sample)
							theta[axis * samples + sample] = -theta[axis * samples + sample];
				const std::vector<Vector3d> positions = DiagnosticPositions(models[model], theta, samples, 0.049);
				Vector3d anchor;
				const double mae = DiagnosticMae(positions, rawLengths, 1.0, 0.0, &anchor);
				if (mae < best.mae)
					best = { mae, model, signMask, 1.0, 0.0, anchor, theta };
			}
		}

		const std::vector<Vector3d> bestPositions = DiagnosticPositions(models[best.model], best.theta, samples, 0.049);
		FitLengthScaleOffset(bestPositions, rawLengths, best.scale, best.offset, best.anchor, best.mae);

		std::cout << "[DIAG] latest best geometry-only MAE(mm)=" << best.mae * 1000.0
			<< " model=" << (best.model == 0 ? "rounded" : "precise")
			<< " signMask=" << best.signMask
			<< " scale=" << best.scale
			<< " offset(mm)=" << best.offset * 1000.0
			<< " anchor=" << best.anchor.transpose() << "\n";
		std::cout << "[DIAG] flipped axes (1-based):";
		for (int axis = 0; axis < n; ++axis)
			if ((best.signMask & (1 << axis)) != 0) std::cout << " " << axis + 1;
		std::cout << "\n";

		// Run the production solver once with the best convention/scale/offset so
		// diagnostics show whether robot-parameter corrections generalize.
		std::vector<double> correctedLengths(samples);
		for (int i = 0; i < samples; ++i) correctedLengths[i] = rawLengths[i] * best.scale;
		CalibrationV2Options options = Options(true);
		options.ropeLengthOffset = best.offset;
		RunOutput replay = Run(models[best.model], best.theta, correctedLengths, He3Mask(), options);
		std::cout << "[DIAG] latest production replay status=" << replay.status
			<< " MAE(mm)=" << replay.result.beforeMae * 1000.0 << " -> " << replay.result.afterMae * 1000.0
			<< " validation(mm)=" << replay.result.validationBeforeMae * 1000.0 << " -> "
			<< replay.result.validationAfterMae * 1000.0 << "\n";

		std::vector<int> expandedMask(4 * n, 1);
		CalibrationV2Options expandedOptions = Options(true);
		expandedOptions.ropeLengthOffset = best.offset;
		expandedOptions.maxLengthCorrection = 0.100;
		expandedOptions.maxAngleCorrection = 20.0 * kPi / 180.0;
		expandedOptions.maxConditionNumber = 1.0e5;
		RunOutput expanded = Run(models[best.model], best.theta, correctedLengths, expandedMask, expandedOptions);
		std::cout << "[DIAG] expanded-model status=" << expanded.status
			<< " active=" << expanded.result.activeParameterCount
			<< " condition=" << expanded.result.conditionNumber
			<< " MAE(mm)=" << expanded.result.beforeMae * 1000.0 << " -> " << expanded.result.afterMae * 1000.0
			<< " validation(mm)=" << expanded.result.validationBeforeMae * 1000.0 << " -> "
			<< expanded.result.validationAfterMae * 1000.0 << "\n";

		for (int model = 0; model < 2; ++model)
		{
			const std::vector<Matrix4d> transforms = DiagnosticTransforms(models[model], baseTheta, samples);
			Vector3d tool(0.0, 0.0, 0.049);
			Vector3d anchor = Vector3d::Zero();
			double scale = 1.0;
			double offset = 0.0;
			const double fixedLengthMae = FitToolPoint(transforms, rawLengths, false, tool, anchor, scale, offset);
			std::cout << "[DIAG] fitted tool fixed-scale model=" << (model == 0 ? "rounded" : "precise")
				<< " MAE(mm)=" << fixedLengthMae * 1000.0 << " tool(mm)=" << (tool * 1000.0).transpose()
				<< " anchor=" << anchor.transpose() << "\n";
			if (model == 1)
			{
				CalibrationV2Options toolOptions = Options(true);
				toolOptions.toolOffset[0] = tool.x();
				toolOptions.toolOffset[1] = tool.y();
				toolOptions.toolOffset[2] = tool.z();
				RunOutput toolReplay = Run(models[model], baseTheta, rawLengths, He3Mask(), toolOptions);
				std::cout << "[DIAG] fitted-tool production status=" << toolReplay.status
					<< " MAE(mm)=" << toolReplay.result.beforeMae * 1000.0 << " -> " << toolReplay.result.afterMae * 1000.0
					<< " validation(mm)=" << toolReplay.result.validationBeforeMae * 1000.0 << " -> "
					<< toolReplay.result.validationAfterMae * 1000.0 << "\n";
				std::vector<int> toolExpandedMask(4 * n, 1);
				toolOptions.maxLengthCorrection = 0.020;
				toolOptions.maxAngleCorrection = 5.0 * kPi / 180.0;
				toolOptions.maxConditionNumber = 1.0e5;
				RunOutput toolExpanded = Run(models[model], baseTheta, rawLengths, toolExpandedMask, toolOptions);
				std::cout << "[DIAG] fitted-tool expanded status=" << toolExpanded.status
					<< " active=" << toolExpanded.result.activeParameterCount
					<< " MAE(mm)=" << toolExpanded.result.beforeMae * 1000.0 << " -> " << toolExpanded.result.afterMae * 1000.0
					<< " validation(mm)=" << toolExpanded.result.validationBeforeMae * 1000.0 << " -> "
					<< toolExpanded.result.validationAfterMae * 1000.0 << "\n";
			}
			tool = Vector3d(0.0, 0.0, 0.049);
			anchor.setZero(); scale = 1.0; offset = 0.0;
			const double freeLengthMae = FitToolPoint(transforms, rawLengths, true, tool, anchor, scale, offset);
			std::cout << "[DIAG] fitted tool free-scale model=" << (model == 0 ? "rounded" : "precise")
				<< " MAE(mm)=" << freeLengthMae * 1000.0 << " tool(mm)=" << (tool * 1000.0).transpose()
				<< " scale=" << scale << " offset(mm)=" << offset * 1000.0
				<< " anchor=" << anchor.transpose() << "\n";
			if (model == 1)
			{
				std::vector<double> adjustedLengths(samples);
				for (int i = 0; i < samples; ++i) adjustedLengths[i] = rawLengths[i] * scale;
				CalibrationV2Options freeOptions = Options(true);
				freeOptions.toolOffset[0] = tool.x();
				freeOptions.toolOffset[1] = tool.y();
				freeOptions.toolOffset[2] = tool.z();
				freeOptions.ropeLengthOffset = offset;
				freeOptions.maxLengthCorrection = 0.100;
				freeOptions.maxAngleCorrection = 20.0 * kPi / 180.0;
				freeOptions.maxConditionNumber = 1.0e5;
				std::vector<int> allMask(4 * n, 1);
				RunOutput freeExpanded = Run(models[model], baseTheta, adjustedLengths, allMask, freeOptions);
				std::cout << "[DIAG] fitted-tool-scale expanded status=" << freeExpanded.status
					<< " MAE(mm)=" << freeExpanded.result.beforeMae * 1000.0 << " -> " << freeExpanded.result.afterMae * 1000.0
					<< " validation(mm)=" << freeExpanded.result.validationBeforeMae * 1000.0 << " -> "
					<< freeExpanded.result.validationAfterMae * 1000.0 << "\n";
				for (int parameter = 0; parameter < 4 * n; ++parameter)
					if (freeExpanded.state[parameter] == CALIBRATION_PARAMETER_ACTIVE)
						std::cout << "[DIAG] free expanded delta p" << parameter << "="
							<< freeExpanded.delta[parameter] << "\n";
			}
			tool = Vector3d(0.0, 0.0, 0.049);
			anchor.setZero(); scale = -1.0; offset = 1.5;
			const double reverseMae = FitToolPoint(transforms, rawLengths, true, tool, anchor, scale, offset);
			std::cout << "[DIAG] fitted tool reverse-count model=" << (model == 0 ? "rounded" : "precise")
				<< " MAE(mm)=" << reverseMae * 1000.0 << " tool(mm)=" << (tool * 1000.0).transpose()
				<< " scale=" << scale << " offset(mm)=" << offset * 1000.0
				<< " anchor=" << anchor.transpose() << "\n";

			if (model == 1)
			{
				const AxisCenterFit center = FitAxisCenterModel(transforms, rawLengths);
				std::cout << "[CENTER] train(mm)=" << center.trainMae * 1000.0
					<< " validation(mm)=" << center.validationMae * 1000.0
					<< " all(mm)=" << center.allMae * 1000.0
					<< " centerZ(mm)=" << center.centerZ * 1000.0
					<< " lengthOffset(mm)=" << center.lengthOffset * 1000.0
					<< " anchor=" << center.anchor.transpose() << "\n";
			}
		}

		double bestShiftMae = std::numeric_limits<double>::infinity();
		int bestShift = 0;
		const std::vector<Vector3d> unshiftedPositions = DiagnosticPositions(He3Mdh(), baseTheta, samples, 0.049);
		for (int shift = 0; shift < samples; ++shift)
		{
			std::vector<double> shiftedLengths(samples);
			for (int i = 0; i < samples; ++i) shiftedLengths[i] = rawLengths[(i + shift) % samples];
			const double mae = DiagnosticMae(unshiftedPositions, shiftedLengths, 1.0, 0.0);
			if (mae < bestShiftMae) { bestShiftMae = mae; bestShift = shift; }
		}
		std::cout << "[DIAG] best cyclic sample shift=" << bestShift << " MAE(mm)=" << bestShiftMae * 1000.0 << "\n";
	}

	void ToolPointHistoryDiagnostics()
	{
		const int n = 7;
		const int samples = 60;
		const char* suffixes[] = { "2026_6_9_16_48", "2026_8_14_15_38" };
		const double offsets[] = { 0.0625, 0.0 };
		for (int dataset = 0; dataset < 2; ++dataset)
		{
			std::vector<double> jointRows, rawMillimetres;
			const std::string suffix = suffixes[dataset];
			if (!ReadNumbers("RopeEncoderCalibration/Joint/HE3_GY_Joint_" + suffix + ".txt", jointRows)
				|| !ReadNumbers("RopeEncoderCalibration/data/HE3_GY_Data_" + suffix + ".txt", rawMillimetres))
				continue;
			std::vector<double> theta(n * samples), lengths(samples);
			for (int sample = 0; sample < samples; ++sample)
			{
				for (int axis = 0; axis < n; ++axis)
					theta[axis * samples + sample] = jointRows[sample * n + axis] * kPi / 180.0;
				lengths[sample] = rawMillimetres[sample] / 1000.0 + offsets[dataset];
			}
			const std::vector<Matrix4d> transforms = DiagnosticTransforms(He3Mdh(), theta, samples);
			Vector3d tool(0.0, 0.0, 0.049), anchor = Vector3d::Zero();
			double scale = 1.0, offset = 0.0;
			const double mae = FitToolPoint(transforms, lengths, false, tool, anchor, scale, offset);
			std::cout << "[TOOL-HISTORY] " << suffix << " MAE(mm)=" << mae * 1000.0
				<< " tool(mm)=" << (tool * 1000.0).transpose() << " anchor=" << anchor.transpose() << "\n";
		}
	}

	void CenterModelDiagnostics()
	{
		const int n = 7, samples = 60;
		std::vector<double> jointRows, rawMillimetres;
		if (!ReadNumbers("RopeEncoderCalibration/Joint/HE3_GY_Joint_2026_8_14_15_38.txt", jointRows)
			|| !ReadNumbers("RopeEncoderCalibration/data/HE3_GY_Data_2026_8_14_15_38.txt", rawMillimetres))
		{
			std::cout << "[CENTER] input files unavailable\n";
			return;
		}
		std::vector<double> theta(n * samples), lengths(samples);
		for (int sample = 0; sample < samples; ++sample)
		{
			for (int axis = 0; axis < n; ++axis)
				theta[axis * samples + sample] = jointRows[sample * n + axis] * kPi / 180.0;
			lengths[sample] = rawMillimetres[sample] / 1000.0;
		}
		const std::vector<double> mdh = He3Mdh();
		const std::vector<Matrix4d> transforms = DiagnosticTransforms(mdh, theta, samples);
		std::vector<Vector3d> physicalCenterPositions(samples);
		for (int i = 0; i < samples; ++i)
			physicalCenterPositions[i] = transforms[i].block<3, 1>(0, 3);
		Vector3d physicalAnchor;
		const double physicalMae = DiagnosticMae(
			physicalCenterPositions, lengths, 1.0, 0.0625 + 0.049, &physicalAnchor);
		std::cout << "[CENTER-PHYSICAL] toolCenter=0, deadLength=62.5mm, radius=49mm, MAE(mm)="
			<< physicalMae * 1000.0 << " anchor=" << physicalAnchor.transpose() << "\n";
		// In this Craig-style MDH implementation d7 translates the final frame.
		// Also report the candidate centre one nominal d7 back along local Z; this
		// helps distinguish "final frame origin" from the physical flange centre
		// without silently selecting either interpretation for production.
		std::vector<Vector3d> beforeD7CenterPositions(samples);
		const double nominalD7 = mdh[n + n - 1];
		for (int i = 0; i < samples; ++i)
		{
			const Vector4d localCentre(0.0, 0.0, -nominalD7, 1.0);
			beforeD7CenterPositions[i] = (transforms[i] * localCentre).head<3>();
		}
		Vector3d beforeD7Anchor;
		const double beforeD7Mae = DiagnosticMae(
			beforeD7CenterPositions, lengths, 1.0, 0.0625 + 0.049, &beforeD7Anchor);
		std::cout << "[CENTER-PHYSICAL] localZ=-d7=" << -nominalD7 * 1000.0
			<< "mm, deadLength+radius=111.5mm, MAE(mm)=" << beforeD7Mae * 1000.0
			<< " anchor=" << beforeD7Anchor.transpose() << "\n";
		CalibrationV2Options beforeD7Options = Options(true);
		beforeD7Options.toolOffset[2] = -nominalD7;
		beforeD7Options.ropeLengthOffset = 0.0625 + 0.049;
		RunOutput beforeD7Run = Run(mdh, theta, lengths, He3Mask(), beforeD7Options);
		std::cout << "[CENTER-PHYSICAL] before-d7 selected status=" << beforeD7Run.status
			<< " MAE(mm)=" << beforeD7Run.result.beforeMae * 1000.0 << " -> "
			<< beforeD7Run.result.afterMae * 1000.0 << " validation(mm)="
			<< beforeD7Run.result.validationBeforeMae * 1000.0 << " -> "
			<< beforeD7Run.result.validationAfterMae * 1000.0 << "\n";
		for (int parameter = 0; parameter < 4 * n; ++parameter)
			if (beforeD7Run.state[parameter] == CALIBRATION_PARAMETER_ACTIVE)
				std::cout << "[CENTER-PHYSICAL-DELTA] p" << parameter << "="
					<< beforeD7Run.delta[parameter] << "\n";
		CalibrationV2Options physicalOptions = Options(true);
		physicalOptions.toolOffset[2] = 0.0;
		physicalOptions.ropeLengthOffset = 0.0625 + 0.049;
		physicalOptions.maxLengthCorrection = 0.020;
		physicalOptions.maxAngleCorrection = 5.0 * kPi / 180.0;
		physicalOptions.maxConditionNumber = 1.0e5;
		RunOutput physicalRun = Run(mdh, theta, lengths, He3Mask(), physicalOptions);
		std::cout << "[CENTER-PHYSICAL] status=" << physicalRun.status
			<< " MAE(mm)=" << physicalRun.result.beforeMae * 1000.0 << " -> " << physicalRun.result.afterMae * 1000.0
			<< " validation(mm)=" << physicalRun.result.validationBeforeMae * 1000.0 << " -> "
			<< physicalRun.result.validationAfterMae * 1000.0 << "\n";
		const AxisCenterFit center = FitAxisCenterModel(transforms, lengths);
		std::cout << "[CENTER] train(mm)=" << center.trainMae * 1000.0
			<< " validation(mm)=" << center.validationMae * 1000.0
			<< " all(mm)=" << center.allMae * 1000.0
			<< " centerZ(mm)=" << center.centerZ * 1000.0
			<< " lengthOffset(mm)=" << center.lengthOffset * 1000.0
			<< " anchor=" << center.anchor.transpose() << "\n";
		const JointCenterFit jointCenter = FitJointCenterAndDq(mdh, theta, lengths, center);
		std::cout << "[CENTER-JOINT] train(mm)=" << jointCenter.trainMae * 1000.0
			<< " validation(mm)=" << jointCenter.validationMae * 1000.0
			<< " all(mm)=" << jointCenter.allMae * 1000.0
			<< " centerZ(mm)=" << jointCenter.parameters(3) * 1000.0
			<< " lengthOffset(mm)=" << jointCenter.parameters(4) * 1000.0 << "\n";
		for (int axis = 0; axis < n; ++axis)
			std::cout << "[CENTER-JOINT] d" << axis + 1 << " delta(mm)="
				<< jointCenter.parameters(5 + axis) * 1000.0 << ", q" << axis + 1
				<< " delta(deg)=" << jointCenter.parameters(5 + n + axis) * 180.0 / kPi << "\n";

		CalibrationV2Options options = Options(true);
		options.toolOffset[2] = center.centerZ;
		options.ropeLengthOffset = center.lengthOffset;
		options.maxLengthCorrection = 0.020;
		options.maxAngleCorrection = 5.0 * kPi / 180.0;
		options.maxConditionNumber = 1.0e5;
		RunOutput selected = Run(mdh, theta, lengths, He3Mask(), options);
		std::cout << "[CENTER-PROD] selected status=" << selected.status
			<< " active=" << selected.result.activeParameterCount
			<< " MAE(mm)=" << selected.result.beforeMae * 1000.0 << " -> " << selected.result.afterMae * 1000.0
			<< " validation(mm)=" << selected.result.validationBeforeMae * 1000.0 << " -> "
			<< selected.result.validationAfterMae * 1000.0 << "\n";

		std::vector<int> dAndQMask(4 * n, 0);
		for (int axis = 0; axis < n; ++axis)
		{
			dAndQMask[n + axis] = 1;
			dAndQMask[3 * n + axis] = 1;
		}
		RunOutput dAndQ = Run(mdh, theta, lengths, dAndQMask, options);
		std::cout << "[CENTER-PROD] d+q status=" << dAndQ.status
			<< " active=" << dAndQ.result.activeParameterCount
			<< " MAE(mm)=" << dAndQ.result.beforeMae * 1000.0 << " -> " << dAndQ.result.afterMae * 1000.0
			<< " validation(mm)=" << dAndQ.result.validationBeforeMae * 1000.0 << " -> "
			<< dAndQ.result.validationAfterMae * 1000.0 << "\n";
		for (int parameter = 0; parameter < 4 * n; ++parameter)
			if (dAndQ.state[parameter] == CALIBRATION_PARAMETER_ACTIVE)
				std::cout << "[CENTER-DQ] delta p" << parameter << "=" << dAndQ.delta[parameter] << "\n";

		std::vector<int> expandedMask(4 * n, 1);
		RunOutput expanded = Run(mdh, theta, lengths, expandedMask, options);
		std::cout << "[CENTER-PROD] expanded status=" << expanded.status
			<< " active=" << expanded.result.activeParameterCount
			<< " MAE(mm)=" << expanded.result.beforeMae * 1000.0 << " -> " << expanded.result.afterMae * 1000.0
			<< " validation(mm)=" << expanded.result.validationBeforeMae * 1000.0 << " -> "
			<< expanded.result.validationAfterMae * 1000.0 << "\n";
		for (int parameter = 0; parameter < 4 * n; ++parameter)
			if (expanded.state[parameter] == CALIBRATION_PARAMETER_ACTIVE)
				std::cout << "[CENTER-PROD] delta p" << parameter << "=" << expanded.delta[parameter] << "\n";

		CalibrationV2Options allPointOptions = options;
		allPointOptions.useValidationSplit = 0;
		allPointOptions.maxLengthCorrection = 0.100;
		allPointOptions.maxAngleCorrection = 20.0 * kPi / 180.0;
		RunOutput allPoint = Run(mdh, theta, lengths, expandedMask, allPointOptions);
		std::cout << "[CENTER-PROD] all-point-expanded status=" << allPoint.status
			<< " active=" << allPoint.result.activeParameterCount
			<< " MAE(mm)=" << allPoint.result.beforeMae * 1000.0 << " -> " << allPoint.result.afterMae * 1000.0
			<< " max(mm)=" << allPoint.result.afterMax * 1000.0 << "\n";
		for (int parameter = 0; parameter < 4 * n; ++parameter)
			if (allPoint.state[parameter] == CALIBRATION_PARAMETER_ACTIVE)
				std::cout << "[CENTER-ALL] delta p" << parameter << "=" << allPoint.delta[parameter] << "\n";
	}
}

int main()
{
	std::cout << "Build: " << CalibrationBuildId() << "\n";
	if (std::getenv("HE3_LATEST_DIAG") != NULL)
	{
		LatestRunDiagnostics();
		return 0;
	}
	if (std::getenv("HE3_TOOL_DIAG") != NULL)
	{
		ToolPointHistoryDiagnostics();
		return 0;
	}
	if (std::getenv("HE3_CENTER_DIAG") != NULL)
	{
		CenterModelDiagnostics();
		return 0;
	}
	SyntheticTests();
	ActualPoseSyntheticTest();
	RealReplayTest();
	std::cout << (failures == 0 ? "ALL TESTS PASSED\n" : "TEST FAILURES: " + std::to_string(failures) + "\n");
	return failures == 0 ? 0 : 1;
}
