// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the MATHEMATICALDLL_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// MATHEMATICALDLL_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifndef MATHEMATICALDLL_H
#define MATHEMATICALDLL_H

#ifdef MATHEMATICALDLL_EXPORTS
#define MATHEMATICALDLL_API __declspec(dllexport)
#else
#define MATHEMATICALDLL_API __declspec(dllimport)
#endif

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include<math.h>
#include <string>

using namespace Eigen;
using namespace std;


class robot
{
public:

	int joint;

	VectorXi pos;      // 关节位置索引
	VectorXd r;        // 关节变量向量

	Matrix4d g0;       // 基坐标系到末端的初始位姿
	MatrixXd xi;       // 运动学参数矩阵
	MatrixXd DH;       // DH 参数矩阵

	// 计算当前关节角 theta 下的末端齐次变换矩阵
	Matrix4d gst(VectorXd theta);

	// 计算机器人正运动学（基于关节 q 与偏置 beta）
	Matrix4d robot::Fkine(VectorXd q, VectorXd beta);
	// VectorXd Ikine(Matrix4d gst,Vector3i pos);
};

// CalibrationV2 uses a plain C ABI so the application and DLL do not exchange
// Eigen objects across the module boundary. All length values are metres and
// all angular values are radians.
enum CalibrationV2Status
{
	CALIBRATION_V2_OK = 0,
	CALIBRATION_V2_INVALID_ARGUMENT = -1,
	CALIBRATION_V2_UNSUPPORTED_ROBOT = -2,
	CALIBRATION_V2_INSUFFICIENT_SAMPLES = -3,
	CALIBRATION_V2_NO_OBSERVABLE_PARAMETERS = -4,
	CALIBRATION_V2_NUMERICAL_FAILURE = -5,
	CALIBRATION_V2_CORRECTION_OUT_OF_RANGE = -6
};

enum CalibrationV2ParameterState
{
	CALIBRATION_PARAMETER_FROZEN = 0,
	CALIBRATION_PARAMETER_ACTIVE = 1,
	CALIBRATION_PARAMETER_UNOBSERVABLE = 2,
	CALIBRATION_PARAMETER_ILL_CONDITIONED = 3
};

struct CalibrationV2Options
{
	int structSize;
	int maxIterations;
	double toolOffset[3];
	double ropeLengthOffset;
	double rankTolerance;
	double maxConditionNumber;
	double huberDelta;
	double initialDamping;
	double lengthParameterScale;
	double angleParameterScale;
	double maxLengthCorrection;
	double maxAngleCorrection;
	int useValidationSplit;
};

struct CalibrationV2Result
{
	int structSize;
	int status;
	int axisCount;
	int sampleCount;
	int iterationCount;
	int parameterCount;
	int activeParameterCount;
	int numericalRank;
	int validationSampleCount;
	int writeBackAllowed;
	double conditionNumber;
	double anchor[3];
	double beforeMae;
	double beforeRmse;
	double beforeMax;
	double afterMae;
	double afterRmse;
	double afterMax;
	double validationBeforeMae;
	double validationAfterMae;
	char message[256];
};

extern "C" MATHEMATICALDLL_API const char* _stdcall CalibrationBuildId();

// Array layouts:
//   mdh5ByAxis / updatedMdh5ByAxis: row-major [a; d; alpha; theta; beta]
//   thetaAxisBySample:              row-major axis x sample
//   parameter arrays:               [a(0..n-1), d, alpha, q0]
extern "C" MATHEMATICALDLL_API int _stdcall CalibrationV2(
	const int RbtType,
	const int RbtAxis,
	const int sampleCount,
	const double* mdh5ByAxis,
	const double* thetaAxisBySample,
	const double* measuredLengths,
	const int* requestedParameterMask,
	const CalibrationV2Options* options,
	double* updatedMdh5ByAxis,
	double* parameterDelta4ByAxis,
	int* parameterState4ByAxis,
	double* zeroOffsets,
	double* beforeResiduals,
	double* afterResiduals,
	CalibrationV2Result* result);

// Calibration:
//  输入：
//    RbtType   - 机器人类型（Scara, Delta, SixAxis, Cobot）
//    RbtAxis   - 机器人关节数
//    RbtDH     - 初始DH参数矩阵，按行存储 [a; d; alpha; theta; beta]
//    theta     - 多组关节角姿态矩阵，每列对应一个姿态
//    L         - 各姿态下的测量杆长数组
//  输出：
//    DHNewParams - 计算得到的修正后的DH参数
//    oldError    - 初始误差数组指针（由函数分配）
//    nowError    - 迭代后最优误差数组指针（由函数分配）
// This declaration is retained only for legacy non-6/7-axis callers in the
// historical upper application. HE3 six/seven-axis code must use CalibrationV2.
extern MATHEMATICALDLL_API void _stdcall Calibration(
	const int RbtType,
	const int RbtAxis,
	const MatrixXd RbtDH,
	const MatrixXd theta,
	const VectorXd L,
	double* DHNewParams,
	double** oldError,
	double** nowError);
#endif
