#pragma once
class RbtConfig
{
	public:
	int RobotType = 0;
	int RobotAxis = 0;
	int CalLocationNumber = 0;
	double ToolOffset = 0.0;
	double RopeLengthOffset = 0.0;
	double* a = nullptr;
	double* alpha = nullptr;
	double* d = nullptr;
	double* theta = nullptr;
	double* beta = nullptr;
	double* EncoderLineNum = nullptr;
	double* MoveDirection = nullptr;
	int MotionWaittime = 60000;
	int GetLocWaittime = 8000;

	~RbtConfig() { ResetArrays(); }

	void ResetArrays()
	{
		delete[] a;
		delete[] alpha;
		delete[] d;
		delete[] theta;
		delete[] beta;
		delete[] EncoderLineNum;
		delete[] MoveDirection;
		a = alpha = d = theta = beta = EncoderLineNum = MoveDirection = nullptr;
	}

	RbtConfig() = default;
	RbtConfig(const RbtConfig&) = delete;
	RbtConfig& operator=(const RbtConfig&) = delete;
};
