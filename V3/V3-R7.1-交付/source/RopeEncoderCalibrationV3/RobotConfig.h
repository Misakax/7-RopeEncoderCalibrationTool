#pragma once
class RbtConfig
{
	public:
	int RobotType = 0;
	int RobotAxis = 0;
	int CalLocationNumber = 0;
	double ToolOffset;
	double* a;
	double* alpha;
	double* d;
	double* theta;
	double* beta;
	double* EncoderLineNum;
	double* MoveDirection;
	int MotionWaittime = 60000;
	int GetLocWaittime = 8000;
};