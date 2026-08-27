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

	VectorXi pos;
	VectorXd r;

	Matrix4d g0;
	MatrixXd xi;
	MatrixXd DH; //= MatrixXd::Zero(7, 6);

				 //³ÉÔ±º¯Êý
	Matrix4d gst(VectorXd theta);
	Matrix4d robot::Fkine(VectorXd q, VectorXd beta);
	// VectorXd Ikine(Matrix4d gst,Vector3i pos);
};

extern MATHEMATICALDLL_API void _stdcall Calibration(const int RbtType, const int RbtAxis, const MatrixXd RbtDH, const MatrixXd theta, const VectorXd L, double* DHNewParams, double** oldError, double** nowError);

#endif