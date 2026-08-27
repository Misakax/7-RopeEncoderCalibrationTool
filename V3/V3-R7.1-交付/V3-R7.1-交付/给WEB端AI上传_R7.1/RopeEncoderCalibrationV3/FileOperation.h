#pragma once
#ifndef Test
#define Test
#include <Eigen/Dense>
#include <string>
#include <vector>
#include"RobotConfig.h"


	Eigen::MatrixXd ReadThor_Joint(int robotType, int RobotAxis, int LocNum, std::string tPath, bool bSkipLine = false);
	bool File_Exist(const std::string& name);
	int DecodeXmlFile();
	CString JointSystemIDNWriteCmdStr(CString IDNModule, CString IDNIndex, CString value, CString arrIndex = _T(""));
	CString JointSystemIDNReadCmdStr(CString IDNModule, CString IDNIndex, CString arrIndex);

	int DownLoad(CString IP, int port, CString userName, CString userPw);
	int UpLoad(CString IP, int port, CString userName, CString userPw);
	int DeleteTemp();
	int ReplaceXmlValue(CString targetIDN, CString IDNValue);
	int WriteXml();
	int FindXmlValue(CString targetIDN);
	std::string GetXmlValue(int tIdx);
	int SaveCalibResult(std::string* sMessage);
	void GetRobotList(std::string path, std::vector<std::string>& files);
	void ReadConfigFile(std::string tPath, RbtConfig& Paras);
	void DecodePara(std::string tMsg, RbtConfig& tPara);
	void trim(std::string &s);
	void SplitString(const std::string& s, std::vector<std::string>& v, const std::string& c);
	void ParaSplit(const std::string& s, double* v);




////不同轴对应的 IDN 值
//CString m_IDNAxis1 = _T("P-0-0513.0.168");
//CString m_IDNAxis2 = _T("P-0-0513.0.169");
//CString m_IDNAxis3 = _T("P-0-0513.0.170");
//CString m_IDNAxis4 = _T("P-0-0513.0.171");
//CString m_IDNAxis5 = _T("P-0-0513.0.172");
//CString m_IDNAxis6 = _T("P-0-0513.0.173");
//
//extern enum MsgIdx;
#endif // !Test




