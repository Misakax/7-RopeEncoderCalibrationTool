#pragma once
#ifndef Test
#define Test
#include <vector>
#include"RobotConfig.h"


	MatrixXd ReadThor_Joint(int robotType, int RobotAxis, int LocNum, string tPath, bool bSkipLine = false);
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
	string GetXmlValue(int tIdx);
	int SaveCalibResult(string* sMessage);
	void GetRobotList(string path, vector<string>& files);
	bool ReadConfigFile(string tPath, RbtConfig& Paras);
	void DecodePara(string tMsg, RbtConfig& tPara);
	void trim(string &s);
	void SplitString(const string& s, vector<string>& v, const string& c);
	bool ParaSplit(const string& s, double* v, int expectedCount);




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




