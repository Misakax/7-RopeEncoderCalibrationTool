#include <sstream>
#include <iostream>
#include <vector>
#include <fstream>
#ifndef ad
#include <Eigen/Dense>
#endif // !ad
#include<math.h>
#include <direct.h> 
#include <afxinet.h>
#include <atlstr.h>
#include <io.h>
#include<string>
#include"RobotConfig.h"
//#include"FileOperation.h"
#define PI 3.1415926
#pragma comment(lib,"ws2_32.lib")

using namespace Eigen;
using namespace std;

enum FtpError { Success, ConnectFail, GetCurrentPath } _ftpError;

#pragma region 变量
/// <summary>
/// xml 报文解析的头报文
/// </summary>
string _headFlag = "<customItem";
/// <summary>
/// xml 报文解析的结尾
/// </summary>
string _tailFlag = "</customItem>";
/// <summary>
/// xml 报文 IDN 解析的标志
/// </summary>
string _idFlag = "id";
/// <summary>
/// xml 报文参数类型解析的标志
/// </summary>
string _attributeFlag = "attribute";
/// <summary>
/// xml 报文解析的头报文结束标志
/// </summary>
string _headEndFlag = ">";
/// <summary>
/// xml 报文结束标识
/// </summary>
string _fileEndFlag = "</robotparam>";
/// <summary>
/// xml IDN 数组
/// </summary>
vector<string> idArray;
/// <summary>
/// xml 类型数组
/// </summary>
vector<string> attributeArray;
/// <summary>
/// xml 值数组
/// </summary>
vector<string> xmlValueArray;
/// <summary>
/// xml 文件读取的字符串数组
/// </summary>
vector<string> fileArray;
/// <summary>
/// IDN 对应文件数组的索引
/// </summary>
vector<int> idxArray;
/// <summary>
/// 字符串初始化判断位
/// </summary>
bool _strInit = false;
/// <summary>
/// 保存标定参数的字符串开头
/// </summary>
string* _msgHeader;
/// <summary>
/// 保存标定参数索引
/// </summary>
enum MsgIdx
{
	StartTime,
	FinishTime,
	A2_Before,
	A3_Before,
	D0_Before,
	D3_Before,
	D4_Before,
	D5_Before,
	J0_Before,
	J1_Before,
	J2_Before,
	J3_Before,
	J4_Before,
	J5_Before,

	A2_After,
	A3_After,
	D0_After,
	D3_After,
	D4_After,
	D5_After,
	J0_Offset,
	J1_Offset,
	J2_Offset,
	J3_Offset,
	J4_Offset,
	J5_Offset,
	J0_After,
	J1_After,
	J2_After,
	J3_After,
	J4_After,
	J5_After,

	Error_Before,
	Error_After,
};

//不同轴对应的 IDN 值
CString m_IDNAxis11 = _T("P-0-0513.0.168");
CString m_IDNAxis12 = _T("P-0-0513.0.169");
CString m_IDNAxis13 = _T("P-0-0513.0.170");
CString m_IDNAxis14 = _T("P-0-0513.0.171");
CString m_IDNAxis15 = _T("P-0-0513.0.172");
CString m_IDNAxis16 = _T("P-0-0513.0.173");

extern CString m_IDNAxis11;
extern CString m_IDNAxis12;
extern CString m_IDNAxis13;
extern CString m_IDNAxis14;
extern CString m_IDNAxis15;
extern CString m_IDNAxis16;

enum ConifgPara {
	RbtType = 1,
	RbtAxis,
	LocNum,
	ToolOffset,
	a,
	alpha,
	d,
	theta,
	beta,
	MoveDirection,
	EncoderLineNum,
	MotionWaittime,
	GetLocWaittime,
	RopeLengthOffset = 15,
	RopeEncoderCountsPerMillimeter = 16,
};


extern enum MsgIdx;
#pragma endregion

//读取点位文件
MatrixXd ReadThor_Joint(int robotType, int RobotAxis, int LocNum, string tPath, bool bSkipLine = false)
{
	if (RobotAxis <= 0 || LocNum <= 0)
		return MatrixXd();
	int ROWS = LocNum;
	//默认60
	int COLS = RobotAxis;
	double a = 0.0;
	MatrixXd data = MatrixXd::Zero(ROWS, COLS);

	//FILE *file = fopen(tPath.c_str(), "r");
	FILE *file;
	int err = fopen_s(&file, tPath.c_str(), "r");
	if (err != 0 || file == NULL)
		return MatrixXd();

	if (bSkipLine == true)
	{
		//跳过版本号那一行
		char buffer[256]; // 假设一行不会超过255个字符
		if (fgets(buffer, sizeof(buffer), file) == NULL)
		{
			fclose(file);
			return MatrixXd();
		}
	}

	for (int i = 0; i < ROWS; i++) {
		for (int j = 0; j < COLS; j++) {
			//fscanf(file, "%lf", &a);
			if (fscanf_s(file, "%lf", &a) != 1)
			{
				fclose(file);
				return MatrixXd();
			}
			data(i, j) = a;
		}
	}
	fclose(file);
	//data.col(2) += -VectorXd::Ones(60)*90;
	/*data *= 3.14159265358979323846 / 180;*/
	data *= PI / 180;

	if (robotType == 0) {
		for (int i = 0; i < ROWS; i++) data(i, 2) *= 180 / PI*0.001;
	}

	return data.transpose();
}

//判断文件是否存在
bool File_Exist(const std::string& name) {
	ifstream f(name.c_str());
	return f.good();
}

//从 ftp 下载文件
int DownLoad(CString IP, int port, CString userName, CString userPw)
{
	CInternetSession tSession((LPCTSTR)"Update Session");
	CFtpConnection* tFtpConnection = NULL;

	CString remotePath = "/media/flash";
	CString localPath = "./temp";
	CString remoteFile = "/media/flash/robot1.xml";
	CString localFile = "./temp/robot1.xml";

	try
	{
		tFtpConnection = tSession.GetFtpConnection(IP, userName, userPw, port);

		if (tFtpConnection == NULL) {
			_ftpError = ConnectFail;
			return -_ftpError;
		}

		bool b = tFtpConnection->SetCurrentDirectory(remotePath);
		if (!b) {
			_ftpError = GetCurrentPath;
			return -_ftpError;
		}

		CreateDirectory(localPath, NULL);

		WIN32_FIND_DATA FindFileData;
		HANDLE hFind;

		hFind = FindFirstFile(localFile, &FindFileData);

		if (hFind != INVALID_HANDLE_VALUE) {
			DeleteFile(localFile);
			FindClose(hFind);
		}

		b = tFtpConnection->GetFile(remoteFile, localFile);

		if (!b) {
			int err = GetLastError();

			tFtpConnection->Close();
			return -err;
		}

		tFtpConnection->Close();

		return 0;
	}
	catch (const std::exception&)
	{
		return -100;
	}

}

//通过 ftp 上传文件
int UpLoad(CString IP, int port, CString userName, CString userPw)
{
	CInternetSession tSession((LPCTSTR)"Update Session");
	CFtpConnection* tFtpConnection = NULL;

	CString remotePath = "/media/flash";
	CString localPath = "./temp";
	CString remoteFile = "/media/flash/robot1.xml";
	CString localFile = "./temp/robot1.xml";

	try
	{
		tFtpConnection = tSession.GetFtpConnection(IP, userName, userPw, port);

		if (tFtpConnection == NULL) {
			_ftpError = ConnectFail;
			return -_ftpError;
		}

		bool b = tFtpConnection->SetCurrentDirectory(remotePath);
		if (!b) {
			_ftpError = GetCurrentPath;
			return -_ftpError;
		}

		b = tFtpConnection->PutFile(localFile,remoteFile);

		if (!b) {
			int err = GetLastError();

			tFtpConnection->Close();
			return -err;
		}

		tFtpConnection->Close();

		return 0;
	}
	catch (const std::exception&)
	{
		return -100;
	}
}

//解析 xml 文件,用于读取机器人配置文件
int DecodeXmlFile()
{
	string tPath = "./temp/robot1.xml";

	fstream f(tPath);

	string idValue;
	string attributeValue;
	string xmlValue;
	bool ContinueRead = true;
	string readValue;
	idArray.clear();
	attributeArray.clear();
	xmlValueArray.clear();
	fileArray.clear();
	CString str;
	int count = -1;

	while (ContinueRead)
	{
		getline(f, readValue);
		count++;
		fileArray.push_back(readValue);
		int num = readValue.find(_headFlag);
		if (num >= 0)
		{
			idxArray.push_back(count);
			idValue = readValue.substr(readValue.find(_idFlag), readValue.find(_attributeFlag) - readValue.find(_idFlag));
			idValue = idValue.substr(idValue.find('"') + 1, idValue.length() - idValue.find('"') - 1);
			idValue = idValue.substr(0, idValue.find('"'));
			transform(idValue.begin(), idValue.end(), idValue.begin(), toupper);

			attributeValue = readValue.substr(readValue.find(_attributeFlag), readValue.find(_tailFlag) - readValue.find(_attributeFlag));
			attributeValue = attributeValue.substr(attributeValue.find('"') + 1, attributeValue.length() - attributeValue.find('"') - 1);
			attributeValue = attributeValue.substr(0, attributeValue.find('"'));

			xmlValue = readValue.substr(readValue.find(_headEndFlag) + 1, readValue.find(_tailFlag) - readValue.find(_headEndFlag) - 1);
			xmlValue.erase(0, xmlValue.find_first_not_of(""));
			xmlValue.erase(0, xmlValue.find_first_not_of("\t"));
			xmlValue.erase(xmlValue.find_last_not_of("") + 1);
			xmlValue.erase(xmlValue.find_last_not_of("\t") + 1);

			CString strTemp = CA2W((idValue + ',' + attributeValue + ',' + xmlValue).c_str());
			str.Format(_T("%s"), strTemp);
			idArray.push_back(idValue);
			attributeArray.push_back(attributeValue);
			xmlValueArray.push_back(xmlValue);
			continue;
		}
		
		num = readValue.find(_fileEndFlag);
		if (num >= 0) {
			break;
		}
	}

	return 0;
}

CString JointSystemIDNWriteCmdStr(CString IDNModule, CString IDNIndex, CString value, CString arrIndex = _T(""))
{
	CString sSend;
	if (arrIndex.IsEmpty()) {
		// 字符串为空
		sSend.Format(_T("System.IDNWrite \"%s\",\"%s\",\"%s\""), IDNModule, IDNIndex, value);
	}
	else
	{
		sSend.Format(_T("System.IDNWrite \"%s\",\"%s\",\"%s, %s\""), IDNModule, IDNIndex, arrIndex, value);
	}
	return sSend;
}

CString JointSystemIDNReadCmdStr(CString IDNModule, CString IDNIndex, CString arrIndex)
{
	CString sSend;
	sSend.Format(_T("System.IDNRead \"%s\",\"%s\",\"%s\""), IDNModule, IDNIndex, arrIndex);
	return sSend;
}

//寻找 xml 值,根据 IDN 去获取对应值
int FindXmlValue(CString targetIDN)
{
	try
	{
		CString tempStr;
		string sValue;

		for (int ii = 0; ii < idArray.size(); ii++)
		{
			tempStr = idArray[ii].c_str();
			if (tempStr == targetIDN) {
				return ii;
			}
		}


		return -1;
	}
	catch (const std::exception&)
	{
		return -1;
	}

}

//根据 IDN 替换 xml 中的值
int ReplaceXmlValue(CString targetIDN, CString IDNValue)
{
	try
	{
		string sValue;

		int idx = FindXmlValue(targetIDN);
		if (idx >= 0) {
			sValue = CT2A(IDNValue.GetBuffer(0));
			fileArray[idxArray[idx]] = fileArray[idxArray[idx]].replace(fileArray[idxArray[idx]].find(xmlValueArray[idx]), xmlValueArray[idx].length(), sValue);
		}
		
		return 0;
	}
	catch (const std::exception&)
	{
		return -1;
	}

}

//根据索引去获取解析出来的 xml 值
string GetXmlValue(int tIdx)
{
	try
	{
		return xmlValueArray[tIdx];
	}
	catch (const std::exception&)
	{
		return NULL;
	}

}

//写入 xml 文件
int WriteXml()
{
	try
	{
		string tPath = "./temp/robot1.xml";

		CString str;
		// TODO: Add your control notification handler code here
		ofstream out(tPath);
		string tResult;

		for each (string target in fileArray)
		{
			out << target;
			out << "\n";
		}

		return 0;
	}
	catch (const std::exception&)
	{
		return -1;
	}

}

//删除文件夹
bool DeleteDirectory(CString strDirName)
{
	CFileFind tempFind;

	BOOL IsFinded = tempFind.FindFile(strDirName + "//*.*");

	while (IsFinded)
	{
		IsFinded = tempFind.FindNextFile();
		if (!tempFind.IsDots())
		{
			char strFoundFileName[MAX_PATH];

			if (tempFind.IsDirectory())
			{
				DeleteDirectory(strDirName + '/' + tempFind.GetFileName());
			}
			else
			{
				DeleteFile(strDirName + '/' + tempFind.GetFileName());
			}
		}
	}

	tempFind.Close();

	if (!RemoveDirectory(strDirName))
	{
		return FALSE;
	}

	return TRUE;
}

//删除临时文件夹
int DeleteTemp() {
	try
	{
		CString tPath = "./temp/robot1.xml";
		DeleteDirectory(_T("./temp"));

		return 0;
	}
	catch (const std::exception&)
	{

		return -1;
	}
	

}

//初始化文件开头数组
int InitMsgArray()
{
	_msgHeader = new string[Error_After + 1];
	_msgHeader[StartTime] = "StartTime = ";
	_msgHeader[FinishTime] = "FinishTime = ";
	_msgHeader[A2_Before] = "Before Calib A2(mm) = ";
	_msgHeader[A3_Before] = "Before Calib A3(mm) = ";
	_msgHeader[D0_Before] = "Before Calib D0(mm) = ";
	_msgHeader[D3_Before] = "Before Calib D3(mm) = ";
	_msgHeader[D4_Before] = "Before Calib D4(mm) = ";
	_msgHeader[D5_Before] = "Before Calib D5(mm) = ";
	_msgHeader[J0_Before] = "Before Calib Joint0(cnts) = ";
	_msgHeader[J1_Before] = "Before Calib Joint1(cnts) = ";
	_msgHeader[J2_Before] = "Before Calib Joint2(cnts) = ";
	_msgHeader[J3_Before] = "Before Calib Joint3(cnts) = ";
	_msgHeader[J4_Before] = "Before Calib Joint4(cnts) = ";
	_msgHeader[J5_Before] = "Before Calib Joint5(cnts) = ";

	_msgHeader[A2_After] = "After Calib A2(mm) = ";
	_msgHeader[A3_After] = "After Calib A3(mm) = ";
	_msgHeader[D0_After] = "After Calib D0(mm) = ";
	_msgHeader[D3_After] = "After Calib D3(mm) = ";
	_msgHeader[D4_After] = "After Calib D4(mm) = ";
	_msgHeader[D5_After] = "After Calib D5(mm) = ";
	_msgHeader[J0_After] = "After Calib Joint0(cnts) = ";
	_msgHeader[J1_After] = "After Calib Joint1(cnts) = ";
	_msgHeader[J2_After] = "After Calib Joint2(cnts) = ";
	_msgHeader[J3_After] = "After Calib Joint3(cnts) = ";
	_msgHeader[J4_After] = "After Calib Joint4(cnts) = ";
	_msgHeader[J5_After] = "After Calib Joint5(cnts) = ";
	_msgHeader[J0_Offset] = "Calib Joint0 Offset(cnts) = ";
	_msgHeader[J1_Offset] = "Calib Joint1 Offset(cnts) = ";
	_msgHeader[J2_Offset] = "Calib Joint2 Offset(cnts) = ";
	_msgHeader[J3_Offset] = "Calib Joint3 Offset(cnts) = ";
	_msgHeader[J4_Offset] = "Calib Joint4 Offset(cnts) = ";
	_msgHeader[J5_Offset] = "Calib Joint5 Offset(cnts) = ";


	_msgHeader[Error_Before] = "Error before calibration(mm) = ";
	_msgHeader[Error_After] = "Error after calibration(mm) = ";
	_strInit = true;

	return 0;
}

//保存标定结果
int SaveCalibResult(string* sMessage) 
{
	if (!_strInit) {
		InitMsgArray();
	}
	CString tDir = "./Result";
	string tPath = "./Result/CalibResult_" + sMessage[FinishTime] + ".txt";

	string msgArray[Error_After + 1];

	CreateDirectory(tDir,NULL);

	ofstream out(tPath);
	string tResult;

	for (int ii = 0; ii <= Error_After; ii++)
	{
		msgArray[ii] = _msgHeader[ii] + sMessage[ii];
		out << msgArray[ii];
		out << "\n";
		if (ii == J5_Before || ii == FinishTime || ii == J5_After)
		{
			out << "\n";
			out << "\n";
		}
	}

	return 0;
}

//去除空格
void trim(string &s)
{
	int index = 0;
	if (!s.empty())
	{
		while ((index = s.find(' ', index)) != string::npos)
		{
			s.erase(index, 1);
		}
	}

}

//字符串分割
void SplitString(const string& s, vector<string>& v, const string& c)
{
	string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while (string::npos != pos2)
	{
		v.push_back(s.substr(pos1, pos2 - pos1));

		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if (pos1 != s.length())
		v.push_back(s.substr(pos1));
}

//参数分割,由于读取的配置文件参数用 ',' 分割
bool ParaSplit(const string& s, double* v, int expectedCount) {
	if (v == NULL || expectedCount <= 0)
		return false;
	vector<string> msgArray;
	SplitString(s, msgArray, ",");
	if (static_cast<int>(msgArray.size()) != expectedCount)
		return false;
	try
	{
		for (int i = 0; i < expectedCount; ++i)
			v[i] = stod(msgArray[i]);
	}
	catch (...)
	{
		return false;
	}
	return true;
}

//获取机器人列表
void GetRobotList(string path, vector<string>& files)
{
     //文件句柄  
     long   hFile = 0;
     //文件信息  
     struct _finddata_t fileinfo;
     string p;
     if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
     {
         do
         {
      //       //如果是目录,迭代之  
      //       //如果不是,加入列表  
      //       if ((fileinfo.attrib &  _A_SUBDIR))
      //       {
      //           if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					 //GetRobotList(p.assign(path).append("\\").append(fileinfo.name), files);
      //       }
      //       else
      //       {
                 files.push_back(fileinfo.name);
             //}
         }while (_findnext(hFile, &fileinfo) == 0);
         _findclose(hFile);
     }
 }

//解析配置文件参数
void DecodePara(string tMsg, RbtConfig &tPara) {
	if (tMsg == "")return;
	// UTF-8 configuration files may start with EF BB BF.  atoi() sees that
	// BOM before the leading "1" and returns zero, which silently skips the
	// Robot Type line.  Strip it here so the same parser works for BOM and
	// non-BOM files.
	if (tMsg.size() >= 3
		&& static_cast<unsigned char>(tMsg[0]) == 0xEF
		&& static_cast<unsigned char>(tMsg[1]) == 0xBB
		&& static_cast<unsigned char>(tMsg[2]) == 0xBF)
	{
		tMsg.erase(0, 3);
	}
	vector<string> msgArray;
	string tempValue;
	trim(tMsg);

	SplitString(tMsg, msgArray,",");
	if (msgArray.size() == 0)
		return;
	int tIdx = atoi(msgArray[0].c_str());
	if (tIdx > 0) {
		tempValue = tMsg.substr(tMsg.find('=') + 1);

		switch (tIdx)
		{
		case RbtType:
			tPara.RobotType = atoi(tempValue.c_str());
			break;
		case RbtAxis:
			tPara.RobotAxis = atoi(tempValue.c_str());
			break;
		case LocNum:
			tPara.CalLocationNumber = atoi(tempValue.c_str());
			break;
		case ToolOffset:
			tPara.ToolOffset = stod(tempValue.c_str()) / 1000;
			break;
		case RopeLengthOffset:
			tPara.RopeLengthOffset = stod(tempValue.c_str()) / 1000;
			break;
		case RopeEncoderCountsPerMillimeter:
			tPara.RopeEncoderCountsPerMillimeter = stod(tempValue.c_str());
			break;
		case a:
			if (tPara.RobotAxis <= 0) break;
			tPara.a = new double[tPara.RobotAxis]();
			if (!ParaSplit(tempValue, tPara.a, tPara.RobotAxis)) { delete[] tPara.a; tPara.a = nullptr; }
			break;
		case alpha:
			if (tPara.RobotAxis <= 0) break;
			tPara.alpha = new double[tPara.RobotAxis]();
			if (!ParaSplit(tempValue, tPara.alpha, tPara.RobotAxis)) { delete[] tPara.alpha; tPara.alpha = nullptr; break; }
			for (size_t i = 0; i < tPara.RobotAxis; i++)
			{
				tPara.alpha[i] *= PI / 180;
			}
			break;
		case d:
			if (tPara.RobotAxis <= 0) break;
			tPara.d = new double[tPara.RobotAxis]();
			if (!ParaSplit(tempValue, tPara.d, tPara.RobotAxis)) { delete[] tPara.d; tPara.d = nullptr; }
			break;
		case theta:
			if (tPara.RobotAxis <= 0) break;
			tPara.theta = new double[tPara.RobotAxis]();
			if (!ParaSplit(tempValue, tPara.theta, tPara.RobotAxis)) { delete[] tPara.theta; tPara.theta = nullptr; break; }
			for (size_t i = 0; i < tPara.RobotAxis; i++)
			{
				tPara.theta[i] *= PI / 180;
			}
			break;
		case beta:
			if (tPara.RobotAxis <= 0) break;
			tPara.beta = new double[tPara.RobotAxis]();
			if (!ParaSplit(tempValue, tPara.beta, tPara.RobotAxis)) { delete[] tPara.beta; tPara.beta = nullptr; break; }
			for (size_t i = 0; i < tPara.RobotAxis; i++)
			{
				tPara.beta[i] *= PI / 180;
			}
			break;
		case EncoderLineNum:
			if (tPara.RobotAxis <= 0) break;
			tPara.EncoderLineNum = new double[tPara.RobotAxis]();
			if (!ParaSplit(tempValue, tPara.EncoderLineNum, tPara.RobotAxis)) { delete[] tPara.EncoderLineNum; tPara.EncoderLineNum = nullptr; }
			break;
		case MoveDirection:
			if (tPara.RobotAxis <= 0) break;
			tPara.MoveDirection = new double[tPara.RobotAxis]();
			if (!ParaSplit(tempValue, tPara.MoveDirection, tPara.RobotAxis)) { delete[] tPara.MoveDirection; tPara.MoveDirection = nullptr; }
			break;
		case MotionWaittime:
			tPara.MotionWaittime = atoi(tempValue.c_str());
			break;
		case GetLocWaittime:
			tPara.GetLocWaittime = atoi(tempValue.c_str());
			break;
		default:
			break;
		}
	}
	

}

bool ReadConfigFile(string tPath, RbtConfig& Paras) {
	// This object is reused after robot selection changes. Release the previous
	// axis arrays before DecodePara allocates the new configuration arrays.
	Paras.ResetArrays();
	// Fail closed when a scalar line is absent or cannot be parsed.  Without
	// this reset a value from the previously selected robot could survive.
	Paras.RobotType = -1;
	Paras.RobotAxis = 0;
	Paras.CalLocationNumber = 0;
	Paras.ToolOffset = 0.0;
	// Preserve the historical HE3 fixture dead length when an old cfg omits
	// line 15. New HE3_GY configs contain the value explicitly.
	Paras.RopeLengthOffset = 0.0625;
	Paras.RopeEncoderCountsPerMillimeter = 284.94;
	Paras.MotionWaittime = 60000;
	Paras.GetLocWaittime = 8000;
	//FILE *file;
	vector<string> readMsg;
	//int err = fopen_s(&file, tPath.c_str(), "r");
	//char tread[520];
	/*for (int i = 0; i < sizeof(file); i++) {
		file(i) >> readMsg[i];
	}*/

	fstream f(tPath);
	if (!f.is_open())
		return false;

	string idValue;
	string attributeValue;
	string xmlValue;
	bool ContinueRead = true;
	string readValue;
	idArray.clear();
	attributeArray.clear();
	xmlValueArray.clear();
	fileArray.clear();
	CString str;
	int count = -1;

	while (getline(f, readValue))
	{
		count++;
		readMsg.push_back(readValue);
		try
		{
			DecodePara(readMsg[count], Paras);
		}
		catch (...)
		{
			Paras.ResetArrays();
			Paras.RobotAxis = 0;
			return false;
		}
	}
	const bool valid = Paras.RobotType >= 0 && Paras.RobotType <= 3
		&& Paras.RobotAxis > 0 && Paras.RobotAxis <= 16
		&& Paras.CalLocationNumber > 0 && Paras.CalLocationNumber <= 10000
		&& Paras.a != nullptr && Paras.alpha != nullptr && Paras.d != nullptr
		&& Paras.theta != nullptr && Paras.beta != nullptr
		&& Paras.EncoderLineNum != nullptr && Paras.MoveDirection != nullptr;
	if (!valid)
	{
		Paras.ResetArrays();
		Paras.RobotAxis = 0;
		Paras.CalLocationNumber = 0;
	}
	return valid;
}
