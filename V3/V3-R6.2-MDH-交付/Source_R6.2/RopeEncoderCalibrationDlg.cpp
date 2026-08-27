// RopeEncoderCalibrationDlg.cpp : 实现文件
//本软件通过使用 QKMLink 发送 Macro 指令控制机器人运动,由于雷神机器人没有 cell,所以外
//接了一个 Cell 通过 websocket 将拉线编码器的编码器值发送过来, 所以整体是用了两种
//通讯方式来实现拉线编码器的整个标定过程


#include "stdafx.h"
#include "RopeEncoderCalibration.h"
#include "RopeEncoderCalibrationDlg.h"
#include "afxdialogex.h"
#include "qkmlinklib_i.h"
#include <direct.h> 
#include <fstream>
#include <iostream>
#include "afxcmn.h"
#include "afxwin.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "CalibrationV3Analytic.h"
#include "FileOperation.h"
#include "websocket_endpoint.h"
#include <WinSock2.h>
#include "QKMLinkComm.h"
#include "RoughCalibrationDlg.h"
#include <string>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace std;
using namespace Eigen;
//websocket 的应用
using namespace kagula;
#define PI 3.1415926

#pragma region 全局变量
//软件版本
CString m_Version = _T(" HE3 V3 Analytic R6.2 Craig MDH 2026.08.25");
//文件的路径
string m_FilePath;
//查询编码器值的指令
string CmdStr = "STATUS_SAMPLE";
//点位文件的路径
string LocPath = "./Config/RobotLocation/";
//机器人 IP 字符串
CString m_RbtIPStr = _T("");
// Cell IP 字符串
string m_CellIPStr = "";
//运动点位的名称
string m_LocName = "RopeEncLoc";
//运动指令
string m_MoveCmd = "Move.Joint RopeEncLoc";
//总点位数
int m_TotalNum = 0;
//编码器零点值
int m_EncoderHomeValue;
//标定后计算得出的关节补偿值
char m_JointOffsetShow;
//websocket 的客户端
websocket_endpoint m_Client;
//机器人的轴关节数组
MatrixXd m_RbtJoint;
//计算结果数组
MatrixXd m_CalResult;
//雷神机器人用的是自启动的 cell 固件,返回信息和普通版本不一样
string m_CobotHeader = "\"enc0_aux_cnt\":";
//正常发布版本返回的报头
string m_NormalHeader = "\"servo0_s_st\":";
//获取编码器值的报头
string m_EncoderFeedbackHeader;
// login 指令
string m_CMDLogin = "System.Login 0";
//上电指令
string m_CMDPowerON = "Robot.PowerEnable 1,1";
//下电指令
string m_CMDPowerOff = "Robot.PowerEnable 1,0";
//机器人回零
string m_CMDHome = "Robot.Home 1";
//定义机器人点位变量指令
string m_CMDDefineLoc = "LocationJ ";
//系统 abrot 指令
string m_CMDAbroted = "System.Abort";
// 系统 start 指令
string m_CMDStart = "System.Start";
//系统 pause 指令
string m_CMDPause = "System.Pause 1";
//系统 continue 指令
string m_CMDContinue = "System.Continue 1";
//线程 abrot 指令
string m_CMDThreadAbroted = "Thread.Abort \"MacroThread\"";
// 线程 start 指令
string m_CMDThreadStart = "Thread.Start \"MacroThread\"";
//线程 pause 指令
string m_CMDThreadPause = "Thread.Pause \"MacroThread\"";
//线程 continue 指令
string m_CMDThreadContinue = "Thread.Continue \"MacroThread\"";
//读取固件版本 指令
string m_CMDSystemVersion = "System.Info Version,1";
int APIVersionType_1 = 1;
int APIVersionType_2 = 2;
int iAPIVersionType = APIVersionType_2;
//int iAPIVersionType = APIVersionType_1;
//设置机器人速度
string m_CMDRbtSpeed = "Robot.Speed 1,";
//定义 inrange 值
//string m_CMDProfile = "Profile Rope = 50,0,50,50,2,2,20";
string m_CMDProfile = "Profile Rope = 50,0,50,50,2,2";
//设置机器人 profile
string m_CMDProfileSet = "Profile.Set 1,Rope";
//获取机器人轴关节点
string m_CMDRobotWhereAngle = "Robot.WhereAngle 1";
//清除变量
string m_CMDClearVariables = "System.ClearVariables";
//清除变量
string m_CMDWaitForEOM = "Move.WaitForEOM";
// 赋予移动权限
string m_CMDRobotAttached = "Robot.Attached 1";
// 读取 IDN
//string m_CMDSystemIDNRead = "System.IDNRead";
// 写入 IDN
//string m_CMDSystemIDNWrite = "System.IDNWrite";
//索引字符串
CString m_IndexStr = _T("1.0.0");
//二轴杆长
//CString m_IDNA1 = _T("P-0-0513.0.50");
CString m_IDNA = _T("1101.10");
CString m_IDNA1_ArrIndex = _T("3");
//三轴杆长
//CString m_IDNA2 = _T("P-0-0513.0.51");
CString m_IDNA2_ArrIndex = _T("4");
//一轴偏置
//CString m_IDND1 = _T("P-0-0513.0.56");
CString m_IDND = _T("1101.11");
CString m_IDND1_ArrIndex = _T("1");
//四轴偏置
//CString m_IDND2 = _T("P-0-0513.0.59");
CString m_IDND2_ArrIndex = _T("4");
//五轴偏置
//CString m_IDND3 = _T("P-0-0513.0.60");
CString m_IDND3_ArrIndex = _T("5");
//六轴偏置
//CString m_IDND4 = _T("P-0-0513.0.61");
CString m_IDND4_ArrIndex = _T("6");
// 保存 IDN
CString m_IDNSave = _T("21.23");
// 保存 IDN 选项
CString m_IDNSaveOption = _T("save");
//标定结果
double m_CalibResult[12];
//是否连接判断位,后续的按钮状态取决于它
bool m_IsConnect = false;
//线程运行判断位
bool m_ThreadRun = false;
//保持连接线程的判断位
bool m_KeepConnect = false;
//运动线程
CWinThread *RunThread;
//保持连接线程
CWinThread *KeepConnectThread;
//回零线程
CWinThread *ReturnZeroThread;
//点位数组
CString *m_RobotHere;
//运动等待时间
int m_WaitTime = 60000;
//短等待时间
int m_ShortWaitTime = 5000;
//完成时间
string m_FinishTime;
//开始时间
string m_StartTime;
//报错判断,1为运动超时,2为编码器获取值超时,3为连接拉线编码器超时
int m_MoveTimeout = 1;
int m_EncoderTimeout = 2;
int m_EncoderConnectError = 3;
//错误信息
CString m_ErrorStr = _T("Error");
//运动超时报错信息
CString m_MoveTimeoutStr = _T("运动超时!");
//编码器超时报错信息
CString m_EncoderTimeoutStr = _T("读取编码器值超时!");
//获取机器人点位报错信息
CString m_WhereTimeoutStr = _T("获取机器人点位失败!");
//获取机器人点位报错信息
CString m_EncoderConnectErrorStr = _T("辅助编码器连接失败!");
//运动的事件
HANDLE m_MoveEvent = INVALID_HANDLE_VALUE;
//运动的事件
HANDLE m_WhereEvent = INVALID_HANDLE_VALUE;
//IDN 读写的事件
HANDLE m_IDNEvent = INVALID_HANDLE_VALUE;
//等待的目标 packetID
int TargetPacketID = 0;
//接受运动反馈的判断位
bool m_MoveReceive = false;
//获取当前位置判断位
bool m_Where = false;
//获取固件版本判断位
bool m_SystemVersion = false;
//获取 IDN 值
bool m_bGeneralWaitValue = false;
string m_sGeneralWaitValue = "";
//固件版本字符串
CString m_SystemVersionStr = _T("");
//点位的索引
int m_LocIdx = 0;
//机器人索引
int m_RobotIdx = 0;
//错误次数,用于判断接收编码器值错误次数
int m_ErrorTime = 0;
// R5: after the robot has stopped, use 20 independent rope samples instead
// of accepting one pair of identical integer counts.  The mean is used by
// calibration and the spread is saved for point-quality diagnosis.
const int m_R5SamplesPerPose = 20;
const DWORD m_R5SampleIntervalMs = 100;
double m_R5LastEncoderStdCounts = 0.0;
double m_R5LastEncoderMeanCounts = 0.0;
std::vector<double> m_R5EncoderMeanCounts;
std::vector<double> m_R5EncoderStdCounts;
bool m_R5CalibrationPassed = false;
// R6 in-memory feedback/replay state.  The feedback button never writes the
// robot; it only makes the last solved MDH the next calculation input.
std::vector<double> m_R6OutA, m_R6OutD, m_R6OutAlpha, m_R6OutQ0;
MatrixXd m_R6OriginalDH;
bool m_R6HaveResult = false;
bool m_R6HaveOriginalDH = false;
CalibrationV3Trace m_R6Trace = {};
double m_R61Q6SensitivityMmPerDeg = 0.0;
double m_R61Alpha6SensitivityMmPerDeg = 0.0;
double m_R61Q7SensitivityMmPerDeg = 0.0;
double m_R61Q6Alpha6Cosine = 0.0;
bool m_R61ObservabilityReady = false;
bool m_R61Q6CandidateActive = false;
//粗标界面的对象
RoughCalibrationDlg RoughCalibDlg = new RoughCalibrationDlg();
//websocket 通讯对象
websocket_endpoint endpoint;
//机器人轴数
int m_RobotAxis = 6;
//机器人类型
int m_RobotType = 3;
//读取配置文件路径
string _rbtConfigPath = "./Config/RobotType";
//配置文件后缀
string _configExtension = "-cfg.txt";
//当前旋转的机器人的配置参数
RbtConfig m_ThisRbt;
//当前机器人的 DH 参数
MatrixXd m_ThisRbtDH;
//解析错误最大次数
int _maxErrorTime = 20;
//保存标定参数索引
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
//机器人类型
enum RobotType {
	Scara,
	Delta,
	SixAxis,
	Cobot
};

//CString m_IDNAxis1 = _T("P-0-0513.0.168");
//CString m_IDNAxis2 = _T("P-0-0513.0.169");
//CString m_IDNAxis3 = _T("P-0-0513.0.170");
//CString m_IDNAxis4 = _T("P-0-0513.0.171");
//CString m_IDNAxis5 = _T("P-0-0513.0.172");
//CString m_IDNAxis6 = _T("P-0-0513.0.173");
CString m_IDNZeroEncoderValue = _T("1127.13");
CString m_IDNZeroEncoderValue1_ArrIndex = _T("1");
CString m_IDNZeroEncoderValue2_ArrIndex = _T("2");
CString m_IDNZeroEncoderValue3_ArrIndex = _T("3");
CString m_IDNZeroEncoderValue4_ArrIndex = _T("4");
CString m_IDNZeroEncoderValue5_ArrIndex = _T("5");
CString m_IDNZeroEncoderValue6_ArrIndex = _T("6");
#pragma endregion

// RopeEncoderCalibrationDlg 对话框

IMPLEMENT_DYNAMIC(RopeEncoderCalibrationDlg, CDialogEx)

RopeEncoderCalibrationDlg::RopeEncoderCalibrationDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_DIALOG1, pParent)
{

}

RopeEncoderCalibrationDlg::~RopeEncoderCalibrationDlg()
{
	
}

void RopeEncoderCalibrationDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_BUTTON1, m_ConnectBtn);
	DDX_Control(pDX, IDC_BUTTON2, m_StartBtn);
	DDX_Control(pDX, IDC_BUTTON3, m_ContinueBtn);
	DDX_Control(pDX, IDC_BUTTON4, m_PauseBtn);
	DDX_Control(pDX, IDC_BUTTON5, m_StopBtn);
	DDX_Control(pDX, IDC_BUTTON6, m_EncoderBtn);
	DDX_Control(pDX, IDC_BUTTON7, m_CalibBtn);
	DDX_Control(pDX, IDC_BUTTON8, m_SaveBtn);
	DDX_Control(pDX, IDC_EDIT2, m_A1);
	DDX_Control(pDX, IDC_EDIT3, m_A2);
	DDX_Control(pDX, IDC_EDIT4, m_D1);
	DDX_Control(pDX, IDC_EDIT5, m_D2);
	DDX_Control(pDX, IDC_EDIT6, m_D3);
	DDX_Control(pDX, IDC_EDIT7, m_D4);
	DDX_Control(pDX, IDC_EDIT8, m_JointShow);
	DDX_Control(pDX, IDC_EDIT1, m_EncoderHomeStr);
	DDX_Control(pDX, IDC_COMBO1, m_RbtTypeCombo);
	DDX_Control(pDX, IDC_COMBO2, m_CalibTypeCombo);
	DDX_Control(pDX, IDC_COMBO3, m_RbtSpeed);
	DDX_Control(pDX, IDC_IPADDRESS1, m_RbtIPAdr);
	DDX_Control(pDX, IDC_IPADDRESS2, m_CellIPAdr);
	DDX_Control(pDX, IDC_LIST1, m_List);
	DDX_Control(pDX, IDC_BUTTON9, m_CalibTypeBtn);
	DDX_Control(pDX, IDC_BUTTON10, m_LocationZero);
	DDX_Control(pDX, IDC_BUTTON11, m_DisconnectBtn);
	DDX_Control(pDX, IDC_EDIT10, m_ErrorBefore);
	DDX_Control(pDX, IDC_EDIT11, m_ErrorAfter);
}

BEGIN_MESSAGE_MAP(RopeEncoderCalibrationDlg, CDialogEx)
	ON_BN_CLICKED(IDC_BUTTON1, &RopeEncoderCalibrationDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &RopeEncoderCalibrationDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &RopeEncoderCalibrationDlg::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_BUTTON4, &RopeEncoderCalibrationDlg::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_BUTTON5, &RopeEncoderCalibrationDlg::OnBnClickedButton5)
	ON_BN_CLICKED(IDC_BUTTON6, &RopeEncoderCalibrationDlg::OnBnClickedButton6)
	ON_BN_CLICKED(IDC_BUTTON7, &RopeEncoderCalibrationDlg::OnBnClickedButton7)
	ON_BN_CLICKED(IDC_BUTTON8, &RopeEncoderCalibrationDlg::OnBnClickedButton8)
	ON_BN_CLICKED(IDC_BUTTON9, &RopeEncoderCalibrationDlg::OnBnClickedButton9)
	ON_BN_CLICKED(IDC_BUTTON10, &RopeEncoderCalibrationDlg::OnBnClickedButton10)
	ON_BN_CLICKED(IDC_BUTTON11, &RopeEncoderCalibrationDlg::OnBnClickedButton11)
	ON_BN_CLICKED(IDC_R6_FEEDBACK, &RopeEncoderCalibrationDlg::OnBnClickedR6Feedback)
	ON_BN_CLICKED(IDC_R6_RESTORE, &RopeEncoderCalibrationDlg::OnBnClickedR6Restore)
	ON_CBN_SELCHANGE(IDC_COMBO1, &RopeEncoderCalibrationDlg::OnCbnSelchangeCombo1)
	ON_MESSAGE(WM_DISPLAY_CHANGE, &RopeEncoderCalibrationDlg::OnDisplayChange)
	ON_MESSAGE(WM_ROBOT_MOVE, &RopeEncoderCalibrationDlg::OnRobotMove)
	ON_MESSAGE(WM_ENABLE_BTN, &RopeEncoderCalibrationDlg::OnEnablebtn)
	ON_MESSAGE(WM_SAVE_JOINT, &RopeEncoderCalibrationDlg::SaveRobotJoint)
	ON_MESSAGE(WM_GET_ROBOT_JOINT, &RopeEncoderCalibrationDlg::GetRobotWhereAngle)
	ON_MESSAGE(WM_CLOSE, &RopeEncoderCalibrationDlg::OnClosing)
	ON_MESSAGE(WM_TIME_OUT, &RopeEncoderCalibrationDlg::MessageBoxShow)
	ON_MESSAGE(WM_BUTTON_ENABLE, &RopeEncoderCalibrationDlg::StartButtonEnable)
	ON_MESSAGE(WM_WRITE_LOG, &RopeEncoderCalibrationDlg::ReceiveLog)
	ON_MESSAGE(WM_GET_ENCODER, &RoughCalibrationDlg::OnEncoderValueShow)
	ON_MESSAGE(WM_MOVE_ZERO, &RopeEncoderCalibrationDlg::OnMoveZero)
	ON_CBN_SELCHANGE(IDC_COMBO2, &RopeEncoderCalibrationDlg::OnSelchangeCombo2)
	ON_EN_CHANGE(IDC_EDIT1, &RopeEncoderCalibrationDlg::OnEnChangeEdit1)
END_MESSAGE_MAP()

//初始化
BOOL RopeEncoderCalibrationDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	CString windowTitle;

	CString tempCstring;
	AfxGetMainWnd()->GetWindowText(windowTitle);
	windowTitle += m_Version;
	AfxGetMainWnd()->SetWindowText(windowTitle);

	// R6 diagnostic panel: show every accepted MDH iteration and allow a safe
	// in-memory feedback/recalculation without touching controller parameters.
	CRect originalClient;
	GetClientRect(&originalClient);
	const int oldWidth = originalClient.Width();
	CRect windowRect;
	GetWindowRect(&windowRect);
	SetWindowPos(NULL, 0, 0, windowRect.Width() + 560, windowRect.Height(),
		SWP_NOMOVE | SWP_NOZORDER);
	m_R6TraceEdit.Create(WS_CHILD | WS_VISIBLE | WS_BORDER | WS_VSCROLL |
		ES_MULTILINE | ES_AUTOVSCROLL | ES_READONLY,
		CRect(oldWidth + 12, 20, oldWidth + 545, originalClient.Height() - 55),
		this, IDC_R6_TRACE);
	m_R6TraceEdit.SetWindowTextW(_T("R6.2迭代记录将在计算后显示。\r\n"
		"q6先按解析雅可比自动判断：可见才参与辨识；q7解析敏感度应为0。"));
	m_R6FeedbackBtn.Create(_T("将本轮MDH作为输入复算"),
		WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
		CRect(oldWidth + 12, originalClient.Height() - 45,
			oldWidth + 280, originalClient.Height() - 15), this, IDC_R6_FEEDBACK);
	m_R6RestoreBtn.Create(_T("恢复本次原始MDH"),
		WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
		CRect(oldWidth + 292, originalClient.Height() - 45,
			oldWidth + 545, originalClient.Height() - 15), this, IDC_R6_RESTORE);
	m_R6FeedbackBtn.EnableWindow(FALSE);
	m_R6RestoreBtn.EnableWindow(FALSE);

	//设置初始 IP
	m_RbtIPAdr.SetAddress(192, 168, 10, 120);
	m_CellIPAdr.SetAddress(192, 168, 10, 165);

	m_CalibTypeCombo.AddString(_T("拉线编码器"));
	m_CalibTypeCombo.AddString(_T("粗标"));

	vector<string> filesList;
	GetRobotList(_rbtConfigPath, filesList);
	vector<string> robotList;
	for (int i = 0; i < filesList.size(); i++) {
		int tidx = filesList[i].find(_configExtension);
		if (tidx >= 0) {
			robotList.push_back(filesList[i].substr(0,tidx));
		}
	}

	for (int i = 0; i < robotList.size(); i++) {
		tempCstring = robotList[i].c_str();
		OutputDebugStringA(robotList[i].c_str());
		m_RbtTypeCombo.InsertString(i,tempCstring);
	}

	CRect rect;
	CString str;
	//获取编程语言列表视图控件的位置和大小   
	m_List.GetClientRect(&rect);
	m_List.SetExtendedStyle(m_List.GetExtendedStyle() | LVS_EX_FULLROWSELECT);
	m_List.InsertColumn(0, _T("序号"), LVCFMT_CENTER, rect.Width() / 10, 0);
	m_List.InsertColumn(1, _T("机器人轴关节点位(°)"), LVCFMT_CENTER, rect.Width() / 2, 1);
	m_List.InsertColumn(2, _T("距离(mm)"), LVCFMT_CENTER, rect.Width() * 2 / 5, 2);

	m_RbtTypeCombo.SetCurSel(0);
	m_CalibTypeCombo.SetCurSel(1);
	
	OnCbnSelchangeCombo1();

	if (m_List.GetItemCount() == 0) {
		GetDlgItem(IDC_BUTTON1)->EnableWindow(false);
	}

	m_TotalNum = m_ThisRbt.CalLocationNumber;
	m_RobotAxis = m_ThisRbt.RobotAxis;
	m_RobotType = m_ThisRbt.RobotType;

	m_RobotHere = new CString[m_TotalNum];

	//通讯初始化
	QKMLinkInit();	
	endpoint.init();

	//增加速度选择下拉表
	for (int ii = 0; ii < 8; ii++) {
		str.Format(_T("%d"), (ii + 2) * 10);
		m_RbtSpeed.AddString(str);
	}

	m_RbtSpeed.SetCurSel(0);

	m_MoveEvent = CreateEvent(NULL, TRUE, TRUE, L"eventa");
	ResetEvent(m_MoveEvent);
	m_WhereEvent = CreateEvent(NULL, TRUE, TRUE, L"eventb");
	ResetEvent(m_WhereEvent);
	m_IDNEvent = CreateEvent(NULL, TRUE, TRUE, L"eventidn");
	ResetEvent(m_IDNEvent);

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

#pragma region 全局函数
// RopeEncoderCalibrationDlg 消息处理程序
//判断是否为数字
bool isNum(string str) {
	stringstream ss(str);
	double d;
	char c;
	if (!(ss >> d)) {
		return false;
	}
	if (ss >> c) {
		return false;
	}
	return true;
}

//解析 websocket 返回的报文
int DecodeEncoder(string message)
{
	string valueStr;
	if (message.find(m_EncoderFeedbackHeader) != string::npos)
	{
		valueStr = message.substr(message.find(m_EncoderFeedbackHeader), message.length());
		valueStr = valueStr.substr(m_EncoderFeedbackHeader.length(), valueStr.length());
		valueStr = valueStr.substr(valueStr.find("\"") + 1, valueStr.length() - 3);

		int indexA = valueStr.find("\"");
		if (indexA >= 0)
		{
			valueStr = valueStr.substr(0, indexA);
		}

		if (isNum(valueStr))
		{
			return stoi(valueStr);
		}
	}

	m_Client.Send(CmdStr);
	Sleep(200);
	string result = m_Client.Receive();
	m_ErrorTime++;
	m_Client.Clear();
	if (m_ErrorTime > _maxErrorTime) {
		m_ErrorTime = 0;
		return -1;
	}
	return DecodeEncoder(result);
}

//获取编码器的值
int GetEncoderValue()
{
	if (m_Client.State() <= 0) {
		m_Client.Connect("ws://" + m_CellIPStr);
	}

	std::vector<double> samples;
	samples.reserve(m_R5SamplesPerPose);
	for (int sample = 0; sample < m_R5SamplesPerPose; ++sample)
	{
		m_Client.Send(CmdStr);
		Sleep(m_R5SampleIntervalMs);
		const string receiveStr = m_Client.Receive();
		const int value = DecodeEncoder(receiveStr);
		m_Client.Clear();
		if (value == -1)
		{
			m_R5LastEncoderMeanCounts = 0.0;
			m_R5LastEncoderStdCounts = 0.0;
			return -1;
		}
		samples.push_back(static_cast<double>(value));
	}

	double mean = 0.0;
	for (size_t i = 0; i < samples.size(); ++i)
		mean += samples[i];
	mean /= static_cast<double>(samples.size());

	double variance = 0.0;
	for (size_t i = 0; i < samples.size(); ++i)
	{
		const double deviation = samples[i] - mean;
		variance += deviation * deviation;
	}
	variance /= static_cast<double>(samples.size());
	m_R5LastEncoderMeanCounts = mean;
	m_R5LastEncoderStdCounts = sqrt(variance);
	return static_cast<int>(floor(mean + 0.5));
}

//保存编码器数据
int SaveData(CString *tList, string rbtType, string time)
{
	try
	{
		CString str;
		fstream _file;

		_file.open("./data");
		if (!_file) {
			_mkdir("./data");
		}

		FILE *fp;
		string path = "./data/" + rbtType + "_Data_" + time + ".txt";

		fopen_s(&fp, path.c_str(), "w");
		fclose(fp);
		// TODO: Add your control notification handler code here
		ifstream in(path);
		ofstream out(path);
		string tResult;
		for (int ii = 0; ii < m_TotalNum; ii++) {
			str = tList[ii];
			tResult = CT2A(str.GetBuffer(0));
			out << tResult;
			out << "\n";
		}
		return 0;
	}
	catch (const std::exception&)
	{
		return -1;
	}

}

// R5 point-quality log.  One encoder count is converted with the unchanged
// V1 scale 284.94 counts/mm so the standard deviation can be read directly.
int SaveR5SampleStats(const string& rbtType, const string& time)
{
	try
	{
		_mkdir("./data");
		const string path = "./data/" + rbtType + "_R5SampleStats_" + time + ".csv";
		ofstream out(path, ios::out | ios::trunc);
		if (!out)
			return -1;

		out << "point,sample_count,mean_counts,std_counts,std_mm\n";
		out << setprecision(12);
		for (int point = 0; point < m_TotalNum; ++point)
		{
			const double meanCounts = (point < static_cast<int>(m_R5EncoderMeanCounts.size()))
				? m_R5EncoderMeanCounts[static_cast<size_t>(point)] : 0.0;
			const double stdCounts = (point < static_cast<int>(m_R5EncoderStdCounts.size()))
				? m_R5EncoderStdCounts[static_cast<size_t>(point)] : 0.0;
			out << (point + 1) << ',' << m_R5SamplesPerPose << ','
				<< meanCounts << ',' << stdCounts << ',' << (stdCounts / 284.94) << '\n';
		}
		return 0;
	}
	catch (const std::exception&)
	{
		return -1;
	}
}

//保存机器人点位数据
int SaveJoint(string rbtType, string time)
{
	try
	{
		fstream _file;

		_file.open("./Joint");
		if (!_file) {
			_mkdir("./Joint");
		}

		FILE *fp;
		string path = "./Joint/" + rbtType + "_Joint_" + time + ".txt";

		fopen_s(&fp, path.c_str(), "w");
		fclose(fp);

		ifstream in(path);
		ofstream out(path);
		string tResult;
		for (int ii = 0; ii < m_TotalNum; ii++) {
			tResult = CT2A(m_RobotHere[ii].GetBuffer(0));
			out << tResult;
			out << "\n";
		}

		out.close();
		return 0;
	}
	catch (const std::exception&)
	{
		return -1;
	}

}

//将轴值转为字符串
CString TurnDoubleToLoc(double *dValue) {

	try
	{
		CString csValue, tempValue;

		for (int ii = 0; ii < m_RobotAxis; ii++) {
			tempValue.Format(_T("%lf,"), dValue[ii]);
			csValue += tempValue;
		}

		csValue = csValue.Left(csValue.GetLength() - 1);

		return csValue;
	}
	catch (const std::exception&)
	{
		return NULL;
	}
}
#pragma endregion

#pragma region 界面按钮操作
//连接按钮,连接 QKMLink 和 websocket
void RopeEncoderCalibrationDlg::OnBnClickedButton1()
{
	// TODO: Add your control notification handler code here
	CString strIP;
	BYTE IP0, IP1, IP2, IP3;
	m_RbtIPAdr.GetAddress(IP0, IP1, IP2, IP3);
	strIP.Format(_T("%d.%d.%d.%d"), IP0, IP1, IP2, IP3);
	m_RbtIPStr = strIP;

	m_CellIPAdr.GetAddress(IP0, IP1, IP2, IP3);
	strIP.Format(_T("%d.%d.%d.%d"), IP0, IP1, IP2, IP3);
	m_CellIPStr = CT2A(strIP.GetBuffer(0));

	// TODO: Add your control notification handler code here
	//websocket 只要地址是对的,就会认为连接成功;所以需要发送指令来确认是否真正的连接成功,但是发送不能太快,不然会造成状态错乱
	int stt = endpoint.Connect("ws://" + m_CellIPStr);

	m_IsConnect = true;
	m_Client = endpoint;

	m_IsConnect = QKMLinkConnect(m_RbtIPStr, this);

	if (!m_IsConnect)
	{
		m_Client.Close();
		MessageBox(_T("连接机器人失败!"));
		return;
	}

	m_ConnectBtn.EnableWindow(!m_IsConnect);
	m_DisconnectBtn.EnableWindow(m_IsConnect);
	m_CalibTypeCombo.EnableWindow(m_IsConnect);
	m_RbtTypeCombo.EnableWindow(m_IsConnect);
	m_EncoderBtn.EnableWindow(m_IsConnect);
	m_LocationZero.EnableWindow(m_IsConnect);
	m_CalibTypeBtn.EnableWindow(m_IsConnect);
	m_RbtSpeed.EnableWindow(m_IsConnect);

	QKMLinkEventReset();
	SendInit();
	QKMLinkEventReset();
	m_KeepConnect = true;
	KeepConnectThread = AfxBeginThread(RopeEncoderCalibrationDlg::KeepConnecting, (LPVOID)this);
	
	m_EncoderHomeStr.EnableWindow(true);
	m_RbtTypeCombo.EnableWindow(false);
}

//开始按钮,开始进行标定动作
void RopeEncoderCalibrationDlg::OnBnClickedButton2()
{
	// TODO: Add your control notification handler code here
	LPVOID pParm = this;
	m_ThreadRun = false;
	CString sSend;

	SYSTEMTIME st;
	GetLocalTime(&st);
	m_StartTime = std::to_string(st.wYear) + '_' + std::to_string(st.wMonth) + '_' + std::to_string(st.wDay) + '_' + std::to_string(st.wHour) + '.' + std::to_string(st.wMinute);

	if (iAPIVersionType == APIVersionType_1)
	{
		sSend = (CString)m_CMDAbroted.c_str();
		SendCmd(sSend);

		sSend = (CString)m_CMDStart.c_str();
		SendCmd(sSend);

		sSend = (CString)m_CMDContinue.c_str();
		SendCmd(sSend);
	}
	else
	{
		sSend = (CString)m_CMDThreadAbroted.c_str();
		SendCmd(sSend);

		sSend = (CString)m_CMDThreadStart.c_str();
		SendCmd(sSend);

		sSend = (CString)m_CMDRobotAttached.c_str();
		SendCmd(sSend);
	}

	sSend = (CString)m_CMDPowerON.c_str();
	SendCmd(sSend);

	m_RbtSpeed.GetWindowTextW(sSend);
	sSend = (CString)(m_CMDRbtSpeed).c_str() + sSend;
	SendCmd(sSend);

	m_LocIdx = 0;
	m_R5CalibrationPassed = false;
	m_R5EncoderMeanCounts.assign(static_cast<size_t>(m_TotalNum), 0.0);
	m_R5EncoderStdCounts.assign(static_cast<size_t>(m_TotalNum), 0.0);
	//上电延迟
	Sleep(2000);
	CString str;
	m_EncoderHomeStr.GetWindowTextW(str);
	m_EncoderHomeValue = _ttoi(str);
	for (int ii = 0; ii < m_List.GetItemCount(); ii++) {
		m_List.SetItemText(ii, 2, NULL);
		m_List.SetItemState(ii, 0, LVIS_SELECTED | LVIS_FOCUSED);
	}
	RunThread = NULL;
	m_ThreadRun = true;

	SuspendThread(KeepConnectThread->m_hThread);
	RunThread = AfxBeginThread(RopeEncoderCalibrationDlg::MyThreadFunction, (LPVOID)this);
	m_EncoderHomeStr.EnableWindow(!m_IsConnect);
	m_EncoderBtn.EnableWindow(!m_IsConnect);
	m_StartBtn.EnableWindow(!m_IsConnect);
	m_StopBtn.EnableWindow(m_IsConnect);
	m_PauseBtn.EnableWindow(m_IsConnect);
	m_LocationZero.EnableWindow(!m_IsConnect);
	m_CalibTypeBtn.EnableWindow(!m_IsConnect);
	m_DisconnectBtn.EnableWindow(!m_IsConnect);
	m_SaveBtn.EnableWindow(!m_IsConnect);
	m_CalibBtn.EnableWindow(!m_IsConnect);
	m_RbtSpeed.EnableWindow(!m_IsConnect);
	m_CalibTypeCombo.EnableWindow(!m_IsConnect);
}

//继续按钮,在暂停后使用
void RopeEncoderCalibrationDlg::OnBnClickedButton3()
{
	// TODO: Add your control notification handler code here
	CString sSend;
	if (iAPIVersionType == APIVersionType_1)
	{
		sSend = (CString)m_CMDAbroted.c_str();
		SendCmd(sSend);

		sSend = (CString)m_CMDStart.c_str();
		SendCmd(sSend);

		sSend = (CString)m_CMDContinue.c_str();
		SendCmd(sSend);
	}
	else
	{
		sSend = (CString)m_CMDThreadAbroted.c_str();
		SendCmd(sSend);

		sSend = (CString)m_CMDThreadStart.c_str();
		SendCmd(sSend);

		sSend = (CString)m_CMDRobotAttached.c_str();
		SendCmd(sSend);
	}

	sSend = (CString)m_CMDPowerON.c_str();
	SendCmd(sSend);

	m_RbtSpeed.GetWindowTextW(sSend);
	sSend = (CString)(m_CMDRbtSpeed).c_str() + sSend;
	SendCmd(sSend);
	m_ThreadRun = true;
	QKMLinkEventReset();
	SuspendThread(KeepConnectThread->m_hThread);
	RunThread = AfxBeginThread(RopeEncoderCalibrationDlg::MyThreadFunction, (LPVOID)this);

	m_ContinueBtn.EnableWindow(!m_IsConnect);
	m_EncoderBtn.EnableWindow(!m_IsConnect);
	m_StartBtn.EnableWindow(!m_IsConnect);
	m_StopBtn.EnableWindow(m_IsConnect);
	m_PauseBtn.EnableWindow(m_IsConnect);
	m_LocationZero.EnableWindow(!m_IsConnect);
	m_CalibTypeBtn.EnableWindow(!m_IsConnect);
	m_DisconnectBtn.EnableWindow(!m_IsConnect);
	m_SaveBtn.EnableWindow(!m_IsConnect);
	m_CalibBtn.EnableWindow(!m_IsConnect);
	m_RbtSpeed.EnableWindow(!m_IsConnect);
}

//暂停按钮(暂时停止使用)
void RopeEncoderCalibrationDlg::OnBnClickedButton4()
{
	SuspendThread(RunThread->m_hThread);
	CString str;
	if (iAPIVersionType == APIVersionType_1)
	{
		str = m_CMDPause.c_str();
		SendCmd(str);
	}
	else
	{
		str = m_CMDThreadPause.c_str();
		SendCmd(str);
	}
	m_ContinueBtn.EnableWindow(m_IsConnect);
	m_PauseBtn.EnableWindow(!m_IsConnect);
}

//停止按钮
void RopeEncoderCalibrationDlg::OnBnClickedButton5()
{
	// TODO: 在此添加控件通知处理程序代码
	m_ThreadRun = false;
	CString str;
	if (iAPIVersionType == APIVersionType_1)
	{
		str = m_CMDAbroted.c_str();
		SendCmd(str);
	}
	else
	{
		str = m_CMDThreadAbroted.c_str();
		SendCmd(str);
	}
	QKMLinkEventReset();
	ResumeThread(RunThread->m_hThread);
	TerminateThread(RunThread, 0);
	ResumeThread(KeepConnectThread->m_hThread);
	m_ContinueBtn.EnableWindow(!m_IsConnect);
	m_PauseBtn.EnableWindow(!m_IsConnect);
	m_DisconnectBtn.EnableWindow(m_IsConnect);
	m_StopBtn.EnableWindow(!m_IsConnect);
	m_LocationZero.EnableWindow(m_IsConnect);
}

//获取编码器的值,用于获取零点值
void RopeEncoderCalibrationDlg::OnBnClickedButton6()
{
	// TODO: 在此添加控件通知处理程序代码
	string RecieveStr;
	CString str;
	m_Client.Send(CmdStr);
	Sleep(300);
	RecieveStr = m_Client.Receive();

	m_EncoderHomeValue = DecodeEncoder(RecieveStr);
	str.Format(_T("%d"), m_EncoderHomeValue);
	m_EncoderHomeStr.SetWindowTextW(str);
	m_StartBtn.EnableWindow(m_IsConnect);
}

// 辅助函数：将 double 转换为 CString 并追加到输出
void AppendDouble(CString& str, double val, const TCHAR* suffix = _T("\t"))
{
	CString tmp;
	tmp.Format(_T("%.6lf%s"), val, suffix);
	str += tmp;
}

// V3 adapter for the original V1 GUI.
// The GUI keeps V1's measurement convention and 60-point acquisition flow;
// only the unsafe Eigen-across-DLL calibration call is replaced here.
static int RunCalibrationV3ForGui(
	int axis,
	int sampleCount,
	const MatrixXd& dh,
	const MatrixXd& jointByAxis,
	const VectorXd& ropeWithFixedOffsetM,
	double toolOffsetM,
	double* calibResult12,
	double* residualBeforeM,
	double* residualAfterM,
	CalibrationV3Report* report)
{
	if (axis < 5 || sampleCount < 4 || dh.cols() != axis ||
		jointByAxis.rows() != axis || jointByAxis.cols() != sampleCount ||
		ropeWithFixedOffsetM.size() != sampleCount || calibResult12 == NULL ||
		residualBeforeM == NULL || residualAfterM == NULL || report == NULL)
	{
		return CALV3_ERR_BAD_ARGUMENT;
	}

	const double kRopeFixedOffsetM = 0.043 + 0.0195; // exact V1 convention
	const double kRadToDeg = 180.0 / PI;

	// R5 100-point layout:
	//   1..60  original production points
	//   61..69 J7-only fixture diagnostic
	//   70..78 repeated 18->19->20 diagnostic sequences
	//   79..100 independent J2..J5 excitation points
	// Diagnostic repetitions are deliberately excluded from least squares so
	// they do not overweight one pose. They remain in the saved logs.
	std::vector<int> calibrationIndices;
	calibrationIndices.reserve(static_cast<size_t>(sampleCount));
	for (int sample = 0; sample < sampleCount; ++sample)
	{
		const bool r5DiagnosticOnly = (axis == 7 && sampleCount >= 100 && sample >= 60 && sample < 78);
		if (!r5DiagnosticOnly)
			calibrationIndices.push_back(sample);
	}
	const int calibrationSampleCount = static_cast<int>(calibrationIndices.size());

	std::vector<double> a(axis), d(axis), alpha(axis), thetaFixed(axis), beta(axis);
	std::vector<double> q0Initial(axis, 0.0);
	for (int joint = 0; joint < axis; ++joint)
	{
		a[joint] = dh(0, joint);
		d[joint] = dh(1, joint); // terminal d already includes ToolOffset
		alpha[joint] = dh(2, joint);
		thetaFixed[joint] = dh(3, joint);
		beta[joint] = dh(4, joint);
	}

	// V3 ABI is sample-major: [point0 J1..JN, point1 J1..JN, ...].
	std::vector<double> jointSampleMajor(calibrationSampleCount * axis);
	for (int calibrationSample = 0; calibrationSample < calibrationSampleCount; ++calibrationSample)
	{
		const int sample = calibrationIndices[static_cast<size_t>(calibrationSample)];
		for (int joint = 0; joint < axis; ++joint)
		{
			jointSampleMajor[calibrationSample * axis + joint] = jointByAxis(joint, sample);
		}
	}

	// The V1 GUI has already added 62.5 mm for display/calculation. Remove it
	// before calling V3, then pass the same fixed offset explicitly exactly once.
	std::vector<double> ropeRawM(calibrationSampleCount);
	for (int calibrationSample = 0; calibrationSample < calibrationSampleCount; ++calibrationSample)
	{
		const int sample = calibrationIndices[static_cast<size_t>(calibrationSample)];
		ropeRawM[calibrationSample] = ropeWithFixedOffsetM(sample) - kRopeFixedOffsetM;
	}

	const double fixtureLocalXYZ[3] = { 0.0, 0.0, 0.0 };

	// R6.2 HE3 profile, aligned with the eight laser-calibration files:
	// Controller-native Craig MDH is used here.  The alpha row is therefore
	// [0, alpha1, ..., alpha6], not the legacy SDH-style
	// [alpha1, ..., alpha6, 0].  d2..d5, MDH alpha1..alpha6 and q2..q5 are
	// normally visible. q6 is first
	// treated as a candidate, then admitted only if its exact analytic point
	// Jacobian is non-zero for the acquired poses. This is evidence-driven,
	// not a hard-coded assumption. The 93 mm structure + 49 mm
	// axial tool is mechanical and is not allowed to become a fictitious d7.
	// q1 is an anchor-frame gauge. q7 is structurally unobservable because the
	// measured point lies on its axis.
	std::vector<int> active(4 * axis, 0);
	for (int joint = 1; joint <= 4 && joint < axis; ++joint)
		active[axis + joint] = 1;       // d2..d5
	for (int joint = 1; joint <= 6 && joint < axis; ++joint)
		active[2 * axis + joint] = 1;   // MDH alpha1..alpha6 (array slots 2..7)
	for (int joint = 1; joint <= 4 && joint < axis; ++joint)
		active[3 * axis + joint] = 1;   // q2..q5
	m_R61Q6CandidateActive = false;
	if (axis >= 7)
	{
		double q6PointNorm2 = 0.0;
		std::vector<double> candidateJacobian(3 * 4 * axis, 0.0);
		for (int calibrationSample = 0; calibrationSample < calibrationSampleCount; ++calibrationSample)
		{
			double point[3] = {};
			const double* pose = jointSampleMajor.data() + calibrationSample * axis;
			if (CalibrationV3EvaluatePointAndJacobian(axis, a.data(), d.data(), alpha.data(),
				thetaFixed.data(), beta.data(), q0Initial.data(), pose, fixtureLocalXYZ,
				point, candidateJacobian.data()) != CALV3_OK)
				continue;
			for (int row = 0; row < 3; ++row)
			{
				const double value = candidateJacobian[row * 4 * axis + 3 * axis + 5];
				q6PointNorm2 += value * value;
			}
		}
		m_R61Q6CandidateActive = q6PointNorm2 > 1e-20;
		if (m_R61Q6CandidateActive)
			active[3 * axis + 5] = 1;
	}
	std::vector<double> outA(axis), outD(axis), outAlpha(axis), outQ0(axis);
	std::vector<double> delta(4 * axis);
	std::vector<double> calibrationResidualBefore(calibrationSampleCount, 0.0);
	std::vector<double> calibrationResidualAfter(calibrationSampleCount, 0.0);

	const int status = CalibrationV3Analytic(
		axis, calibrationSampleCount,
		a.data(), d.data(), alpha.data(), thetaFixed.data(), beta.data(), q0Initial.data(),
		jointSampleMajor.data(), ropeRawM.data(), kRopeFixedOffsetM,
		fixtureLocalXYZ, active.data(),
		30, 1e-9, 0, 1e8,
		outA.data(), outD.data(), outAlpha.data(), outQ0.data(), delta.data(),
		calibrationResidualBefore.data(), calibrationResidualAfter.data(), report);

	const double missingResidual = std::numeric_limits<double>::quiet_NaN();
	for (int sample = 0; sample < sampleCount; ++sample)
	{
		residualBeforeM[sample] = missingResidual;
		residualAfterM[sample] = missingResidual;
	}
	for (int calibrationSample = 0; calibrationSample < calibrationSampleCount; ++calibrationSample)
	{
		const int sample = calibrationIndices[static_cast<size_t>(calibrationSample)];
		residualBeforeM[sample] = calibrationResidualBefore[static_cast<size_t>(calibrationSample)];
		residualAfterM[sample] = calibrationResidualAfter[static_cast<size_t>(calibrationSample)];
	}

	if (status != CALV3_OK && status != CALV3_STOP_NO_IMPROVEMENT)
	{
		return status;
	}

	// R6.2 observability evidence from the exact analytic point Jacobian.
	// We project physical alpha6/q6/q7 point derivatives onto the rope direction at
	// every calibration pose. RMS sensitivity says whether a column is visible;
	// cosine close to +/-1 says q6 and alpha6 are difficult to distinguish.
	m_R61ObservabilityReady = false;
	m_R61Q6SensitivityMmPerDeg = 0.0;
	m_R61Alpha6SensitivityMmPerDeg = 0.0;
	m_R61Q7SensitivityMmPerDeg = 0.0;
	m_R61Q6Alpha6Cosine = 0.0;
	if (axis >= 7 && calibrationSampleCount > 0)
	{
		double q6Norm2 = 0.0;
		double alpha6Norm2 = 0.0;
		double q7Norm2 = 0.0;
		double q6Alpha6Dot = 0.0;
		int validSensitivitySamples = 0;
		std::vector<double> pointJacobian(3 * 4 * axis, 0.0);
		for (int calibrationSample = 0; calibrationSample < calibrationSampleCount; ++calibrationSample)
		{
			double point[3] = {};
			const double* pose = jointSampleMajor.data() + calibrationSample * axis;
			if (CalibrationV3EvaluatePointAndJacobian(axis,
				outA.data(), outD.data(), outAlpha.data(), thetaFixed.data(),
				beta.data(), outQ0.data(), pose, fixtureLocalXYZ,
				point, pointJacobian.data()) != CALV3_OK)
				continue;

			double ropeDirection[3] = {
				point[0] - report->anchorAfter[0],
				point[1] - report->anchorAfter[1],
				point[2] - report->anchorAfter[2]
			};
			const double ropeNorm = std::sqrt(ropeDirection[0] * ropeDirection[0] +
				ropeDirection[1] * ropeDirection[1] + ropeDirection[2] * ropeDirection[2]);
			if (!(ropeNorm > 1e-12)) continue;
			for (int row = 0; row < 3; ++row) ropeDirection[row] /= ropeNorm;

			// Craig MDH stores physical alpha6 in array slot 7 (zero-based 6).
			const int alpha6Column = 2 * axis + 6;
			const int q6Column = 3 * axis + 5;
			const int q7Column = 3 * axis + 6;
			double alpha6Derivative = 0.0;
			double q6Derivative = 0.0;
			double q7Derivative = 0.0;
			for (int row = 0; row < 3; ++row)
			{
				const int rowOffset = row * 4 * axis;
				alpha6Derivative += ropeDirection[row] * pointJacobian[rowOffset + alpha6Column];
				q6Derivative += ropeDirection[row] * pointJacobian[rowOffset + q6Column];
				q7Derivative += ropeDirection[row] * pointJacobian[rowOffset + q7Column];
			}
			alpha6Norm2 += alpha6Derivative * alpha6Derivative;
			q6Norm2 += q6Derivative * q6Derivative;
			q7Norm2 += q7Derivative * q7Derivative;
			q6Alpha6Dot += q6Derivative * alpha6Derivative;
			++validSensitivitySamples;
		}
		if (validSensitivitySamples > 0)
		{
			const double radToDegreeScale = 1000.0 * PI / 180.0;
			m_R61Q6SensitivityMmPerDeg = std::sqrt(q6Norm2 / validSensitivitySamples) * radToDegreeScale;
			m_R61Alpha6SensitivityMmPerDeg = std::sqrt(alpha6Norm2 / validSensitivitySamples) * radToDegreeScale;
			m_R61Q7SensitivityMmPerDeg = std::sqrt(q7Norm2 / validSensitivitySamples) * radToDegreeScale;
			if (q6Norm2 > 1e-24 && alpha6Norm2 > 1e-24)
				m_R61Q6Alpha6Cosine = q6Alpha6Dot / std::sqrt(q6Norm2 * alpha6Norm2);
			m_R61ObservabilityReady = true;
		}
	}

	m_R6OutA = outA;
	m_R6OutD = outD;
	m_R6OutAlpha = outAlpha;
	m_R6OutQ0 = outQ0;
	m_R6HaveResult = true;
	CalibrationV3GetLastTrace(&m_R6Trace);

	// Preserve V1's 12-slot display/writeback contract. For seven-axis its
	// terminal-length field represents d7 minus ToolOffset. q7 remains unchanged.
	calibResult12[0] = outA[2];
	calibResult12[1] = outA[3];
	calibResult12[2] = outD[0];
	calibResult12[3] = outD[3];
	calibResult12[4] = outD[4];
	calibResult12[5] = outD[axis - 1];
	for (int joint = 0; joint < 6; ++joint)
	{
		calibResult12[6 + joint] = (joint < axis) ? -outQ0[joint] * kRadToDeg : 0.0;
	}

	(void)toolOffsetM; // subtracted by the unchanged V1 GUI display code
	return status;
}

static CString FormatR6Trace(const CalibrationV3Trace& trace)
{
	CString text, line;
	line.Format(_T("R6.2 Craig MDH解析雅可比 + 激光8机先验 + q6可观测性诊断\r\n轴数=%d 记录=%d\r\n"),
		trace.axis, trace.count);
	text += line;
	text += _T("控制器Craig MDH：Alpha(deg)=0,90,-90,90,-90,90,-90\r\n");
	text += _T("激光量产基准：D(mm)=0,49,493,-120,317,0,93；Tool=49 mm\r\n");
	text += _T("量产物理参考：d2~d5跨8机最大跨度<=1.8 mm，alpha/q0约<=1.1 deg\r\n");
	if (m_R61ObservabilityReady)
	{
		line.Format(_T("q6候选状态=%s\r\n"
			"q6 RMS敏感度=%.6f mm/deg\r\n"
			"alpha6 RMS敏感度=%.6f mm/deg\r\n"
			"q7 RMS敏感度=%.9f mm/deg（应接近0）\r\n"
			"q6-alpha6列余弦=%.6f（绝对值越接近1越难区分）\r\n"),
			m_R61Q6CandidateActive ? _T("可见，已参与") : _T("零列，保持机械设定"),
			m_R61Q6SensitivityMmPerDeg, m_R61Alpha6SensitivityMmPerDeg,
			m_R61Q7SensitivityMmPerDeg, m_R61Q6Alpha6Cosine);
		text += line;
	}
	for (int recordIndex = 0; recordIndex < trace.count; ++recordIndex)
	{
		const CalibrationV3IterationRecord& rec = trace.record[recordIndex];
		line.Format(_T("\r\n[轮次 %d] scale=%.5g MAE=%.6f mm MAX=%.6f mm\r\n"),
			rec.iteration, rec.stepScale, rec.maeM * 1000.0, rec.maxAbsM * 1000.0);
		text += line;
		text += _T("d(mm): ");
		for (int joint = 0; joint < trace.axis; ++joint)
		{
			line.Format(_T("%.4f%s"), rec.d[joint] * 1000.0,
				joint + 1 == trace.axis ? _T("\r\n") : _T(", "));
			text += line;
		}
		text += _T("alpha(deg): ");
		for (int joint = 0; joint < trace.axis; ++joint)
		{
			line.Format(_T("%.5f%s"), rec.alpha[joint] * 180.0 / PI,
				joint + 1 == trace.axis ? _T("\r\n") : _T(", "));
			text += line;
		}
		text += _T("q0(deg): ");
		for (int joint = 0; joint < trace.axis; ++joint)
		{
			line.Format(_T("%.5f%s"), rec.q0[joint] * 180.0 / PI,
				joint + 1 == trace.axis ? _T("\r\n") : _T(", "));
			text += line;
		}
	}
	return text;
}

//计算按钮
void RopeEncoderCalibrationDlg::OnBnClickedButton7()
{
	CString selectitem;
	std::vector<double> oldErrorStorage(m_TotalNum, 0.0);
	std::vector<double> nowErrorStorage(m_TotalNum, 0.0);
	double* oldError[1] = { oldErrorStorage.data() };
	double* nowError[1] = { nowErrorStorage.data() };

	double totalOld = 0;
	double totalNow = 0;
	double aveOld = 0;
	double aveNow = 0;

	//获取机器人类型
	m_RbtTypeCombo.GetLBText(m_RbtTypeCombo.GetCurSel(), selectitem);
	string rbtType = CT2A(selectitem.GetBuffer(0));
	//保存测试数据
	SaveJoint(rbtType, m_FinishTime);
	m_RbtJoint = ReadThor_Joint(m_RobotType, m_RobotAxis, m_TotalNum, "./Joint/" + rbtType + "_Joint_" + m_FinishTime + ".txt");
	if (!m_R6HaveOriginalDH)
	{
		m_R6OriginalDH = m_ThisRbtDH;
		m_R6HaveOriginalDH = true;
		m_R6RestoreBtn.EnableWindow(TRUE);
	}

	CString JointOffsetCstr, str, show;
	VectorXd EncoderValue(m_TotalNum);
	CString EncoderValueStr;
	for (int ii = 0; ii < m_TotalNum; ii++) {
		EncoderValueStr = m_List.GetItemText(ii, 2);
		EncoderValue[ii] = atof(CT2A(EncoderValueStr.GetBuffer(0)));
	}

	//单位转换为 m
	EncoderValue *= 0.001;
	//补偿拉线编码器内部长度和末端旋转的长度
	EncoderValue += VectorXd::Ones(m_TotalNum) * (0.043 + 0.0195);

	if (m_RobotType == Scara) 
	{

	}
	else 
	{

		// 在调用 Calibration 之前添加以下代码输出打印
		{
			CString output;

			// 1. 基本类型
			output.Format(_T("\n=== Calibration 入参开始 ===\n"));
			OutputDebugString(output);

			output.Format(_T("RobotType = %d\n"), m_RobotType);
			OutputDebugString(output);
			output.Format(_T("RobotAxis = %d\n"), m_RobotAxis);
			OutputDebugString(output);

			// 2. m_ThisRbtDH (7 x m_RobotAxis)
			output.Format(_T("m_ThisRbtDH 矩阵 (rows=%d, cols=%d):\n"), m_ThisRbtDH.rows(), m_ThisRbtDH.cols());
			OutputDebugString(output);
			for (int row = 0; row < m_ThisRbtDH.rows(); row++)
			{
				CString line;
				for (int col = 0; col < m_ThisRbtDH.cols(); col++)
				{
					CString val;
					val.Format(_T("%.6lf\t"), m_ThisRbtDH(row, col));
					line += val;
				}
				line += _T("\n");
				OutputDebugString(line);
			}

			// 3. m_RbtJoint (m_RobotAxis x m_TotalNum)
			output.Format(_T("m_RbtJoint 矩阵 (rows=%d, cols=%d):\n"), m_RbtJoint.rows(), m_RbtJoint.cols());
			OutputDebugString(output);
			for (int row = 0; row < m_RbtJoint.rows(); row++)
			{
				CString line;
				for (int col = 0; col < m_RbtJoint.cols(); col++)
				{
					CString val;
					val.Format(_T("%.6lf\t"), m_RbtJoint(row, col));
					line += val;
				}
				line += _T("\n");
				OutputDebugString(line);
			}

			// 4. EncoderValue (VectorXd, 长度 m_TotalNum)
			output.Format(_T("EncoderValue (size=%d, 单位:m):\n"), EncoderValue.size());
			OutputDebugString(output);
			for (int i = 0; i < EncoderValue.size(); i++)
			{
				CString val;
				val.Format(_T("  [%d] = %.6lf\n"), i, EncoderValue(i));
				OutputDebugString(val);
			}

			// 5. m_CalibResult (double[12])
			OutputDebugString(_T("m_CalibResult (before calibration):\n"));
			for (int i = 0; i < 12; i++)
			{
				CString val;
				val.Format(_T("  [%d] = %.6lf\n"), i, m_CalibResult[i]);
				OutputDebugString(val);
			}

			// 6. oldError 和 nowError（双指针，入参时只分配了外层数组，内层指针未初始化）
			output.Format(_T("oldError 指针地址 = %p, 外层数组大小 = %d\n"), oldError, m_TotalNum);
			OutputDebugString(output);
			output.Format(_T("nowError 指针地址 = %p, 外层数组大小 = %d\n"), nowError, m_TotalNum);
			OutputDebugString(output);
			// 注意：oldError[i] 和 nowError[i] 在入参时未初始化，打印无意义，故省略

			OutputDebugString(_T("=== Calibration 入参结束 ===\n"));
		}

		CalibrationV3Report v3Report = {};
		const int v3Status = RunCalibrationV3ForGui(
			m_RobotAxis, m_TotalNum, m_ThisRbtDH, m_RbtJoint, EncoderValue,
			m_ThisRbt.ToolOffset, m_CalibResult, oldError[0], nowError[0], &v3Report);

		CString v3Log;
		v3Log.Format(
			_T("V3 build=%S status=%d iteration=%d active=%d rank=%d condition=%.6g MAE(mm)=%.6f -> %.6f MAX(mm)=%.6f -> %.6f\n"),
			CalibrationV3BuildId(), v3Status, v3Report.iterations,
			v3Report.activeCount, v3Report.jacobianRank, v3Report.jacobianCondition,
			v3Report.maeBeforeM * 1000.0, v3Report.maeAfterM * 1000.0,
			v3Report.maxAbsBeforeM * 1000.0, v3Report.maxAbsAfterM * 1000.0);
		OutputDebugString(v3Log);
		m_R6TraceEdit.SetWindowTextW(FormatR6Trace(m_R6Trace));
		m_R6FeedbackBtn.EnableWindow(m_R6HaveResult ? TRUE : FALSE);

		// Persist the V3 summary into the normal communication log and also
		// write a point-by-point CSV. The latter is required to distinguish a
		// global geometry error from one bad rope sample or one bad pose.
		std::string v3LogText = CT2A(v3Log.GetString());
		WriteLog(0, const_cast<char*>(v3LogText.c_str()));
		_mkdir("./Result");
		const std::string diagnosticStamp = m_FinishTime.empty() ? "manual" : m_FinishTime;
		const std::string diagnosticPath = "./Result/V3Diagnostic_" + diagnosticStamp + ".csv";
		std::ofstream diagnostic(diagnosticPath, std::ios::out | std::ios::trunc);
		if (diagnostic)
		{
			diagnostic << std::setprecision(15);
			diagnostic << "build," << CalibrationV3BuildId() << "\n";
			diagnostic << "status," << v3Status << "\n";
			diagnostic << "iterations," << v3Report.iterations << "\n";
			diagnostic << "active_count," << v3Report.activeCount << "\n";
			diagnostic << "rank," << v3Report.jacobianRank << "\n";
			diagnostic << "condition," << v3Report.jacobianCondition << "\n";
			diagnostic << "mae_before_mm," << v3Report.maeBeforeM * 1000.0 << "\n";
			diagnostic << "mae_after_mm," << v3Report.maeAfterM * 1000.0 << "\n";
			diagnostic << "max_before_mm," << v3Report.maxAbsBeforeM * 1000.0 << "\n";
			diagnostic << "max_after_mm," << v3Report.maxAbsAfterM * 1000.0 << "\n";
			diagnostic << "laser_reference_count,8\n";
			diagnostic << "laser_reference_source,2.HE3_laser_calibration_dataset\n";
			diagnostic << "laser_design_d_mm,0|49|493|-120|317|0|93\n";
			diagnostic << "controller_mdh_alpha_deg,0|90|-90|90|-90|90|-90\n";
			diagnostic << "legacy_sdh_alpha_deg,90|-90|90|-90|90|-90|0\n";
			diagnostic << "laser_population_d2_d5_max_span_mm,1.798\n";
			diagnostic << "laser_population_alpha_q_max_span_deg,1.079407\n";
			diagnostic << "terminal_structure_d7_mm," << (m_ThisRbtDH(1, m_RobotAxis - 1) - m_ThisRbt.ToolOffset) * 1000.0 << "\n";
			diagnostic << "tool_offset_mm," << m_ThisRbt.ToolOffset * 1000.0 << "\n";
			diagnostic << "terminal_effective_mm," << m_ThisRbtDH(1, m_RobotAxis - 1) * 1000.0 << "\n";
			diagnostic << "q6_sensitivity_rms_mm_per_deg," << m_R61Q6SensitivityMmPerDeg << "\n";
			diagnostic << "alpha6_sensitivity_rms_mm_per_deg," << m_R61Alpha6SensitivityMmPerDeg << "\n";
			diagnostic << "q7_sensitivity_rms_mm_per_deg," << m_R61Q7SensitivityMmPerDeg << "\n";
			diagnostic << "q6_alpha6_column_cosine," << m_R61Q6Alpha6Cosine << "\n";
			diagnostic << "q6_candidate_active," << (m_R61Q6CandidateActive ? 1 : 0) << "\n";
			diagnostic << "iteration,mae_mm,max_mm,step_scale";
			for (int joint = 0; joint < m_RobotAxis; ++joint) diagnostic << ",d" << (joint + 1) << "_mm";
			for (int joint = 0; joint < m_RobotAxis; ++joint) diagnostic << ",alpha" << (joint + 1) << "_deg";
			for (int joint = 0; joint < m_RobotAxis; ++joint) diagnostic << ",q0_" << (joint + 1) << "_deg";
			diagnostic << "\n";
			for (int traceIndex = 0; traceIndex < m_R6Trace.count; ++traceIndex)
			{
				const CalibrationV3IterationRecord& rec = m_R6Trace.record[traceIndex];
				diagnostic << rec.iteration << ',' << rec.maeM * 1000.0 << ','
					<< rec.maxAbsM * 1000.0 << ',' << rec.stepScale;
				for (int joint = 0; joint < m_RobotAxis; ++joint) diagnostic << ',' << rec.d[joint] * 1000.0;
				for (int joint = 0; joint < m_RobotAxis; ++joint) diagnostic << ',' << rec.alpha[joint] * 180.0 / PI;
				for (int joint = 0; joint < m_RobotAxis; ++joint) diagnostic << ',' << rec.q0[joint] * 180.0 / PI;
				diagnostic << "\n";
			}
			diagnostic << "result_slot,value_before_gui_unit_conversion\n";
			for (int resultIndex = 0; resultIndex < 12; ++resultIndex)
			{
				diagnostic << resultIndex << ',' << m_CalibResult[resultIndex] << "\n";
			}
			diagnostic << "point,rope_raw_mm,rope_with_fixed_offset_mm";
			for (int joint = 0; joint < m_RobotAxis; ++joint)
			{
				diagnostic << ",q" << (joint + 1) << "_rad";
			}
			diagnostic << ",sample_std_mm,residual_before_mm,residual_after_mm\n";
			for (int point = 0; point < m_TotalNum; ++point)
			{
				const double ropeWithOffsetM = EncoderValue(point);
				diagnostic << (point + 1) << ','
					<< (ropeWithOffsetM - 0.043 - 0.0195) * 1000.0 << ','
					<< ropeWithOffsetM * 1000.0;
				for (int joint = 0; joint < m_RobotAxis; ++joint)
				{
					diagnostic << ',' << m_RbtJoint(joint, point);
				}
				const double sampleStdMm = (point < static_cast<int>(m_R5EncoderStdCounts.size()))
					? m_R5EncoderStdCounts[static_cast<size_t>(point)] / 284.94 : 0.0;
				diagnostic << ',' << sampleStdMm
					<< ',' << oldError[0][point] * 1000.0
					<< ',' << nowError[0][point] * 1000.0 << "\n";
			}
		}

		if (v3Status != CALV3_OK && v3Status != CALV3_STOP_NO_IMPROVEMENT)
		{
			CString errorText;
			errorText.Format(_T("V3七轴标定失败。\r\n状态码: %d\r\n秩: %d/%d\r\n条件数: %.6g\r\n未修改机器人参数。"),
				v3Status, v3Report.jacobianRank, v3Report.activeCount, v3Report.jacobianCondition);
			MessageBox(errorText, _T("V3标定安全检查"), MB_OK | MB_ICONERROR);
			return;
		}

		// R6.2 production gate: a low fitted residual alone is insufficient.
		// Reject ill-conditioned, noisy or physically implausible solutions so
		// the Save button cannot write a compensating-but-wrong parameter set.
		double j7SweepRangeMm = 0.0;
		double point19RepeatRangeMm = 0.0;
		if (m_RobotAxis == 7 && m_TotalNum >= 100)
		{
			double j7Min = EncoderValue(60);
			double j7Max = EncoderValue(60);
			for (int point = 61; point <= 68; ++point)
			{
				if (EncoderValue(point) < j7Min) j7Min = EncoderValue(point);
				if (EncoderValue(point) > j7Max) j7Max = EncoderValue(point);
			}
			j7SweepRangeMm = (j7Max - j7Min) * 1000.0;

			const int point19Indices[4] = { 18, 70, 73, 76 };
			double p19Min = EncoderValue(point19Indices[0]);
			double p19Max = p19Min;
			for (int repeat = 1; repeat < 4; ++repeat)
			{
				const double value = EncoderValue(point19Indices[repeat]);
				if (value < p19Min) p19Min = value;
				if (value > p19Max) p19Max = value;
			}
			point19RepeatRangeMm = (p19Max - p19Min) * 1000.0;
		}
		double maxSampleStdMm = 0.0;
		for (size_t i = 0; i < m_R5EncoderStdCounts.size(); ++i)
		{
			const double pointStdMm = m_R5EncoderStdCounts[i] / 284.94;
			if (pointStdMm > maxSampleStdMm) maxSampleStdMm = pointStdMm;
		}
		double maxTargetErrorDeg = 0.0;
		int maxTargetErrorPoint = 0;
		for (int point = 0; point < m_TotalNum; ++point)
		{
			CString targetText = m_List.GetItemText(point, 1);
			targetText.Replace(_T(','), _T(' '));
			std::istringstream targetStream{ static_cast<const char*>(CT2A(targetText)) };
			for (int joint = 0; joint < m_RobotAxis; ++joint)
			{
				double targetDeg = 0.0;
				if (!(targetStream >> targetDeg)) break;
				const double actualDeg = m_RbtJoint(joint, point) * 180.0 / PI;
				const double errorDeg = std::fabs(targetDeg - actualDeg);
				if (errorDeg > maxTargetErrorDeg)
				{
					maxTargetErrorDeg = errorDeg;
					maxTargetErrorPoint = point + 1;
				}
			}
		}
		const bool residualGate = v3Report.maeAfterM <= 0.002 && v3Report.maxAbsAfterM <= 0.010;
		const bool conditionGate = std::isfinite(v3Report.jacobianCondition) && v3Report.jacobianCondition <= 1000.0;
		const bool rankGate = v3Report.jacobianRank == v3Report.activeCount;
		const bool motionGate = maxTargetErrorDeg <= 0.1;
		const bool repeatGate = maxSampleStdMm <= 0.5;
		const bool fixtureGate = j7SweepRangeMm <= 1.0;
		const bool point19Gate = point19RepeatRangeMm <= 1.0;
		// R6.2 does not force a structurally zero joint into the solve.  With the
		// current HE3 axial measurement point, q6 and q7 are exact zero columns.
		// If a future fixture/model moves the point off the q6 axis, q6 is admitted
		// automatically and must then have useful sensitivity without duplicating
		// alpha6.  q7 must remain invisible for this fixture model.
		const bool q7StructuralGate = m_R61ObservabilityReady &&
			m_R61Q7SensitivityMmPerDeg < 0.00001;
		const bool q6DecisionGate = !m_R61Q6CandidateActive
			? (m_R61ObservabilityReady && m_R61Q6SensitivityMmPerDeg < 0.00001)
			: (m_R61ObservabilityReady && m_R61Q6SensitivityMmPerDeg > 0.01 &&
				std::fabs(m_R61Q6Alpha6Cosine) < 0.995);
		const bool observabilityGate = q6DecisionGate && q7StructuralGate;
		bool lengthGate = true;
		bool alphaGate = true;
		for (int joint = 0; joint < m_RobotAxis; ++joint)
		{
			lengthGate = lengthGate && std::fabs(m_R6OutD[joint] - m_ThisRbtDH(1, joint)) <= 0.0061;
			alphaGate = alphaGate && std::fabs(m_R6OutAlpha[joint] - m_ThisRbtDH(2, joint)) <= 1.51 * PI / 180.0;
		}
		bool zeroGate = true;
		const int zeroJointEnd = (m_RobotAxis < 6) ? m_RobotAxis : 6;
		for (int joint = 1; joint < zeroJointEnd; ++joint)
			zeroGate = zeroGate && std::fabs(m_R6OutQ0[joint] * 180.0 / PI) <= 2.01;
		m_R5CalibrationPassed = residualGate && conditionGate && rankGate && motionGate &&
			repeatGate && fixtureGate && point19Gate && observabilityGate &&
			lengthGate && alphaGate && zeroGate;

		CString gateLog;
		gateLog.Format(_T("R6.2 gate pass=%d residual=%d condition=%d rank=%d motion=%d repeat=%d fixture=%d point19=%d observability=%d length=%d alpha=%d zero=%d q6_active=%d max_target_error_deg=%.6f bad_point=%d max_sample_std_mm=%.6f j7_range_mm=%.6f point19_range_mm=%.6f q6_sensitivity_mm_per_deg=%.6f q7_sensitivity_mm_per_deg=%.9f q6_alpha6_cosine=%.6f\n"),
			m_R5CalibrationPassed ? 1 : 0, residualGate ? 1 : 0, conditionGate ? 1 : 0,
			rankGate ? 1 : 0, motionGate ? 1 : 0,
			repeatGate ? 1 : 0, fixtureGate ? 1 : 0, point19Gate ? 1 : 0,
			observabilityGate ? 1 : 0, lengthGate ? 1 : 0, alphaGate ? 1 : 0, zeroGate ? 1 : 0,
			m_R61Q6CandidateActive ? 1 : 0,
			maxTargetErrorDeg, maxTargetErrorPoint, maxSampleStdMm,
			j7SweepRangeMm, point19RepeatRangeMm,
			m_R61Q6SensitivityMmPerDeg, m_R61Q7SensitivityMmPerDeg, m_R61Q6Alpha6Cosine);
		OutputDebugString(gateLog);
		std::string gateLogText = CT2A(gateLog.GetString());
		WriteLog(0, const_cast<char*>(gateLogText.c_str()));

		//单位转换从 m 转为 mm
		aveOld = v3Report.maeBeforeM * 1000.0;
		aveNow = v3Report.maeAfterM * 1000.0;
		m_CalibResult[0] = m_CalibResult[0] * 1000;
		m_CalibResult[1] = m_CalibResult[1] * 1000;
		m_CalibResult[2] = m_CalibResult[2] * 1000;
		m_CalibResult[3] = m_CalibResult[3] * 1000;
		m_CalibResult[4] = m_CalibResult[4] * 1000;
		m_CalibResult[5] = (m_CalibResult[5]  - m_ThisRbt.ToolOffset) * 1000;

		str.Format(_T("%.3lf"), m_CalibResult[0]);
		m_A1.SetWindowTextW(str);
		str.Format(_T("%.3lf"), m_CalibResult[1]);
		m_A2.SetWindowTextW(str);
		str.Format(_T("%.3lf"), m_CalibResult[2]);
		m_D1.SetWindowTextW(str);
		str.Format(_T("%.3lf"), m_CalibResult[3]);
		m_D2.SetWindowTextW(str);
		str.Format(_T("%.3lf"), m_CalibResult[4]);
		m_D3.SetWindowTextW(str);
		str.Format(_T("%.3lf"), m_CalibResult[5]);
		m_D4.SetWindowTextW(str);

		str.Format(_T("%.3lf"), aveOld);
		m_ErrorBefore.SetWindowTextW(str);
		str.Format(_T("%.3lf"), aveNow);
		m_ErrorAfter.SetWindowTextW(str);

		//角度转编码器脉冲值
		for (int ii = 0; ii < 6; ii++) {
			m_CalibResult[ii + 6] = m_CalibResult[ii + 6] * m_ThisRbt.EncoderLineNum[ii] /360;
			str.Format(_T("%.3lf，"), m_CalibResult[ii + 6]);
			JointOffsetCstr += str;
		}

		JointOffsetCstr = JointOffsetCstr.Left(JointOffsetCstr.GetLength() - 1);

		//输出打印信息用于调试//
		CString msg;
		for (int i = 0; i < 12; i++) {
			msg.Format(_T("m_CalibResult[%d] = %.6lf\n"), i, m_CalibResult[i]);
			OutputDebugString(msg);
		}
		msg.Format(_T("JointOffsetCstr = %s\n"), JointOffsetCstr);
		OutputDebugString(msg);
		////////////////////////

		m_JointShow.SetWindowTextW(JointOffsetCstr);
	}

	m_SaveBtn.EnableWindow(m_R5CalibrationPassed);
	if (!m_R5CalibrationPassed)
	{
		MessageBox(_T("R6.2计算已完成，但未通过量产安全门限。\r\n"
			"要求：MAE<=2mm、最大残差<=10mm、满秩、条件数<=1000、\r\n"
			"单点20次标准差<=0.5mm、J7扫描及19点重复极差<=1mm、\r\n"
			"q6按解析雅可比自动判定：零列时固定；可见时必须有有效敏感度且不能与alpha6重合；\r\n"
			"q7敏感度必须接近0；\r\n"
			"目标/实际关节误差<=0.1度、杆长<=6mm、alpha<=1.5度、零位<=2度。\r\n"
			"结果仅供诊断，已禁止保存和写回机器人。"),
			_T("R6.2标定结果未通过"), MB_OK | MB_ICONWARNING);
	}
	m_CalibBtn.EnableWindow(false);
}

void RopeEncoderCalibrationDlg::OnBnClickedR6Feedback()
{
	if (!m_R6HaveResult || static_cast<int>(m_R6OutD.size()) != m_RobotAxis)
	{
		MessageBox(_T("当前没有可回灌的R6.2计算结果。"), _T("R6.2复算"), MB_OK | MB_ICONINFORMATION);
		return;
	}
	for (int joint = 0; joint < m_RobotAxis; ++joint)
	{
		m_ThisRbtDH(0, joint) = m_R6OutA[joint];
		m_ThisRbtDH(1, joint) = m_R6OutD[joint];
		m_ThisRbtDH(2, joint) = m_R6OutAlpha[joint];
		// q0 and thetaFixed enter FK as a sum.  Folding q0 into row 4 makes
		// the next calculation a genuine model-feedback stability check.
		m_ThisRbtDH(3, joint) += m_R6OutQ0[joint];
	}
	MessageBox(_T("已在内存中将本轮完整MDH作为下一轮输入。\r\n"
		"不会写机器人；现在复算同一批数据。"), _T("R6.2复算"), MB_OK | MB_ICONINFORMATION);
	OnBnClickedButton7();
}

void RopeEncoderCalibrationDlg::OnBnClickedR6Restore()
{
	if (!m_R6HaveOriginalDH)
	{
		MessageBox(_T("没有保存的原始MDH。"), _T("R6.2恢复"), MB_OK | MB_ICONINFORMATION);
		return;
	}
	m_ThisRbtDH = m_R6OriginalDH;
	m_R6HaveResult = false;
	m_R6FeedbackBtn.EnableWindow(FALSE);
	m_R6TraceEdit.SetWindowTextW(_T("已恢复本次计算前的原始MDH。"));
}

//保存按钮,用于保存末端距离拉线编码器的距离
void RopeEncoderCalibrationDlg::OnBnClickedButton8()
{
	if (!m_R5CalibrationPassed)
	{
		MessageBox(_T("当前R6.2结果未通过安全门限，禁止保存或写回机器人。"),
			_T("R6.2写回保护"), MB_OK | MB_ICONERROR);
		return;
	}
	PowerOff();
	// TODO: 在此添加控件通知处理程序代码

	CString selectitem;
	CString str;
	string tempValue;
	double dValue;
	string* msgArray = new string[Error_After + 1]();
	if (m_RobotType == Scara)
	{

	}
	else
	{
		msgArray = new string[Error_After + 1]();
		m_RbtTypeCombo.GetLBText(m_RbtTypeCombo.GetCurSel(), selectitem);
		// TODO: Add your control notification handler code here	
		string rbtType = CT2A(selectitem.GetBuffer(0));

		msgArray[StartTime] = m_StartTime;
		msgArray[FinishTime] = m_FinishTime;

		for (int ii = 0; ii < 6; ii++) {
			msgArray[ii + J0_Offset] = to_string(m_CalibResult[ii + 6]);
		}

		CString *list = new CString[m_TotalNum];
		for (int ii = 0; ii < m_TotalNum; ii++) {
			list[ii] = m_List.GetItemText(ii, 2);
		}

		SaveData(list, rbtType, m_FinishTime);

		//int stt = DownLoad(m_RbtIPStr, 21, _T(""), _T(""));

		//stt = DecodeXmlFile();

		//将 DH 参数写入到内存中
		CString sSend;

		//替换 DH 参数
		msgArray[A2_Before] = ReadIDNValue(m_IDNA, m_IndexStr, m_IDNA1_ArrIndex);
		m_A1.GetWindowTextW(str);
		msgArray[A2_After] = CT2A(str.GetBuffer(0));
		WriteIDNValue(m_IDNA, m_IndexStr, str, m_IDNA1_ArrIndex);

		msgArray[A3_Before] = ReadIDNValue(m_IDNA, m_IndexStr, m_IDNA2_ArrIndex);
		m_A2.GetWindowTextW(str);
		msgArray[A3_After] = CT2A(str.GetBuffer(0));
		WriteIDNValue(m_IDNA, m_IndexStr, str, m_IDNA2_ArrIndex);

		msgArray[D0_Before] = ReadIDNValue(m_IDND, m_IndexStr, m_IDND1_ArrIndex);
		m_D1.GetWindowTextW(str);
		msgArray[D0_After] = CT2A(str.GetBuffer(0));
		WriteIDNValue(m_IDND, m_IndexStr, str, m_IDND1_ArrIndex);

		msgArray[D3_Before] = ReadIDNValue(m_IDND, m_IndexStr, m_IDND2_ArrIndex);
		m_D2.GetWindowTextW(str);
		msgArray[D3_After] = CT2A(str.GetBuffer(0));
		WriteIDNValue(m_IDND, m_IndexStr, str, m_IDND2_ArrIndex);

		msgArray[D4_Before] = ReadIDNValue(m_IDND, m_IndexStr, m_IDND3_ArrIndex);
		m_D3.GetWindowTextW(str);
		msgArray[D4_After] = CT2A(str.GetBuffer(0));
		WriteIDNValue(m_IDND, m_IndexStr, str, m_IDND3_ArrIndex);

		msgArray[D5_Before] = ReadIDNValue(m_IDND, m_IndexStr, m_IDND4_ArrIndex);
		m_D4.GetWindowTextW(str);
		msgArray[D5_After] = CT2A(str.GetBuffer(0));
		WriteIDNValue(m_IDND, m_IndexStr, str, m_IDND4_ArrIndex);

		//叠加零点值
		tempValue = ReadIDNValue(m_IDNZeroEncoderValue, m_IndexStr, m_IDNZeroEncoderValue1_ArrIndex);
		msgArray[J0_Before] = tempValue;
		dValue = atof(tempValue.c_str());
		int iValue = (int)((m_ThisRbt.MoveDirection[0] * m_CalibResult[6]) + dValue);
		str.Format(_T("%d"), iValue);
		msgArray[J0_After] = CT2A(str.GetBuffer(0));
		WriteIDNValue(m_IDNZeroEncoderValue, m_IndexStr, str, m_IDNZeroEncoderValue1_ArrIndex);

		tempValue = ReadIDNValue(m_IDNZeroEncoderValue, m_IndexStr, m_IDNZeroEncoderValue2_ArrIndex);
		msgArray[J1_Before] = tempValue;
		dValue = atof(tempValue.c_str());
		iValue = (int)((m_ThisRbt.MoveDirection[1] * m_CalibResult[7]) + dValue);
		str.Format(_T("%d"), iValue);
		msgArray[J1_After] = CT2A(str.GetBuffer(0));
		WriteIDNValue(m_IDNZeroEncoderValue, m_IndexStr, str, m_IDNZeroEncoderValue2_ArrIndex);

		tempValue = ReadIDNValue(m_IDNZeroEncoderValue, m_IndexStr, m_IDNZeroEncoderValue3_ArrIndex);
		msgArray[J2_Before] = tempValue;
		dValue = atof(tempValue.c_str());
		iValue = (int)((m_ThisRbt.MoveDirection[2] * m_CalibResult[8]) + dValue);
		str.Format(_T("%d"), iValue);
		msgArray[J2_After] = CT2A(str.GetBuffer(0));
		WriteIDNValue(m_IDNZeroEncoderValue, m_IndexStr, str, m_IDNZeroEncoderValue3_ArrIndex);

		tempValue = ReadIDNValue(m_IDNZeroEncoderValue, m_IndexStr, m_IDNZeroEncoderValue4_ArrIndex);
		msgArray[J3_Before] = tempValue;
		dValue = atof(tempValue.c_str());
		iValue = (int)((m_ThisRbt.MoveDirection[3] * m_CalibResult[9]) + dValue);
		str.Format(_T("%d"), iValue);
		msgArray[J3_After] = CT2A(str.GetBuffer(0));
		WriteIDNValue(m_IDNZeroEncoderValue, m_IndexStr, str, m_IDNZeroEncoderValue4_ArrIndex);

		tempValue = ReadIDNValue(m_IDNZeroEncoderValue, m_IndexStr, m_IDNZeroEncoderValue5_ArrIndex);
		msgArray[J4_Before] = tempValue;
		dValue = atof(tempValue.c_str());
		iValue = (int)((m_ThisRbt.MoveDirection[4] * m_CalibResult[10]) + dValue);
		str.Format(_T("%d"), iValue);
		msgArray[J4_After] = CT2A(str.GetBuffer(0));
		WriteIDNValue(m_IDNZeroEncoderValue, m_IndexStr, str, m_IDNZeroEncoderValue5_ArrIndex);

		tempValue = ReadIDNValue(m_IDNZeroEncoderValue, m_IndexStr, m_IDNZeroEncoderValue6_ArrIndex);
		msgArray[J5_Before] = tempValue;
		dValue = atof(tempValue.c_str());
		iValue = (int)((m_ThisRbt.MoveDirection[5] * m_CalibResult[11]) + dValue);
		str.Format(_T("%d"), iValue);
		msgArray[J5_After] = CT2A(str.GetBuffer(0));
		WriteIDNValue(m_IDNZeroEncoderValue, m_IndexStr, str, m_IDNZeroEncoderValue6_ArrIndex);

		m_ErrorBefore.GetWindowTextW(str);
		msgArray[Error_Before] = CT2A(str.GetBuffer(0));

		m_ErrorAfter.GetWindowTextW(str);
		msgArray[Error_After] = CT2A(str.GetBuffer(0));
	}

	SaveCalibResult(msgArray);
	//OnWriteXml(NULL, NULL);

	int bStt = WriteIDNValue(m_IDNSave, m_IndexStr, m_IDNSaveOption);
	if (bStt == 0) {
		MessageBox(_T("保存成功"), _T("提示"), MB_OK);
	}

	DeleteTemp();
	m_SaveBtn.EnableWindow(false);
}

//选择标定模式
void RopeEncoderCalibrationDlg::OnBnClickedButton9()
{
	// TODO: 在此添加控件通知处理程序代码
	int idx = m_RbtTypeCombo.GetCurSel();
	CString robotString;
	CString selectString;

	m_RbtTypeCombo.GetLBText(idx, robotString);
	

	idx = m_CalibTypeCombo.GetCurSel();
	m_CalibTypeCombo.GetLBText(idx, selectString);
	if (selectString == _T("粗标"))
	{
		CString strIP;
		BYTE IP0, IP1, IP2, IP3;
		m_RbtIPAdr.GetAddress(IP0, IP1, IP2, IP3);
		strIP.Format(_T("%d.%d.%d.%d"), IP0, IP1, IP2, IP3);
		m_RbtIPStr = strIP;
		int stt = DownLoad(m_RbtIPStr, 21, _T(""), _T(""));

		stt = DecodeXmlFile();

		RoughCalibDlg.DoModal();
		m_CalibTypeCombo.SelectString(0, _T("拉线编码器"));
		DeleteTemp();
	}
}

//回机器人零点
void RopeEncoderCalibrationDlg::OnBnClickedButton10()
{
	if (ReturnZeroThread != NULL) {
		return;
	}
	// TODO: 在此添加控件通知处理程序代码
	ReturnZeroThread = NULL;
	CString sSend;
	if (iAPIVersionType == APIVersionType_1)
	{
		sSend = (CString)m_CMDStart.c_str();
		TargetPacketID = QKMLinkSend(sSend);
	}
	else
	{
		sSend = (CString)m_CMDThreadStart.c_str();
		TargetPacketID = QKMLinkSend(sSend);

		sSend = (CString)m_CMDRobotAttached.c_str();
		TargetPacketID = QKMLinkSend(sSend);
	}
	m_LocIdx = 0;
	sSend = (CString)m_CMDRobotWhereAngle.c_str();
	TargetPacketID = QKMLinkSend(sSend);
	QKMLinkSetTargetPacketID(TargetPacketID);
	m_Where = true;

	ReturnZeroThread = AfxBeginThread(RopeEncoderCalibrationDlg::ZeroThread, (LPVOID)this);
	m_LocationZero.EnableWindow(false);
	return;
}

//断开连接
void RopeEncoderCalibrationDlg::OnBnClickedButton11()
{
	QKMLinkDisconnect();
	if (KeepConnectThread != NULL) {
		TerminateThread(KeepConnectThread->m_hThread, 0);
	}
	m_Client.Close();
	m_ConnectBtn.EnableWindow(true);
	m_DisconnectBtn.EnableWindow(false);
	m_StartBtn.EnableWindow(false);
	m_PauseBtn.EnableWindow(false);
	m_ContinueBtn.EnableWindow(false);
	m_StopBtn.EnableWindow(false);
	m_CalibTypeCombo.EnableWindow(false);
	m_RbtTypeCombo.EnableWindow(true);
	m_EncoderBtn.EnableWindow(false);
	m_LocationZero.EnableWindow(false);
	m_CalibTypeBtn.EnableWindow(true);
	m_RbtSpeed.EnableWindow(false);
	m_EncoderHomeStr.EnableWindow(false);
}

//双精度数组转换为字符串
std::string doubleArrayToString(double* array, int size) {
	std::ostringstream oss;
	for (int i = 0; i < size; ++i) {
		oss << std::fixed << std::setprecision(2) << array[i]; // 设置小数点后两位
		if (i < size - 1) {
			oss << ", "; // 在数字之间添加逗号和空格
		}
	}
	return oss.str();
}

//行转换为字符串
std::string rowToString(const Eigen::MatrixXd& matrix, int rowIndex) {
	std::ostringstream oss;
	bool firstElement = true;
	for (int col = 0; col < matrix.cols(); ++col) {
		if (!firstElement) {
			oss << ", "; // 在元素之间添加逗号和空格
		}
		oss << matrix(rowIndex, col); // 将当前元素添加到字符串流中
		firstElement = false; // 标记已经添加了第一个元素
	}
	return oss.str(); // 返回最终的字符串
}

//选择机器人类型,根据类型去读取文件
void RopeEncoderCalibrationDlg::OnCbnSelchangeCombo1()
{
	CString selectitem;
	CString str;
	m_RbtTypeCombo.GetLBText(m_RbtTypeCombo.GetCurSel(), selectitem);
	string rbtType = CT2A(selectitem.GetBuffer(0));
	ReadConfigFile(_rbtConfigPath + '/' + rbtType + _configExtension, m_ThisRbt);

	m_TotalNum = m_ThisRbt.CalLocationNumber;
	m_RobotAxis = m_ThisRbt.RobotAxis;
	m_RobotType = m_ThisRbt.RobotType;
	m_WaitTime = m_ThisRbt.MotionWaittime;
	m_ShortWaitTime = m_ThisRbt.GetLocWaittime;
	m_RobotHere = new CString[m_TotalNum];
	m_ThisRbtDH = MatrixXd(7, m_RobotAxis);
	for (size_t i = 0; i < m_RobotAxis; i++)
	{
		m_ThisRbtDH(0, i) = m_ThisRbt.a[i];
		m_ThisRbtDH(1, i) = m_ThisRbt.d[i];
		m_ThisRbtDH(2, i) = m_ThisRbt.alpha[i];
		m_ThisRbtDH(3, i) = m_ThisRbt.theta[i];
		m_ThisRbtDH(4, i) = m_ThisRbt.beta[i];
		m_ThisRbtDH(5, i) = m_ThisRbt.MoveDirection[i];
		m_ThisRbtDH(6, i) = m_ThisRbt.EncoderLineNum[i];
	}

	m_ThisRbtDH(1, m_RobotAxis - 1) += m_ThisRbt.ToolOffset;

	if (m_List.GetItemCount() < m_TotalNum) {
		CString str;
		for (int ii = m_List.GetItemCount() + 1; ii <= m_TotalNum; ii++)
		{
			str.Format(_T("%d"), ii);
			m_List.InsertItem(ii, str);
		}
	}
	else
	{
		int offsetNum = m_List.GetItemCount() - m_TotalNum;
		for (int ii = 0; ii < offsetNum; ii++)
		{
			m_List.DeleteItem(m_List.GetItemCount() - 1);
		}
	}
	
	m_EncoderFeedbackHeader = m_NormalHeader;
	if (m_RobotType == Scara)
	{
		m_CellIPAdr.SetAddress(192, 168, 10, 105);
		GetDlgItem(IDC_EDIT4)->ShowWindow(FALSE);
		GetDlgItem(IDC_EDIT5)->ShowWindow(FALSE);
		GetDlgItem(IDC_EDIT6)->ShowWindow(FALSE);
		GetDlgItem(IDC_EDIT7)->ShowWindow(FALSE);
		GetDlgItem(IDC_EDIT10)->ShowWindow(FALSE);
		GetDlgItem(IDC_STATIC1)->ShowWindow(FALSE);
		GetDlgItem(IDC_STATIC2)->ShowWindow(FALSE);
		GetDlgItem(IDC_STATIC3)->ShowWindow(FALSE);
		GetDlgItem(IDC_STATIC4)->ShowWindow(FALSE);
		GetDlgItem(IDC_STATIC5)->ShowWindow(FALSE);

	}
	else if(m_RobotType == Cobot)
	{
		m_CellIPAdr.SetAddress(192, 168, 10, 165);
		m_EncoderFeedbackHeader = m_CobotHeader;

		GetDlgItem(IDC_EDIT4)->ShowWindow(TRUE);
		GetDlgItem(IDC_EDIT5)->ShowWindow(TRUE);
		GetDlgItem(IDC_EDIT6)->ShowWindow(TRUE);
		GetDlgItem(IDC_EDIT7)->ShowWindow(TRUE);
		GetDlgItem(IDC_EDIT10)->ShowWindow(TRUE);
		GetDlgItem(IDC_STATIC1)->ShowWindow(TRUE);
		GetDlgItem(IDC_STATIC2)->ShowWindow(TRUE);
		GetDlgItem(IDC_STATIC3)->ShowWindow(TRUE);
		GetDlgItem(IDC_STATIC4)->ShowWindow(TRUE);
		GetDlgItem(IDC_STATIC5)->ShowWindow(TRUE);

	}

	// TODO: Add your control notification handler code here	
	string curPath = getcwd(NULL, 0);
	m_FilePath = curPath + LocPath + rbtType + ".txt";

	if (File_Exist(m_FilePath))
	{
		MatrixXd joint = ReadThor_Joint(m_RobotType, m_RobotAxis, m_TotalNum, m_FilePath, true);

		m_RbtJoint = joint;

		CString showStr;
		for (int ii = 0; ii < joint.cols(); ii++)
		{
			for (int jj = 0; jj < joint.rows(); jj++)
			{
				str.Format(_T("%.3lf"), joint(jj, ii) / PI * 180);
				showStr += str + ',';
			}
			int end = showStr.GetLength() - 1;
			showStr = showStr.Left(end);

			m_List.SetItemText(ii, 1, showStr);
			showStr.Empty();
		}
	}
	else
	{
		m_List.DeleteAllItems();
		GetDlgItem(IDC_BUTTON1)->EnableWindow(false);
		MessageBox((CString)"找不到文件\n" + m_FilePath.c_str(), (CString)"错误", MB_OK);
		return;
	}
	GetDlgItem(IDC_BUTTON1)->EnableWindow(true);

	string msg = "Print Robot Cfg:\n";
	msg += (" Robot Type(0:Scara; 1:Delta; 2:SixAxis; 3:Cobot) = ") + to_string(m_RobotType) + "\n";
	msg += (" Robot Axis = ") + to_string(m_RobotAxis) + "\n";
	msg += (" Calbration location number = ") + to_string(m_TotalNum) + "\n";
	msg += (" Tool offset(length) = ") + to_string(m_ThisRbt.ToolOffset) + "\n";
	msg += (" mDH a = ") + rowToString(m_ThisRbtDH, 0) + "\n";
	msg += (" mDH alpha = ") + rowToString(m_ThisRbtDH, 2) + "\n";
	msg += (" mDH d = ") + rowToString(m_ThisRbtDH, 1) + "\n";
	msg += (" mDH theta = ") + rowToString(m_ThisRbtDH, 3) + "\n";
	msg += (" mDH beta = ") + rowToString(m_ThisRbtDH, 4) + "\n";
	msg += (" Move direction = ") + rowToString(m_ThisRbtDH, 5) + "\n";
	msg += (" Encoder line number = ") + rowToString(m_ThisRbtDH, 6) + "\n";
	WriteLog(-999999, const_cast<char*>(msg.data()));
}

//选择标定模式
void RopeEncoderCalibrationDlg::OnSelchangeCombo2()
{
	// TODO: Add your control notification handler code here
	// TODO: 在此添加控件通知处理程序代码
	CString selectString;

	int idx = m_CalibTypeCombo.GetCurSel();
	m_CalibTypeCombo.GetLBText(idx, selectString);
	m_CalibTypeCombo.SetCurSel(idx);
	if (selectString == _T("粗标"))
	{
		RoughCalibDlg.DoModal();
		m_CalibTypeCombo.SelectString(0, _T("拉线编码器"));
		//DeleteTemp();
	}

}

//判断编码器零点处输入是否为 int 
void RopeEncoderCalibrationDlg::OnEnChangeEdit1()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialogEx::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here

	//EditChange(GetDlgItem(IDC_EDIT1));
	CString EncoderValue;
	m_EncoderHomeStr.GetWindowTextW(EncoderValue);
	bool isInt = false;

	if (EncoderValue.SpanIncluding(_T("-0123456789")) == EncoderValue)
	{
		return;
	}
	else
	{
		MessageBox(_T("请输入整数"), _T("错误"), MB_OK);
	}

	while (!isInt)
	{
		EncoderValue = EncoderValue.Left(EncoderValue.GetLength() - 1);
		if (EncoderValue.SpanIncluding(_T("-0123456789")) == EncoderValue)
		{
			isInt = true;
		}
	}

	//m_EncoderHomeStr.SetSel(EncoderValue.GetLength(), EncoderValue.GetLength(), false);
	if (!isInt) return;

	m_EncoderHomeStr.SetWindowTextW(EncoderValue);
	m_EncoderHomeStr.SetSel(EncoderValue.GetLength(), EncoderValue.GetLength(), false);

}
#pragma endregion

#pragma region 界面函数

//QKMLink 发送函数
int RopeEncoderCalibrationDlg::SendCmd(CString msg) {
	Sleep(30);
	long lPacketID = QKMLinkSend(msg);

	return lPacketID;
}

//设置 Packetid
bool RopeEncoderCalibrationDlg::SetTargetPacketID(int target) {
	try
	{
		return QKMLinkSetTargetPacketID(target);
	}
	catch (const std::exception&)
	{
		return false;
	}
}
//解析固件版本
int RopeEncoderCalibrationDlg::DecodeRobotSystemVersion(CString tMessage)
{
	try
	{
		char spiltA = ',';
		char spiltB = ';';
		char spiltC = '_';
		char spiltD = 'V';
		char spiltE = '.';
		int tIdxA = 0;
		int tIdxB = 0;
		int tIdxC = 0;
		int tIdxD = 0;
		int tIdxE = 0;
		int tIdxF = 0;
		tIdxA = tMessage.Find(spiltA);
		tIdxB = tMessage.Find(spiltB);
		CString tMessageSubA = tMessage.Mid(tIdxA, tIdxB - tIdxA + 1);

		tIdxC = tMessageSubA.Find(spiltC);
		tIdxD = tMessageSubA.Find(spiltD);
		CString tMessageSubB = tMessageSubA.Mid(0, tIdxC + 1);
		tIdxE = tMessageSubA.Find(spiltE, tIdxD + 1);
		tIdxF = tMessageSubA.Find(spiltE, tIdxE + 1);
		CString tMessageSubC = tMessageSubA.Mid(tIdxD + 1, tIdxE - tIdxD - 1);
		CString tMessageSubD = tMessageSubA.Mid(tIdxE + 1, tIdxF - tIdxE - 1);

		return 0;
	}
	catch (const std::exception&)
	{
		return -1;
	}
}

//初始化发送的指令
void RopeEncoderCalibrationDlg::SendInit() {
	CString sSend;

	sSend = (CString)m_CMDLogin.c_str();
	SendCmd(sSend);

	//m_SystemVersionStr = _T("");
	//sSend = (CString)m_CMDSystemVersion.c_str();
	//TargetPacketID = SendCmd(sSend);
	//SetTargetPacketID(TargetPacketID);
	//m_SystemVersion = true;
	//long WaitSystemVersionFeedBackCount = 0;
	//while (true)
	//{
		//if(m_SystemVersionStr != _T(""))
		//{
		//	DecodeRobotSystemVersion(m_SystemVersionStr);
		//	break;
		//}
		//if (WaitSystemVersionFeedBackCount >= 100)
		//{
		//	break;
		//}
		//WaitSystemVersionFeedBackCount = WaitSystemVersionFeedBackCount + 1;
		//Sleep(40);
	//}

	if (iAPIVersionType == APIVersionType_1)
	{
		sSend = (CString)m_CMDAbroted.c_str();
		SendCmd(sSend);

		sSend = (CString)m_CMDStart.c_str();
		SendCmd(sSend);
	}
	else
	{
		sSend = (CString)m_CMDThreadAbroted.c_str();
		SendCmd(sSend);

		sSend = (CString)m_CMDThreadStart.c_str();
		SendCmd(sSend);

		sSend = (CString)m_CMDRobotAttached.c_str();
		SendCmd(sSend);
	}

	sSend = (CString)m_CMDClearVariables.c_str();
	SendCmd(sSend);

	sSend = (CString)m_CMDPowerON.c_str();
	SendCmd(sSend);

	sSend = (CString)m_CMDHome.c_str();
	SendCmd(sSend);

	sSend = (CString)(m_CMDDefineLoc + m_LocName).c_str();
	SendCmd(sSend);

	sSend = (CString)(m_CMDProfile).c_str();
	SendCmd(sSend);

	sSend = (CString)(m_CMDProfileSet).c_str();
	SendCmd(sSend);
}

//运动点位函数
void RopeEncoderCalibrationDlg::MoveLocation(int target)
{
	// 1.获取指令
	CString sSend, sResult;
	BSTR bsSend = NULL, bsCommand = NULL, bsState = NULL, bsSourceID = NULL, bsDestinationID = NULL;
	sSend = m_List.GetItemText(target, 1);

	sSend = (CString)m_LocName.c_str() + "=" + sSend;
	SendCmd(sSend);

	sSend = (CString)m_MoveCmd.c_str();
	TargetPacketID = SendCmd(sSend);
	SetTargetPacketID(TargetPacketID);
	m_MoveReceive = true;
}

//回零点运动函数
void RopeEncoderCalibrationDlg::MoveZero(CString csValue)
{
	// 1.获取指令
	CString sSend;

	if (iAPIVersionType == APIVersionType_1)
	{
		sSend = (CString)m_CMDAbroted.c_str();
		SendCmd(sSend);

		sSend = (CString)m_CMDStart.c_str();
		SendCmd(sSend);

		sSend = (CString)m_CMDContinue.c_str();
		SendCmd(sSend);
	}
	else
	{
		sSend = (CString)m_CMDThreadAbroted.c_str();
		SendCmd(sSend);

		sSend = (CString)m_CMDThreadStart.c_str();
		SendCmd(sSend);

		sSend = (CString)m_CMDRobotAttached.c_str();
		SendCmd(sSend);
	}

	sSend = (CString)m_CMDPowerON.c_str();
	SendCmd(sSend);

	sSend = (CString)m_LocName.c_str() + "=" + csValue;
	QKMLinkSend(sSend);
	sSend = (CString)m_MoveCmd.c_str();
	TargetPacketID = QKMLinkSend(sSend);
	QKMLinkSetTargetPacketID(TargetPacketID);
	m_MoveReceive = true;
	sSend = (CString)m_CMDWaitForEOM.c_str();
	QKMLinkSend(sSend);
}

//写 log
void RopeEncoderCalibrationDlg::WriteLog(long PacketID, char *logMsg) {
	SYSTEMTIME st;
	GetLocalTime(&st);

	fstream _file;
	_file.open("./log", ios::in);

	string time = std::to_string(st.wYear) + '_' + std::to_string(st.wMonth) + '_' + std::to_string(st.wDay);


	if (!_file) {
		_mkdir("./log");
	}
	_file.close();

	FILE *fp;

	string path = "./log/" + time + "_log.txt";

	fopen_s(&fp, path.c_str(), "at");
	fprintf(fp, "Time:%d:%d:%d:%d  ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
	fprintf(fp, "[%d] %s", PacketID, logMsg);
	fprintf(fp, "\n");
	fclose(fp);
}
#pragma endregion

// 分割字符串的辅助函数
std::vector<std::string> split(const std::string& s, char delimiter) {
	std::vector<std::string> tokens;
	std::string token;
	std::stringstream tokenStream(s);
	while (std::getline(tokenStream, token, delimiter)) {
		tokens.push_back(token);
	}
	return tokens;
}

// ================ 添加 ReadIDNValue 函数实现 ================
std::string RopeEncoderCalibrationDlg::ReadIDNValue(CString IDNModule, CString IDNIndex, CString arrIndex)
{
	CString sSend = JointSystemIDNReadCmdStr(IDNModule, IDNIndex, arrIndex);
	GeneralWaitValue(sSend, 5000);
	// 按空格分割字符串，得到 ["0", "-400.000"]
	std::vector<std::string> parts = split(m_sGeneralWaitValue, ' ');
	string sResult;
	// 提取第二个元素
	if (parts.size() >= 2)
	{
		sResult = parts[1];
	}
	return sResult;
}

// ================ 添加 GeneralWaitValue 函数实现 ================
std::string RopeEncoderCalibrationDlg::GeneralWaitValue(CString sSend, int iWaitTime)
{
	m_bGeneralWaitValue = true;
	m_sGeneralWaitValue = "";
	TargetPacketID = SendCmd(sSend);
	SetTargetPacketID(TargetPacketID);

	/*
	if (WAIT_TIMEOUT == WaitForSingleObject(m_IDNEvent, iWaitTime)) {
		return "";
	}
	*/
	// 使用 MsgWaitForMultipleObjects 可以同时等待对象和消息
	while (TRUE)
	{
		DWORD dwResult = MsgWaitForMultipleObjects(
			1, &m_IDNEvent, FALSE, iWaitTime, QS_ALLINPUT);

		if (dwResult == WAIT_OBJECT_0)  // 事件已触发
		{
			break;
		}
		else if (dwResult == WAIT_OBJECT_0 + 1)  // 有消息到达
		{
			MSG msg;
			while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}
		else if (dwResult == WAIT_TIMEOUT)  // 超时
		{
			break;
		}
	}

	ResetEvent(m_IDNEvent);
	return m_sGeneralWaitValue;
}

// ================ 添加 WriteIDNValue 函数实现 ================
int RopeEncoderCalibrationDlg::WriteIDNValue(CString IDNModule, CString IDNIndex, CString value, CString arrIndex)
{
	CString sSend = JointSystemIDNWriteCmdStr(IDNModule, IDNIndex, value, arrIndex);
	GeneralWaitValue(sSend, 60000);
	int iStt = -1;
	if(m_sGeneralWaitValue == "0")
	{
		iStt = 0;
	}
	return iStt;
}

// ================ 添加 WriteIDNValue 函数实现 ================
int RopeEncoderCalibrationDlg::PowerOff()
{
	GeneralWaitValue((CString)m_CMDPowerOff.c_str(), 5000);
	int iStt = -1;
	if (m_sGeneralWaitValue == "0")
	{
		iStt = 0;
	}
	return iStt;
}

#pragma region 事件

//显示事件,用于在子线程通过事件的方式更新 listcontrol 里的值
afx_msg LRESULT RopeEncoderCalibrationDlg::OnDisplayChange(WPARAM wParam, LPARAM lParam)
{
	CString listResult;
	//284.94 是拉线编码器的 mm 转换值
	double EncoderValue = (lParam - m_EncoderHomeValue) / 284.94;
	listResult.Format(_T("%lf"), EncoderValue);
	m_List.SetItemText(wParam, 2, listResult);

	m_List.SetSelectionMark(wParam);
	m_List.SetFocus();
	m_List.EnsureVisible(wParam, FALSE);
	if (wParam > 0) {
		m_List.SetItemState(wParam - 1, 0, LVIS_SELECTED | LVIS_FOCUSED);
	}
	m_List.SetItemState(wParam, LVIS_SELECTED | LVIS_FOCUSED, LVIS_SELECTED | LVIS_FOCUSED);

	return 0;
}

//机器人运动事件
afx_msg LRESULT RopeEncoderCalibrationDlg::OnRobotMove(WPARAM wParam, LPARAM lParam)
{
	MoveLocation(wParam);
	return 0;
}

//机器人运动回零点事件,先四五六轴回零,然后二轴回零,再三轴回零,最后一轴回零
afx_msg LRESULT RopeEncoderCalibrationDlg::OnMoveZero(WPARAM wParam, LPARAM lParam)
{
	CString* pTry = (CString*)lParam;
	CString LocationValue = *pTry;

	MoveZero(LocationValue);
	return 0;
}

//按钮使能事件
afx_msg LRESULT RopeEncoderCalibrationDlg::OnEnablebtn(WPARAM wParam, LPARAM lParam)
{
	m_EncoderHomeStr.EnableWindow(m_IsConnect);
	m_StartBtn.EnableWindow(m_IsConnect);
	m_StopBtn.EnableWindow(!m_IsConnect);
	m_ContinueBtn.EnableWindow(!m_IsConnect);
	m_PauseBtn.EnableWindow(!m_IsConnect);
	m_CalibTypeBtn.EnableWindow(m_IsConnect);
	m_EncoderBtn.EnableWindow(m_IsConnect);
	if (lParam == NULL) {
		MessageBox(m_WhereTimeoutStr, m_ErrorStr, MB_OK);
		OnBnClickedButton5();
		return -1;
	}
	m_CalibBtn.EnableWindow(m_IsConnect);
	//m_SaveBtn.EnableWindow(m_IsConnect);
	m_LocationZero.EnableWindow(m_IsConnect);
	m_DisconnectBtn.EnableWindow(m_IsConnect);
	return 0;
}

//保存机器人实际点位事件
afx_msg LRESULT RopeEncoderCalibrationDlg::SaveRobotJoint(WPARAM wParam, LPARAM lParam)
{
	CString selectitem;
	m_RbtTypeCombo.GetLBText(m_RbtTypeCombo.GetCurSel(), selectitem);
	string rbtType = CT2A(selectitem.GetBuffer(0));

	time_t now_time = time(NULL);
	tm t;
	time(&now_time);
	localtime_s(&t, &now_time);
	m_FinishTime = std::to_string(t.tm_year + 1900) + '_' + std::to_string(t.tm_mon + 1) + '_' + std::to_string(t.tm_mday) + '_' + std::to_string(t.tm_hour) + '_' + std::to_string(t.tm_min);

	SaveJoint(rbtType, m_FinishTime);

	CString *list = new CString[m_TotalNum];

	for (int ii = 0; ii < m_TotalNum; ii++) {
		list[ii] = m_List.GetItemText(ii, 2);
	}

	SaveData(list, rbtType, m_FinishTime);
	SaveR5SampleStats(rbtType, m_FinishTime);

	return 0;
}

//显示事件,用于在子线程通过事件的方式更新 listcontrol 里的值
afx_msg LRESULT RopeEncoderCalibrationDlg::MessageBoxShow(WPARAM wParam, LPARAM lParam)
{
	CString msgValue;
	if (wParam == m_MoveTimeout) {
		msgValue = m_MoveTimeoutStr;
	}
	else if (wParam == m_EncoderTimeout)
	{
		msgValue = m_EncoderTimeoutStr;
	}
	else if (wParam == m_EncoderConnectError)
	{
		msgValue = m_EncoderConnectErrorStr;
		OnBnClickedButton11();
	}

	MessageBox(msgValue, m_ErrorStr, MB_OK);

	return 0;
}

//获取实际机器人点位
afx_msg LRESULT RopeEncoderCalibrationDlg::GetRobotWhereAngle(WPARAM wParam, LPARAM lParam)
{
	CString sSend;

	sSend = (CString)m_CMDRobotWhereAngle.c_str();
	TargetPacketID = SendCmd(sSend);
	SetTargetPacketID(TargetPacketID);
	m_Where = true;

	return 0;
}

//关闭事件
afx_msg LRESULT RopeEncoderCalibrationDlg::OnClosing(WPARAM wParam, LPARAM lParam)
{
	m_ThreadRun = false;
	m_KeepConnect = false;
	endpoint.dispose();

	PostMessage(WM_QUIT, NULL, NULL);
	return 0;
}

//点击停止后,使能按钮事件
afx_msg LRESULT RopeEncoderCalibrationDlg::StartButtonEnable(WPARAM wParam, LPARAM lParam)
{
	m_StartBtn.EnableWindow(m_IsConnect);
	m_LocationZero.EnableWindow(m_IsConnect);
	m_CalibTypeBtn.EnableWindow(m_IsConnect);
	m_EncoderBtn.EnableWindow(m_IsConnect);
	m_ContinueBtn.EnableWindow(m_IsConnect);
	m_StopBtn.EnableWindow(!m_IsConnect);
	m_DisconnectBtn.EnableWindow(m_IsConnect);
	return 0;
}

//log 接受信息
afx_msg LRESULT RopeEncoderCalibrationDlg::ReceiveLog(WPARAM wParam, LPARAM lParam) {
	CString* pTry = (CString*)lParam;

	CString asd = *pTry;

	USES_CONVERSION;

	char* msg = T2A(asd);

	WriteLog((long)wParam, msg);
	return 0;
}

//替换 xml 值
afx_msg LRESULT RopeEncoderCalibrationDlg::OnReplaceXmlValue(WPARAM wParam, LPARAM lParam) {
	CString targetIDN;
	CString IDNValue;

	CString* pTry = (CString*)wParam;

	targetIDN = *pTry;

	CString* pTry1 = (CString*)lParam;

	IDNValue = *pTry1;

	ReplaceXmlValue(targetIDN, IDNValue);
	return 0;
}

//写 xml 值
afx_msg LRESULT RopeEncoderCalibrationDlg::OnWriteXml(WPARAM wParam, LPARAM lParam) {

	WriteXml();
	//Ftp 默认端口号 21
	int stt = UpLoad(m_RbtIPStr, 21, _T(""), _T(""));
	if (stt == 0) {
		MessageBox(_T("保存成功"), _T("提示"), MB_OK);
	}
	return 0;
}
#pragma endregion

#pragma region 线程
//回零点线程
UINT RopeEncoderCalibrationDlg::ZeroThread(LPVOID pParam) {
	try
	{
		CString sSend;

		RopeEncoderCalibrationDlg* pDlg;
		pDlg = (RopeEncoderCalibrationDlg*)pParam;

		if (WAIT_TIMEOUT == WaitForSingleObject(m_WhereEvent, m_ShortWaitTime))
		{
			return -1;
		}

		ResetEvent(m_WhereEvent);
		m_LocIdx = 0;
		CString *csvValue = &m_RobotHere[0];
		CString strvValue = *csvValue;
		CString AxisValue;
		double Axis[6];
		char spilt = ' ';
		int sIdx = 0;
		int eIdx = 0;
		int errCode = 0;
		strvValue.Left(strvValue.Find(spilt)).Format(_T("%d"), errCode);
		if (errCode < 0) {
			return -2;
		}

		strvValue += spilt;
		for (int ii = 0; ii < m_RobotAxis; ii++)
		{
			eIdx = strvValue.Find(spilt, sIdx + 1);
			AxisValue = strvValue.Mid(sIdx, eIdx - sIdx);
			Axis[ii] = atof(CT2A(AxisValue.GetBuffer(0)));

			sIdx = eIdx + 1;
		}

		if (m_RobotType == Cobot) 
		{
			Axis[3] = 0;
			Axis[4] = 0;
			Axis[5] = 0;
			AxisValue = TurnDoubleToLoc(Axis);
			SendMessageA(pDlg->m_hWnd, WM_MOVE_ZERO, NULL, (LPARAM)&AxisValue);

			if (WAIT_TIMEOUT == WaitForSingleObject(m_MoveEvent, m_WaitTime))
			{
				return -3;
			}
			ResetEvent(m_MoveEvent);

			Axis[1] = 0;
			AxisValue = TurnDoubleToLoc(Axis);
			SendMessageA(pDlg->m_hWnd, WM_MOVE_ZERO, NULL, (LPARAM)&AxisValue);

			if (WAIT_TIMEOUT == WaitForSingleObject(m_MoveEvent, m_WaitTime))
			{
				return -3;
			}
			ResetEvent(m_MoveEvent);

			Axis[2] = 0;
			AxisValue = TurnDoubleToLoc(Axis);
			SendMessageA(pDlg->m_hWnd, WM_MOVE_ZERO, NULL, (LPARAM)&AxisValue);

			if (WAIT_TIMEOUT == WaitForSingleObject(m_MoveEvent, m_WaitTime))
			{
				return -3;
			}
			ResetEvent(m_MoveEvent);

			Axis[0] = 0;
			AxisValue = TurnDoubleToLoc(Axis);
			SendMessageA(pDlg->m_hWnd, WM_MOVE_ZERO, NULL, (LPARAM)&AxisValue);

			if (WAIT_TIMEOUT == WaitForSingleObject(m_MoveEvent, m_WaitTime))
			{
				return -3;
			}
			ResetEvent(m_MoveEvent);
		}
		else if(m_RobotType == Scara)
		{
			Axis[3] = 0;
			Axis[2] = 0;
			AxisValue = TurnDoubleToLoc(Axis);
			SendMessageA(pDlg->m_hWnd, WM_MOVE_ZERO, NULL, (LPARAM)&AxisValue);

			if (WAIT_TIMEOUT == WaitForSingleObject(m_MoveEvent, m_WaitTime))
			{
				return -3;
			}
			ResetEvent(m_MoveEvent);

			Axis[1] = 0;
			Axis[0] = 0;
			AxisValue = TurnDoubleToLoc(Axis);
			SendMessageA(pDlg->m_hWnd, WM_MOVE_ZERO, NULL, (LPARAM)&AxisValue);
			if (WAIT_TIMEOUT == WaitForSingleObject(m_MoveEvent, m_WaitTime))
			{
				return -3;
			}
			ResetEvent(m_MoveEvent);
		}
		m_MoveEvent = NULL;
		m_WhereEvent = NULL;
	}
	catch (const std::exception&)
	{

	}



	return 0;
}

//保持 websocket 连接的线程,在空闲时保持发送指令
UINT RopeEncoderCalibrationDlg::KeepConnecting(LPVOID pParam)
{
	RopeEncoderCalibrationDlg* pDlg;
	pDlg = (RopeEncoderCalibrationDlg*)pParam;
	Sleep(20);
	while (m_KeepConnect) {
		int stt = m_Client.Send(CmdStr);
		if (stt < 0) {
			PostMessageA(pDlg->m_hWnd, WM_TIME_OUT, (WPARAM)m_EncoderConnectError, (LPARAM)m_EncoderConnectError);
			break;
		}
		//int stt = m_Client.Send(CmdStr);
		Sleep(1000);
		m_Client.Clear();
	}

	return 0;
}

//运动线程,用于标定运动的整个流程
UINT RopeEncoderCalibrationDlg::MyThreadFunction(LPVOID pParam)
{
	try
	{

		int EncoderValue = 0;
		RopeEncoderCalibrationDlg* pDlg;
		pDlg = (RopeEncoderCalibrationDlg*)pParam;
		int startidx = m_LocIdx;

		for (int i = startidx; i < m_TotalNum; i++)
		{
			if (!m_ThreadRun) {
				PostMessageA(pDlg->m_hWnd, WM_BUTTON_ENABLE, (WPARAM)i, (LPARAM)i);
				return 0;
			}
			ResumeThread(KeepConnectThread->m_hThread);
			PostMessageA(pDlg->m_hWnd, WM_ROBOT_MOVE, (WPARAM)i, (LPARAM)i);
			if (WAIT_TIMEOUT == WaitForSingleObject(m_MoveEvent, m_WaitTime)) {
				if (m_ThreadRun) {
					PostMessageA(pDlg->m_hWnd, WM_TIME_OUT, (WPARAM)m_MoveTimeout, (LPARAM)m_MoveTimeout);
				}
				PostMessageA(pDlg->m_hWnd, WM_BUTTON_ENABLE, (WPARAM)i, (LPARAM)i);
				return -1;
			}
			Sleep(m_ShortWaitTime);
			SuspendThread(KeepConnectThread->m_hThread);

			m_ErrorTime = 0;
			EncoderValue = GetEncoderValue();
			if (EncoderValue == -1) {
				PostMessageA(pDlg->m_hWnd, WM_TIME_OUT, (WPARAM)m_EncoderTimeout, (LPARAM)m_EncoderTimeout);
				PostMessageA(pDlg->m_hWnd, WM_BUTTON_ENABLE, (WPARAM)i, (LPARAM)i);
				return -1;
			}
			if (i < static_cast<int>(m_R5EncoderMeanCounts.size()))
			{
				m_R5EncoderMeanCounts[static_cast<size_t>(i)] = m_R5LastEncoderMeanCounts;
				m_R5EncoderStdCounts[static_cast<size_t>(i)] = m_R5LastEncoderStdCounts;
			}
			CString sampleLog;
			sampleLog.Format(_T("R5 point=%d samples=%d mean_counts=%.6f rounded_counts=%d std_counts=%.6f std_mm=%.6f\n"),
				i + 1, m_R5SamplesPerPose, m_R5LastEncoderMeanCounts, EncoderValue,
				m_R5LastEncoderStdCounts, m_R5LastEncoderStdCounts / 284.94);
			OutputDebugString(sampleLog);

			PostMessageA(pDlg->m_hWnd, WM_GET_ROBOT_JOINT, (WPARAM)i, (LPARAM)EncoderValue);
			if (WAIT_TIMEOUT == WaitForSingleObject(m_WhereEvent, m_ShortWaitTime)) {
				if (m_ThreadRun) {
					PostMessageA(pDlg->m_hWnd, WM_TIME_OUT, (WPARAM)m_EncoderTimeout, (LPARAM)m_EncoderTimeout);
				}
				PostMessageA(pDlg->m_hWnd, WM_BUTTON_ENABLE, (WPARAM)i, (LPARAM)i);
				return -1;
			}

			ResetEvent(m_WhereEvent);
			ResetEvent(m_MoveEvent);
			PostMessageA(pDlg->m_hWnd, WM_DISPLAY_CHANGE, (WPARAM)i, (LPARAM)EncoderValue);

		}

		ResumeThread(KeepConnectThread->m_hThread);
		PostMessageA(pDlg->m_hWnd, WM_ENABLE_BTN, (WPARAM)NULL, (LPARAM)EncoderValue);
		PostMessageA(pDlg->m_hWnd, WM_SAVE_JOINT, (WPARAM)NULL, (LPARAM)EncoderValue);

		return 0;
	}
	catch (const std::exception&)
	{
		return -1;
	}
	OutputDebugString(_T("break"));
}
#pragma endregion
