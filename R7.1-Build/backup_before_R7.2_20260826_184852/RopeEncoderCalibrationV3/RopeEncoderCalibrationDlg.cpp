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
#include <io.h>

using namespace std;
using namespace Eigen;
//websocket 的应用
using namespace kagula;
#define PI 3.1415926

#pragma region 全局变量
//软件版本
CString m_Version = _T(" HE3 V3 Analytic R7.1 Public MDH + Rope Bias 2026.08.26");
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
// Controller stores the legacy/SDH-style alpha sequence in 1101.12.  The
// analytic solver consumes Craig MDH and converts this sequence on readback.
CString m_IDNAlpha = _T("1101.12");
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
// R6.3: the R6.2 field log showed an average per-pose standard deviation of
// only 0.000074 mm and a maximum of 0.001746 mm. Five reads keep a very large
// noise margin while cutting the encoder wait to one quarter of R6.2.
const int m_R5SamplesPerPose = 5;
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
CalibrationV3Report m_R6LastReport = {};
bool m_R63FeedbackValidationRunning = false;
double m_R61Q6SensitivityMmPerDeg = 0.0;
double m_R61Alpha6SensitivityMmPerDeg = 0.0;
double m_R61Q7SensitivityMmPerDeg = 0.0;
double m_R61Q6Alpha6Cosine = 0.0;
bool m_R61ObservabilityReady = false;
bool m_R61Q6CandidateActive = false;
// R6.5 controller snapshot.  These values are the authoritative before-write
// values and are refreshed after a successful persistent save.
std::vector<double> m_R65ControllerDmm;
std::vector<double> m_R65ControllerLegacyAlphaDeg;
std::vector<double> m_R65ControllerZeroCounts;
bool m_R65ControllerSnapshotValid = false;
CString m_R65InputSource = _T("公共HE3_GY-cfg.txt（Craig MDH）");
// R7.1 preserves the file-loaded public model independently from controller readback.
// Every acquisition restores this immutable public MDH immediately before motion.
MatrixXd m_R7OriginalConfigDH;
bool m_R7OriginalConfigValid = false;
CString m_R7OriginalConfigSource = _T("公共MDH（Config/RobotType/HE3_GY-cfg.txt）");
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
	DDX_Control(pDX, IDC_EDIT12, m_RopeBias);
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

	// R7.1 keeps the complete V1 operation area and adds a read-only monitor on
	// the right. The monitor never sends a robot command.
	CRect originalClient;
	GetClientRect(&originalClient);
	CRect originalWindow;
	GetWindowRect(&originalWindow);
	const int monitorWidth = 540;
	SetWindowPos(NULL, 0, 0, originalWindow.Width() + monitorWidth,
		originalWindow.Height(), SWP_NOMOVE | SWP_NOZORDER);
	CRect monitorRect(originalClient.right + 12, 12,
		originalClient.right + monitorWidth - 12, originalClient.bottom - 12);
	m_R6Monitor.Create(WS_CHILD | WS_VISIBLE | WS_BORDER | WS_VSCROLL |
		ES_MULTILINE | ES_AUTOVSCROLL | ES_READONLY | ES_LEFT,
		monitorRect, this, IDC_R6_MONITOR);
	m_R6Monitor.SetFont(GetFont());
	m_R6Monitor.SetLimitText(1024 * 1024);

	// Keep the two non-motion diagnostic buttons beside the V1 result fields.
	CRect beforeRect;
	m_ErrorBefore.GetWindowRect(&beforeRect);
	ScreenToClient(&beforeRect);
	const int buttonLeft = beforeRect.right + 10;
	const int buttonWidth = 105;
	m_R6FeedbackBtn.Create(_T("复输入检验"), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
		CRect(buttonLeft, beforeRect.top, buttonLeft + buttonWidth, beforeRect.bottom),
		this, IDC_R6_FEEDBACK);
	CRect afterRect;
	m_ErrorAfter.GetWindowRect(&afterRect);
	ScreenToClient(&afterRect);
	m_R6RestoreBtn.Create(_T("恢复初始"), WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
		CRect(buttonLeft, afterRect.top, buttonLeft + buttonWidth, afterRect.bottom),
		this, IDC_R6_RESTORE);
	m_R6FeedbackBtn.EnableWindow(FALSE);
	m_R6RestoreBtn.EnableWindow(FALSE);
	AppendR6Monitor(_T("R7.1闭环监控已启动。算法输入固定为公共 Config/RobotType/HE3_GY-cfg.txt；控制器回读仅用于写回前后核对。拉线零偏 bL 独立求解，不写入MDH。"), false);

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
	ShowR65ModelContract(_T("软件启动"));
	ShowLastCalibrationSummary();

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
	// R7.1 reads the controller before the keep-alive thread starts only as a
	// diagnostic/writeback snapshot. It never replaces the public calibration seed.
	ReloadControllerModelAsInput(_T("连接成功后的控制器只读快照"), false);
	m_KeepConnect = true;
	KeepConnectThread = AfxBeginThread(RopeEncoderCalibrationDlg::KeepConnecting, (LPVOID)this);
	
	m_EncoderHomeStr.EnableWindow(true);
	m_RbtTypeCombo.EnableWindow(false);
}

//开始按钮,开始进行标定动作
void RopeEncoderCalibrationDlg::OnBnClickedButton2()
{
	if (!PrepareR71PublicInputBeforeStart())
	{
		AppendR6Monitor(_T("R7.1本次采集已取消：公共HE3_GY-cfg.txt输入未通过准备检查。"));
		return;
	}
	ShowR65ModelContract(_T("本次采集开始"));
	ShowCurrentMdh(_T("开始运动采集前"));
	CString startInfo;
	startInfo.Format(_T("准备采集：点位=%d，每点拉线读取=%d次，间隔=%lu ms。"),
		m_TotalNum, m_R5SamplesPerPose, static_cast<unsigned long>(m_R5SampleIntervalMs));
	AppendR6Monitor(startInfo);
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

	// R6.3 production layout uses 82 solving points:
	//   1..60  original V1 production points
	//   61..82 independent J2..J5 excitation points
	// R6.2 points 61..78 were one-time fixture/repeat diagnostics and never
	// entered the solve. They are removed from the production run.
	std::vector<int> calibrationIndices;
	calibrationIndices.reserve(static_cast<size_t>(sampleCount));
	for (int sample = 0; sample < sampleCount; ++sample)
		calibrationIndices.push_back(sample);
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

	// The V1 GUI has already added the nominal 62.5 mm for display/calculation.
	// Remove it before calling V3, then pass the same nominal fixed offset exactly
	// once. R7.1 solves any remaining common absolute-length error as bL; it does
	// not rewrite the 62.5 mm constant or hide the bias inside robot MDH.
	std::vector<double> ropeRawM(calibrationSampleCount);
	for (int calibrationSample = 0; calibrationSample < calibrationSampleCount; ++calibrationSample)
	{
		const int sample = calibrationIndices[static_cast<size_t>(calibrationSample)];
		ropeRawM[calibrationSample] = ropeWithFixedOffsetM(sample) - kRopeFixedOffsetM;
	}

	const double fixtureLocalXYZ[3] = { 0.0, 0.0, 0.0 };

	// R6.3 HE3 profile, aligned with the eight laser-calibration files:
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

	// R6.3 observability evidence from the exact analytic point Jacobian.
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
	line.Format(_T("R7.1 Craig MDH解析雅可比 + 公共MDH固定输入 + 独立拉线零偏bL + 写回闭环\r\n轴数=%d 记录=%d\r\n"),
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
		line.Format(_T("\r\n[轮次 %d] scale=%.5g 几何MAE=%.6f mm MAX=%.6f mm bL=%+.6f mm\r\n"),
			rec.iteration, rec.stepScale, rec.maeM * 1000.0, rec.maxAbsM * 1000.0, rec.ropeBiasM * 1000.0);
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
		line.Format(_T("anchor(m): %.7f, %.7f, %.7f\r\n"),
			rec.anchor[0], rec.anchor[1], rec.anchor[2]);
		text += line;
		text += _T("本轮实际应用Δd(mm): ");
		for (int joint = 0; joint < trace.axis; ++joint)
		{
			line.Format(_T("%.6f%s"), rec.appliedDelta4N[trace.axis + joint] * 1000.0,
				joint + 1 == trace.axis ? _T("\r\n") : _T(", "));
			text += line;
		}
		text += _T("本轮实际应用Δalpha(deg): ");
		for (int joint = 0; joint < trace.axis; ++joint)
		{
			line.Format(_T("%.7f%s"), rec.appliedDelta4N[2 * trace.axis + joint] * 180.0 / PI,
				joint + 1 == trace.axis ? _T("\r\n") : _T(", "));
			text += line;
		}
		text += _T("本轮实际应用Δq0(deg): ");
		for (int joint = 0; joint < trace.axis; ++joint)
		{
			line.Format(_T("%.7f%s"), rec.appliedDelta4N[3 * trace.axis + joint] * 180.0 / PI,
				joint + 1 == trace.axis ? _T("\r\n") : _T(", "));
			text += line;
		}
	}
	return text;
}

void RopeEncoderCalibrationDlg::AppendR6Monitor(const CString& text, bool mirrorToFile)
{
	SYSTEMTIME st = {};
	GetLocalTime(&st);
	CString entry;
	entry.Format(_T("[%02d:%02d:%02d.%03d] "), st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
	entry += text;
	if (entry.GetLength() < 2 || entry.Right(2) != _T("\r\n")) entry += _T("\r\n");

	if (::IsWindow(m_R6Monitor.GetSafeHwnd()))
	{
		const int currentLength = m_R6Monitor.GetWindowTextLength();
		if (currentLength > 900000)
		{
			m_R6Monitor.SetSel(0, 300000);
			m_R6Monitor.ReplaceSel(_T("[较早日志已截断]\r\n"), FALSE);
		}
		m_R6Monitor.SetSel(-1, -1);
		m_R6Monitor.ReplaceSel(entry, FALSE);
		m_R6Monitor.LineScroll(m_R6Monitor.GetLineCount());
	}

	if (mirrorToFile)
	{
		CT2A converted(entry.GetString());
		std::string line(converted);
		WriteLog(-600400, const_cast<char*>(line.c_str()));
	}
}

void RopeEncoderCalibrationDlg::ShowCurrentMdh(const CString& operation)
{
	CString text, line;
	line.Format(_T("=== %s：当前计算输入（来源=%s，不是尚未保存的候选结果） ===\r\n"),
		operation.GetString(), m_R65InputSource.GetString());
	text += line;
	line.Format(_T("轴数=%d，点位=%d，ToolOffset=%.3f mm\r\n"),
		m_RobotAxis, m_TotalNum, m_ThisRbt.ToolOffset * 1000.0);
	text += line;

	const TCHAR* names[5] = { _T("a(mm)"), _T("d(mm)"), _T("alpha(deg)"), _T("theta/q0(deg)"), _T("beta(deg)") };
	for (int row = 0; row < 5 && row < m_ThisRbtDH.rows(); ++row)
	{
		text += names[row];
		text += _T(": ");
		for (int joint = 0; joint < m_RobotAxis; ++joint)
		{
			double value = m_ThisRbtDH(row, joint);
			if (row <= 1) value *= 1000.0;
			else value *= 180.0 / PI;
			line.Format(_T("%.5f%s"), value, joint + 1 == m_RobotAxis ? _T("\r\n") : _T(", "));
			text += line;
		}
	}
	AppendR6Monitor(text);
}

void RopeEncoderCalibrationDlg::ShowR65ModelContract(const CString& operation)
{
	CString text, line;
	line.Format(_T("=== R7.1模型契约 / %s ===\r\n"), operation.GetString());
	text += line;
	text += _T("算法输入判定：Craig MDH；alpha槽位=[0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6]。\r\n");
	text += _T("控制器存储判定：1101.12为旧式DH/SDH alpha序列=[alpha1..alpha6, alpha7]；读取后右移一槽转换为Craig MDH。\r\n");
	text += _T("激光方案允许参数：D2~D5、Alpha1~Alpha7、Theta/零位q2~q6。\r\n");
	text += _T("拉线实际求解：D2~D5、Alpha1~Alpha6、q2~q6（q6仅在解析列可见时参与）；另独立求解拉线零偏bL。\r\n");
	text += _T("R7.1输入固定：公共 Config/RobotType/HE3_GY-cfg.txt；不使用控制器回读MDH作为算法初值。\r\n");
	text += _T("测量模型：L实测 = |P-lambda| + bL；bL只校准拉线测量链，不进入4N机器人参数，也不写入控制器MDH。\r\n");
	text += _T("结构保持：A1~A7、D1/D6/D7、q1/q7、Beta；Alpha7虽属激光候选，但当前轴向拉线点对其为结构零列，保留控制器值。\r\n");
	line.Format(_T("末端模型：控制器D7结构长度 + ToolOffset %.3f mm = 算法有效D7；ToolOffset不重复写入控制器。\r\n"),
		m_ThisRbt.ToolOffset * 1000.0);
	text += line;
	AppendR6Monitor(text);
}

bool RopeEncoderCalibrationDlg::ReadControllerArray(const CString& idn, int count,
	std::vector<double>& values, const CString& label)
{
	values.assign(static_cast<size_t>(count), 0.0);
	for (int i = 0; i < count; ++i)
	{
		CString index;
		index.Format(_T("%d"), i + 1);
		const std::string raw = ReadIDNValue(idn, m_IndexStr, index);
		if (!isNum(raw))
		{
			CString failure;
			failure.Format(_T("控制器读取失败：%s IDN=%s[%d] 返回='%S'。保留现有本地模型。"),
				label.GetString(), idn.GetString(), i + 1, raw.c_str());
			AppendR6Monitor(failure);
			return false;
		}
		values[static_cast<size_t>(i)] = atof(raw.c_str());
	}
	return true;
}

static CString FormatR65Vector(const std::vector<double>& values, int precision)
{
	CString text, item;
	for (size_t i = 0; i < values.size(); ++i)
	{
		item.Format(precision <= 0 ? _T("%.0f%s") : _T("%.5f%s"), values[i],
			i + 1 == values.size() ? _T("") : _T(", "));
		text += item;
	}
	return text;
}

bool RopeEncoderCalibrationDlg::ReloadControllerModelAsInput(const CString& reason, bool updateInput)
{
	if (!m_IsConnect || m_RobotAxis != 7)
	{
		AppendR6Monitor(_T("未读取控制器模型：机器人未连接或当前不是HE3七轴；继续使用本地Craig MDH配置。"));
		return false;
	}

	std::vector<double> dMm, alphaLegacyDeg, zeroCounts;
	if (!ReadControllerArray(m_IDND, 7, dMm, _T("D")) ||
		!ReadControllerArray(m_IDNAlpha, 7, alphaLegacyDeg, _T("Alpha")) ||
		!ReadControllerArray(m_IDNZeroEncoderValue, 7, zeroCounts, _T("零位")))
		return false;

	const double nominalAlpha[7] = { 90.0, -90.0, 90.0, -90.0, 90.0, -90.0, 0.0 };
	const bool plausibleD = std::fabs(dMm[0]) <= 5.0 && std::fabs(dMm[5]) <= 5.0 &&
		dMm[1] > 35.0 && dMm[1] < 65.0 &&
		dMm[2] > 470.0 && dMm[2] < 515.0 && dMm[3] > -145.0 && dMm[3] < -95.0 &&
		dMm[4] > 295.0 && dMm[4] < 340.0 && dMm[6] > 75.0 && dMm[6] < 110.0;
	bool plausibleAlpha = true;
	bool plausibleZero = true;
	for (int i = 0; i < 7; ++i)
	{
		plausibleAlpha = plausibleAlpha && std::fabs(alphaLegacyDeg[i] - nominalAlpha[i]) <= 8.0;
		plausibleZero = plausibleZero && std::fabs(zeroCounts[i]) <= 100000.0;
	}
	if (!plausibleD || !plausibleAlpha || !plausibleZero)
	{
		CString rejected;
		rejected.Format(_T("控制器快照未通过HE3模型合理性检查（D=%d Alpha=%d Zero=%d），不作为算法输入。R7要求D1/D6接近0 mm、D7为末端结构长度；可防止旧六轴写回把93 mm误写到D6。"),
			plausibleD ? 1 : 0, plausibleAlpha ? 1 : 0, plausibleZero ? 1 : 0);
		AppendR6Monitor(rejected);
		AppendR6Monitor(_T("回读D(mm): ") + FormatR65Vector(dMm, 5));
		AppendR6Monitor(_T("回读旧式Alpha(deg): ") + FormatR65Vector(alphaLegacyDeg, 5));
		AppendR6Monitor(_T("回读Zero(cnt): ") + FormatR65Vector(zeroCounts, 0));
		return false;
	}

	m_R65ControllerDmm = dMm;
	m_R65ControllerLegacyAlphaDeg = alphaLegacyDeg;
	m_R65ControllerZeroCounts = zeroCounts;
	m_R65ControllerSnapshotValid = true;
	if (updateInput)
	{
		for (int joint = 0; joint < 7; ++joint)
			m_ThisRbtDH(1, joint) = dMm[static_cast<size_t>(joint)] / 1000.0;
		m_ThisRbtDH(1, 6) += m_ThisRbt.ToolOffset;
		m_ThisRbtDH(2, 0) = 0.0;
		for (int slot = 1; slot < 7; ++slot)
			m_ThisRbtDH(2, slot) = alphaLegacyDeg[static_cast<size_t>(slot - 1)] * PI / 180.0;
		m_R65InputSource = _T("控制器回读 1101.11/1101.12/1127.13");
		m_R6HaveOriginalDH = false;
		m_R6HaveResult = false;
		m_R6FeedbackBtn.EnableWindow(FALSE);
		m_SaveBtn.EnableWindow(FALSE);
	}

	CString snapshot;
	snapshot.Format(_T("=== %s：控制器完整回读通过 ===\r\nD(mm): %s\r\n旧式Alpha(deg): %s\r\nZero(cnt): %s\r\n"),
		reason.GetString(), FormatR65Vector(dMm, 5).GetString(),
		FormatR65Vector(alphaLegacyDeg, 5).GetString(), FormatR65Vector(zeroCounts, 0).GetString());
	AppendR6Monitor(snapshot);
	if (updateInput)
	{
		ShowR65ModelContract(_T("控制器回读已转为算法输入"));
		ShowCurrentMdh(_T("控制器回读成为下一次原始输入"));
	}
	return true;
}

bool RopeEncoderCalibrationDlg::PrepareR71PublicInputBeforeStart()
{
	if (m_RobotAxis != 7)
	{
		AppendR6Monitor(_T("R7.1公共MDH固定输入仅对HE3七轴启用；当前按已加载配置继续。"));
		return true;
	}

	if (!m_R7OriginalConfigValid || m_R7OriginalConfigDH.rows() < 5 ||
		m_R7OriginalConfigDH.cols() != m_RobotAxis)
	{
		AppendR6Monitor(_T("R7.1公共MDH不可用：Config/RobotType/HE3_GY-cfg.txt启动快照无效，禁止运动。"));
		MessageBox(_T("公共 HE3_GY-cfg.txt 的启动快照无效，本次不会运动机器人。"),
			_T("R7.1：公共MDH不可用"), MB_OK | MB_ICONERROR);
		return false;
	}

	// R7.1 contract: every new acquisition starts from the immutable public
	// HE3_GY-cfg.txt snapshot. Controller readback remains diagnostic/writeback
	// evidence only and can never silently become the calibration seed.
	m_ThisRbtDH = m_R7OriginalConfigDH;
	m_R65InputSource = m_R7OriginalConfigSource + _T("（R7.1固定公共输入）");
	m_R6HaveOriginalDH = false;
	m_R6HaveResult = false;
	m_R6FeedbackBtn.EnableWindow(FALSE);
	m_SaveBtn.EnableWindow(FALSE);

	AppendR6Monitor(_T("R7.1已强制恢复公共HE3_GY-cfg.txt Craig MDH；控制器快照不会作为本次算法输入。"));
	ShowR65ModelContract(_T("R7.1固定公共MDH输入"));
	ShowCurrentMdh(_T("运动前公共MDH输入"));

	CString confirmation;
	confirmation.Format(_T("R7.1本次算法输入固定为：%s\r\n\r\n"
		"完整MDH已写入右侧日志。\r\n"
		"按‘确定’后开始运动；按‘取消’则不运动。"),
		m_R65InputSource.GetString());
	if (MessageBox(confirmation, _T("R7.1：确认公共MDH输入"),
		MB_OKCANCEL | MB_ICONWARNING) != IDOK)
		return false;

	AppendR6Monitor(_T("R7.1公共MDH输入已确认，允许开始本次运动采集。"));
	return true;
}

bool RopeEncoderCalibrationDlg::WriteControllerValueVerified(const CString& idn, int arrayIndex,
	double value, double tolerance, CString& failure)
{
	CString index, valueText;
	index.Format(_T("%d"), arrayIndex);
	valueText.Format(_T("%.10f"), value);
	if (WriteIDNValue(idn, m_IndexStr, valueText, index) != 0)
	{
		failure.Format(_T("写入失败：%s[%d]=%.10f。"), idn.GetString(), arrayIndex, value);
		return false;
	}
	const std::string raw = ReadIDNValue(idn, m_IndexStr, index);
	if (!isNum(raw) || std::fabs(atof(raw.c_str()) - value) > tolerance)
	{
		failure.Format(_T("回读校验失败：%s[%d] 写入=%.10f，回读='%S'。"),
			idn.GetString(), arrayIndex, value, raw.c_str());
		return false;
	}
	CString success;
	success.Format(_T("写入并回读通过：%s[%d] = %.10f。"), idn.GetString(), arrayIndex, value);
	AppendR6Monitor(success);
	return true;
}

bool RopeEncoderCalibrationDlg::ValidateR65Candidate(const std::vector<double>& dMm,
	const std::vector<double>& legacyAlphaDeg, const std::vector<double>& zeroCounts,
	CString& failure) const
{
	if (dMm.size() != 7 || legacyAlphaDeg.size() != 7 || zeroCounts.size() != 7)
	{
		failure = _T("候选数组不是7轴完整长度。");
		return false;
	}
	if (std::fabs(dMm[0]) > 5.0 || std::fabs(dMm[5]) > 5.0 ||
		dMm[6] < 75.0 || dMm[6] > 110.0)
	{
		failure = _T("D1/D6/D7不符合HE3七轴结构：D1/D6应接近0 mm，D7应为75~110 mm的本体末端长度（不含ToolOffset）。");
		return false;
	}
	// Expanded envelopes around the eight laser-calibrated HE3 machines.  They
	// stop a low-residual but wrong compensating model from being persisted.
	if (!(dMm[1] >= 47.25 && dMm[1] <= 49.50 &&
		dMm[2] >= 491.49 && dMm[2] <= 493.77 &&
		dMm[3] >= -122.44 && dMm[3] <= -119.14 &&
		dMm[4] >= 315.65 && dMm[4] <= 318.49))
	{
		failure = _T("D2~D5超出8台激光机型数据扩展包络，禁止写回。");
		return false;
	}
	const double nominalAlpha[7] = { 90.0, -90.0, 90.0, -90.0, 90.0, -90.0, 0.0 };
	for (int i = 0; i < 7; ++i)
	{
		if (std::fabs(legacyAlphaDeg[i] - nominalAlpha[i]) > 2.0)
		{
			failure.Format(_T("Alpha%d超出名义值±2度，禁止写回。"), i + 1);
			return false;
		}
		if (std::fabs(zeroCounts[i]) > 5000.0)
		{
			failure.Format(_T("q%d零位绝对值超过5000 count，禁止写回。"), i + 1);
			return false;
		}
	}
	return true;
}

void RopeEncoderCalibrationDlg::ShowLastCalibrationSummary()
{
	_finddata_t data = {};
	intptr_t handle = _findfirst("./Result/V3Diagnostic_*.csv", &data);
	if (handle == -1)
	{
		AppendR6Monitor(_T("历史精度：未找到既往V3Diagnostic文件；本次启动前真实精度需完成采集后计算。"), false);
		return;
	}
	std::string newestName;
	__time64_t newestTime = 0;
	do
	{
		const std::string name = data.name;
		if (name.find("_feedback2") == std::string::npos && data.time_write >= newestTime)
		{
			newestTime = data.time_write;
			newestName = name;
		}
	} while (_findnext(handle, &data) == 0);
	_findclose(handle);
	if (newestName.empty()) return;

	double before = std::numeric_limits<double>::quiet_NaN();
	double after = std::numeric_limits<double>::quiet_NaN();
	double maximum = std::numeric_limits<double>::quiet_NaN();
	double ropeBias = std::numeric_limits<double>::quiet_NaN();
	std::ifstream input(std::string("./Result/") + newestName);
	std::string row;
	while (std::getline(input, row))
	{
		const size_t comma = row.find(',');
		if (comma == std::string::npos) continue;
		const std::string key = row.substr(0, comma);
		const double value = atof(row.substr(comma + 1).c_str());
		if (key == "mae_before_bias_mm" || key == "mae_before_mm") before = value;
		else if (key == "geometry_mae_after_bias_mm" || key == "mae_after_mm") after = value;
		else if (key == "max_after_mm") maximum = value;
		else if (key == "rope_bias_mm") ropeBias = value;
	}
	CString summary;
	if (std::isfinite(ropeBias))
		summary.Format(_T("最近一次历史结果（%S，不代表本次启动实时值）：bL=%+.3f mm，去零偏前MAE %.3f mm，几何MAE %.3f mm，标定后MAX %.3f mm。"),
			newestName.c_str(), ropeBias, before, after, maximum);
	else
		summary.Format(_T("最近一次历史结果（%S，R7旧格式）：MAE %.3f -> %.3f mm，标定后MAX %.3f mm。"),
			newestName.c_str(), before, after, maximum);
	AppendR6Monitor(summary, false);
}

//计算按钮
void RopeEncoderCalibrationDlg::OnBnClickedButton7()
{
	ShowR65ModelContract(_T("本次标定计算"));
	ShowCurrentMdh(_T("标定计算前"));
	AppendR6Monitor(_T("开始用当前已采集关节角与拉线长度计算基线误差和修正参数。"));
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
		m_R6LastReport = v3Report;
		CString monitorResult;
		monitorResult.Format(_T("计算完成：status=%d，拉线零偏 bL=%+.3f mm，去零偏前MAE=%.3f mm，去零偏后几何MAE=%.3f mm，MAX %.3f -> %.3f mm，rank=%d/%d，condition=%.1f。"),
			v3Status, v3Report.ropeBiasM * 1000.0,
			v3Report.maeBeforeM * 1000.0, v3Report.maeAfterM * 1000.0,
			v3Report.maxAbsBeforeM * 1000.0, v3Report.maxAbsAfterM * 1000.0,
			v3Report.jacobianRank, v3Report.activeCount, v3Report.jacobianCondition);
		AppendR6Monitor(monitorResult);

		CString v3Log;
		v3Log.Format(
			_T("V3 build=%S status=%d iteration=%d active=%d rank=%d condition=%.6g bL(mm)=%+.6f MAE_before_bias(mm)=%.6f geometry_MAE_after_bias(mm)=%.6f MAX(mm)=%.6f -> %.6f\n"),
			CalibrationV3BuildId(), v3Status, v3Report.iterations,
			v3Report.activeCount, v3Report.jacobianRank, v3Report.jacobianCondition,
			v3Report.ropeBiasM * 1000.0, v3Report.maeBeforeM * 1000.0, v3Report.maeAfterM * 1000.0,
			v3Report.maxAbsBeforeM * 1000.0, v3Report.maxAbsAfterM * 1000.0);
		OutputDebugString(v3Log);
		m_R6FeedbackBtn.EnableWindow(m_R6HaveResult ? TRUE : FALSE);

		// Persist the V3 summary into the normal communication log and also
		// write a point-by-point CSV. The latter is required to distinguish a
		// global geometry error from one bad rope sample or one bad pose.
		std::string v3LogText = CT2A(v3Log.GetString());
		WriteLog(0, const_cast<char*>(v3LogText.c_str()));
		_mkdir("./Result");
		std::string diagnosticStamp = m_FinishTime.empty() ? "manual" : m_FinishTime;
		if (m_R63FeedbackValidationRunning) diagnosticStamp += "_feedback2";
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
			diagnostic << "condition_gate_max,3000\n";
			diagnostic << "augmented_nuisance_count,4\n";
			diagnostic << "rope_bias_mm," << v3Report.ropeBiasM * 1000.0 << "\n";
			diagnostic << "measurement_model,L_measured=norm(P-lambda)+bL\n";
			diagnostic << "rope_fixed_offset_nominal_mm,62.5\n";
			diagnostic << "rope_fixed_offset_effective_mm," << 62.5 - v3Report.ropeBiasM * 1000.0 << "\n";
			diagnostic << "mae_before_bias_mm," << v3Report.maeBeforeM * 1000.0 << "\n";
			diagnostic << "geometry_mae_after_bias_mm," << v3Report.maeAfterM * 1000.0 << "\n";
			// Legacy keys retained for existing tooling.
			diagnostic << "mae_before_mm," << v3Report.maeBeforeM * 1000.0 << "\n";
			diagnostic << "mae_after_mm," << v3Report.maeAfterM * 1000.0 << "\n";
			diagnostic << "max_before_mm," << v3Report.maxAbsBeforeM * 1000.0 << "\n";
			diagnostic << "max_after_mm," << v3Report.maxAbsAfterM * 1000.0 << "\n";
			diagnostic << "laser_reference_count,8\n";
			diagnostic << "laser_reference_source,2.HE3_laser_calibration_dataset\n";
			diagnostic << "laser_design_d_mm,0|49|493|-120|317|0|93\n";
			diagnostic << "controller_mdh_alpha_deg,0|90|-90|90|-90|90|-90\n";
				diagnostic << "legacy_sdh_alpha_deg,90|-90|90|-90|90|-90|0\n";
				diagnostic << "input_model_source," << static_cast<const char*>(CT2A(m_R65InputSource)) << "\n";
				diagnostic << "laser_allowed_parameters,D2|D3|D4|D5|Alpha1|Alpha2|Alpha3|Alpha4|Alpha5|Alpha6|Alpha7|q2|q3|q4|q5|q6\n";
				diagnostic << "rope_active_parameters,D2|D3|D4|D5|Alpha1|Alpha2|Alpha3|Alpha4|Alpha5|Alpha6|q2|q3|q4|q5";
				if (m_R61Q6CandidateActive) diagnostic << "|q6";
				diagnostic << "\n";
				diagnostic << "structurally_held,Alpha7|q7|A1-A7|D1|D6|D7|q1|Beta1-Beta7\n";
				diagnostic << "measurement_nuisance_parameters,anchor_x|anchor_y|anchor_z|bL\n";
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
			diagnostic << "iteration,geometry_mae_after_bias_mm,max_mm,step_scale,rope_bias_mm";
			for (int joint = 0; joint < m_RobotAxis; ++joint) diagnostic << ",d" << (joint + 1) << "_mm";
			for (int joint = 0; joint < m_RobotAxis; ++joint) diagnostic << ",alpha" << (joint + 1) << "_deg";
			for (int joint = 0; joint < m_RobotAxis; ++joint) diagnostic << ",q0_" << (joint + 1) << "_deg";
			diagnostic << "\n";
			for (int traceIndex = 0; traceIndex < m_R6Trace.count; ++traceIndex)
			{
				const CalibrationV3IterationRecord& rec = m_R6Trace.record[traceIndex];
				diagnostic << rec.iteration << ',' << rec.maeM * 1000.0 << ','
					<< rec.maxAbsM * 1000.0 << ',' << rec.stepScale << ',' << rec.ropeBiasM * 1000.0;
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
			diagnostic << ",sample_std_mm,rope_bias_corrected_mm,residual_before_bias_mm,residual_after_geometry_mm\n";
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
					<< ',' << ropeWithOffsetM * 1000.0 - v3Report.ropeBiasM * 1000.0
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

		// R6.3 production gate: a low fitted residual alone is insufficient.
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
		// R7 was near 2.1e3 with 15 robot + 3 anchor columns. R7.1 adds the
		// independent bL column; the same 82-point field data is about 2.64e3.
		// Use a narrow 3.0e3 ceiling while still requiring full augmented rank.
		const bool conditionGate = std::isfinite(v3Report.jacobianCondition) && v3Report.jacobianCondition <= 3000.0;
		const bool rankGate = v3Report.jacobianRank == v3Report.activeCount;
		const bool motionGate = maxTargetErrorDeg <= 0.1;
		const bool repeatGate = maxSampleStdMm <= 0.5;
		const bool fixtureGate = j7SweepRangeMm <= 1.0;
		const bool point19Gate = point19RepeatRangeMm <= 1.0;
		// R6.3 does not force a structurally zero joint into the solve.  With the
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
			const bool laserEnvelopeGate = m_RobotAxis == 7 &&
				m_R6OutD[1] * 1000.0 >= 47.25 && m_R6OutD[1] * 1000.0 <= 49.50 &&
				m_R6OutD[2] * 1000.0 >= 491.49 && m_R6OutD[2] * 1000.0 <= 493.77 &&
				m_R6OutD[3] * 1000.0 >= -122.44 && m_R6OutD[3] * 1000.0 <= -119.14 &&
				m_R6OutD[4] * 1000.0 >= 315.65 && m_R6OutD[4] * 1000.0 <= 318.49;
			m_R5CalibrationPassed = residualGate && conditionGate && rankGate && motionGate &&
				repeatGate && fixtureGate && point19Gate && observabilityGate &&
				lengthGate && alphaGate && zeroGate && laserEnvelopeGate;

		CString gateLog;
			gateLog.Format(_T("R7.1 gate pass=%d residual=%d condition=%d rank=%d motion=%d repeat=%d fixture=%d point19=%d observability=%d length=%d alpha=%d zero=%d laser_envelope=%d q6_active=%d max_target_error_deg=%.6f bad_point=%d max_sample_std_mm=%.6f j7_range_mm=%.6f point19_range_mm=%.6f q6_sensitivity_mm_per_deg=%.6f q7_sensitivity_mm_per_deg=%.9f q6_alpha6_cosine=%.6f\n"),
				m_R5CalibrationPassed ? 1 : 0, residualGate ? 1 : 0, conditionGate ? 1 : 0,
			rankGate ? 1 : 0, motionGate ? 1 : 0,
			repeatGate ? 1 : 0, fixtureGate ? 1 : 0, point19Gate ? 1 : 0,
				observabilityGate ? 1 : 0, lengthGate ? 1 : 0, alphaGate ? 1 : 0, zeroGate ? 1 : 0,
				laserEnvelopeGate ? 1 : 0,
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
		str.Format(_T("%+.3lf"), v3Report.ropeBiasM * 1000.0);
		m_RopeBias.SetWindowTextW(str);

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
		AppendR6Monitor(FormatR6Trace(m_R6Trace), false);
	}

	m_SaveBtn.EnableWindow(m_R5CalibrationPassed);
	if (!m_R5CalibrationPassed && !m_R63FeedbackValidationRunning)
	{
			MessageBox(_T("R7.1计算已完成，但未通过量产安全门限。\r\n"
			"要求：去零偏后几何MAE<=2mm、最大残差<=10mm、满秩、条件数<=3000、\r\n"
			"单点5次标准差<=0.5mm、\r\n"
			"q6按解析雅可比自动判定：零列时固定；可见时必须有有效敏感度且不能与alpha6重合；\r\n"
			"q7敏感度必须接近0；\r\n"
				"目标/实际关节误差<=0.1度、杆长<=6mm、alpha<=1.5度、零位<=2度，且D2~D5位于激光8机扩展包络。\r\n"
			"结果仅供诊断，已禁止保存和写回机器人。"),
				_T("R7.1标定结果未通过"), MB_OK | MB_ICONWARNING);
	}
	m_CalibBtn.EnableWindow(false);
}

void RopeEncoderCalibrationDlg::OnBnClickedR6Feedback()
{
	if (!m_R6HaveResult || static_cast<int>(m_R6OutD.size()) != m_RobotAxis)
	{
		MessageBox(_T("当前没有可复输入的R7计算结果。"), _T("R7复输入检验"), MB_OK | MB_ICONINFORMATION);
		return;
	}

	// R6.5 performs the second solve directly. R6.3 recursively invoked the
	// complete Calculate button handler, which rewrote files, converted shared
	// units and mutated GUI/global state. Windows recorded heap corruption after
	// that path. This direct calculation is offline and leaves the first save
	// candidate untouched.
	ShowCurrentMdh(_T("复输入检验前（第一次输入）"));
	AppendR6Monitor(_T("复输入检验开始：不运动、不写机器人，仅把第一次候选MDH作为第二次算法输入。"));
	if (m_RbtJoint.rows() != m_RobotAxis || m_RbtJoint.cols() != m_TotalNum)
	{
		MessageBox(_T("内存中的关节数据尺寸不完整，不能复输入检验。"), _T("R7复输入检验"), MB_OK | MB_ICONERROR);
		return;
	}

	const std::vector<double> firstA = m_R6OutA;
	const std::vector<double> firstD = m_R6OutD;
	const std::vector<double> firstAlpha = m_R6OutAlpha;
	const std::vector<double> firstQ0 = m_R6OutQ0;
	const CalibrationV3Trace firstTrace = m_R6Trace;
	const CalibrationV3Report firstReport = m_R6LastReport;
	const bool firstPassed = m_R5CalibrationPassed;

	MatrixXd feedbackDh = m_ThisRbtDH;
	for (int joint = 0; joint < m_RobotAxis; ++joint)
	{
		feedbackDh(0, joint) = firstA[joint];
		feedbackDh(1, joint) = firstD[joint];
		feedbackDh(2, joint) = firstAlpha[joint];
		// q0 and thetaFixed enter FK as a sum.  Folding q0 into row 4 makes
		// the next calculation a genuine model-feedback stability check.
		feedbackDh(3, joint) += firstQ0[joint];
	}
	CString feedbackModel = _T("第二次算法输入MDH（第一次候选结果）：\r\nd(mm): ");
	CString valueText;
	for (int joint = 0; joint < m_RobotAxis; ++joint)
	{
		valueText.Format(_T("%.5f%s"), feedbackDh(1, joint) * 1000.0,
			joint + 1 == m_RobotAxis ? _T("\r\nalpha(deg): ") : _T(", "));
		feedbackModel += valueText;
	}
	for (int joint = 0; joint < m_RobotAxis; ++joint)
	{
		valueText.Format(_T("%.5f%s"), feedbackDh(2, joint) * 180.0 / PI,
			joint + 1 == m_RobotAxis ? _T("\r\ntheta+q0(deg): ") : _T(", "));
		feedbackModel += valueText;
	}
	for (int joint = 0; joint < m_RobotAxis; ++joint)
	{
		valueText.Format(_T("%.5f%s"), feedbackDh(3, joint) * 180.0 / PI,
			joint + 1 == m_RobotAxis ? _T("\r\n") : _T(", "));
		feedbackModel += valueText;
	}
	AppendR6Monitor(feedbackModel);

	VectorXd encoderValue(m_TotalNum);
	for (int point = 0; point < m_TotalNum; ++point)
	{
		CString value = m_List.GetItemText(point, 2);
		if (value.IsEmpty())
		{
			MessageBox(_T("拉线点位数据不完整，不能复输入检验。"), _T("R7复输入检验"), MB_OK | MB_ICONERROR);
			return;
		}
		encoderValue(point) = atof(CT2A(value.GetString())) * 0.001 + 0.043 + 0.0195;
	}
	std::vector<double> secondBefore(m_TotalNum, 0.0), secondAfter(m_TotalNum, 0.0);
	double secondCalibResult[12] = {};
	CalibrationV3Report secondReport = {};
	m_R63FeedbackValidationRunning = true;
	const int secondStatus = RunCalibrationV3ForGui(m_RobotAxis, m_TotalNum,
		feedbackDh, m_RbtJoint, encoderValue, m_ThisRbt.ToolOffset,
		secondCalibResult, secondBefore.data(), secondAfter.data(), &secondReport);
	const CalibrationV3Trace secondTrace = m_R6Trace;
	m_R63FeedbackValidationRunning = false;

	// Restore the first result as the only save/write candidate.
	m_R6OutA = firstA;
	m_R6OutD = firstD;
	m_R6OutAlpha = firstAlpha;
	m_R6OutQ0 = firstQ0;
	m_R6Trace = firstTrace;
	m_R6LastReport = firstReport;
	m_R5CalibrationPassed = firstPassed;
	m_SaveBtn.EnableWindow(firstPassed ? TRUE : FALSE);
	m_R6FeedbackBtn.EnableWindow(TRUE);

	_mkdir("./Result");
	const std::string stamp = m_FinishTime.empty() ? "manual" : m_FinishTime;
	std::ofstream feedback("./Result/V3Diagnostic_" + stamp + "_feedback2.csv", std::ios::out | std::ios::trunc);
	if (feedback)
	{
		feedback << std::setprecision(15)
			<< "build," << CalibrationV3BuildId() << '\n'
			<< "status," << secondStatus << '\n'
			<< "rope_bias_mm," << secondReport.ropeBiasM * 1000.0 << '\n'
			<< "mae_before_bias_mm," << secondReport.maeBeforeM * 1000.0 << '\n'
			<< "geometry_mae_after_bias_mm," << secondReport.maeAfterM * 1000.0 << '\n'
			<< "max_after_mm," << secondReport.maxAbsAfterM * 1000.0 << '\n'
			<< "iteration,geometry_mae_after_bias_mm,max_mm,step_scale,rope_bias_mm\n";
		for (int i = 0; i < secondTrace.count; ++i)
			feedback << secondTrace.record[i].iteration << ',' << secondTrace.record[i].maeM * 1000.0
				<< ',' << secondTrace.record[i].maxAbsM * 1000.0 << ',' << secondTrace.record[i].stepScale
				<< ',' << secondTrace.record[i].ropeBiasM * 1000.0 << '\n';
	}

	CString summary;
	summary.Format(_T("同一批数据复输入检验完成（status=%d，不运动、不写机器人、不替换第一次结果）。\r\n\r\n"
		"第一次：bL=%+.3f mm，去零偏前MAE %.3f mm，几何MAE %.3f mm，最大残差 %.3f mm\r\n"
		"第二次：bL=%+.3f mm，去零偏前MAE %.3f mm，几何MAE %.3f mm，最大残差 %.3f mm\r\n\r\n"
		"第二次仅检查收敛趋势，不是独立精度验证；详细结果见 *_feedback2.csv。"),
		secondStatus, firstReport.ropeBiasM * 1000.0, firstReport.maeBeforeM * 1000.0, firstReport.maeAfterM * 1000.0,
		firstReport.maxAbsAfterM * 1000.0,
		secondReport.ropeBiasM * 1000.0, secondReport.maeBeforeM * 1000.0, secondReport.maeAfterM * 1000.0,
		secondReport.maxAbsAfterM * 1000.0);
	AppendR6Monitor(summary);
	MessageBox(summary, _T("R7复输入检验"), MB_OK | MB_ICONINFORMATION);
}

void RopeEncoderCalibrationDlg::OnBnClickedR6Restore()
{
	if (!m_R6HaveOriginalDH)
	{
		MessageBox(_T("没有保存的原始MDH。"), _T("R6.3恢复"), MB_OK | MB_ICONINFORMATION);
		return;
	}
	m_ThisRbtDH = m_R6OriginalDH;
	m_R6HaveResult = false;
	m_R6FeedbackBtn.EnableWindow(FALSE);
	m_SaveBtn.EnableWindow(FALSE);
	ShowCurrentMdh(_T("已恢复初始MDH"));
	MessageBox(_T("已恢复本次计算前的原始MDH。"), _T("R6.3恢复"), MB_OK | MB_ICONINFORMATION);
}

//保存按钮,用于保存末端距离拉线编码器的距离
void RopeEncoderCalibrationDlg::OnBnClickedButton8()
{
	ShowR65ModelContract(_T("保存候选前"));
	ShowCurrentMdh(_T("保存操作前的算法原始输入"));
	CString saveInfo;
	saveInfo.Format(_T("准备R7.1完整写回：去零偏后几何MAE %.3f mm，MAX %.3f mm，bL=%+.3f mm（仅测量链，不写MDH），安全门限=%s。"),
		m_R6LastReport.maeAfterM * 1000.0, m_R6LastReport.maxAbsAfterM * 1000.0,
		m_R6LastReport.ropeBiasM * 1000.0, m_R5CalibrationPassed ? _T("通过") : _T("未通过"));
	AppendR6Monitor(saveInfo);
	if (!m_R5CalibrationPassed || !m_R6HaveResult || m_RobotAxis != 7)
	{
		AppendR6Monitor(_T("写回被阻止：结果未通过安全门限、没有有效候选或不是HE3七轴。"));
		MessageBox(_T("当前结果不满足R7.1写回条件，未修改机器人。"),
			_T("R7.1写回保护"), MB_OK | MB_ICONERROR);
		return;
	}
	if (!ReloadControllerModelAsInput(_T("写回前备份快照"), false))
	{
		MessageBox(_T("无法完整读取控制器D/Alpha/零位，未执行写回。"),
			_T("R7.1写回保护"), MB_OK | MB_ICONERROR);
		return;
	}

	const std::vector<double> beforeD = m_R65ControllerDmm;
	const std::vector<double> beforeAlpha = m_R65ControllerLegacyAlphaDeg;
	const std::vector<double> beforeZero = m_R65ControllerZeroCounts;
	std::vector<double> candidateD = beforeD;
	std::vector<double> candidateAlpha = beforeAlpha;
	std::vector<double> candidateZero = beforeZero;

	// Laser scheme: D2..D5, physical Alpha1..Alpha6 and q2..q6.  A, D1/D6/D7,
	// q1/q7 and Alpha7 stay exactly at the controller snapshot.  Craig alpha
	// slot 2 maps back to controller legacy Alpha1, etc.
	for (int joint = 1; joint <= 4; ++joint)
		candidateD[static_cast<size_t>(joint)] = m_R6OutD[static_cast<size_t>(joint)] * 1000.0;
	for (int physicalAlpha = 0; physicalAlpha < 6; ++physicalAlpha)
		candidateAlpha[static_cast<size_t>(physicalAlpha)] =
			m_R6OutAlpha[static_cast<size_t>(physicalAlpha + 1)] * 180.0 / PI;
	for (int joint = 1; joint <= 5; ++joint)
	{
		if (joint == 5 && !m_R61Q6CandidateActive) continue;
		const double correctionCounts = m_ThisRbt.MoveDirection[joint] *
			(-m_R6OutQ0[static_cast<size_t>(joint)] * 180.0 / PI) *
			m_ThisRbt.EncoderLineNum[joint] / 360.0;
		const double rounded = correctionCounts >= 0.0 ?
			std::floor(correctionCounts + 0.5) : std::ceil(correctionCounts - 0.5);
		candidateZero[static_cast<size_t>(joint)] = beforeZero[static_cast<size_t>(joint)] + rounded;
	}

	CString candidateLog;
	candidateLog.Format(_T("R7.1候选（写入前；bL不在候选中）\r\nD(mm): %s\r\n旧式Alpha(deg): %s\r\nZero(cnt): %s\r\n"),
		FormatR65Vector(candidateD, 5).GetString(), FormatR65Vector(candidateAlpha, 5).GetString(),
		FormatR65Vector(candidateZero, 0).GetString());
	AppendR6Monitor(candidateLog);
	CString candidateFailure;
	if (!ValidateR65Candidate(candidateD, candidateAlpha, candidateZero, candidateFailure))
	{
		AppendR6Monitor(_T("候选合理性检查失败：") + candidateFailure);
		MessageBox(candidateFailure, _T("R7.1写回保护"), MB_OK | MB_ICONERROR);
		return;
	}
	if (PowerOff() != 0)
	{
		AppendR6Monitor(_T("机器人下电命令未确认成功，禁止写参数。"));
		MessageBox(_T("机器人下电未确认，未执行写回。"), _T("R7.1写回保护"), MB_OK | MB_ICONERROR);
		return;
	}

	CString failure;
	bool writeOk = true;
	for (int joint = 1; joint <= 4 && writeOk; ++joint)
		writeOk = WriteControllerValueVerified(m_IDND, joint + 1,
			candidateD[static_cast<size_t>(joint)], 0.0005, failure);
	for (int physicalAlpha = 0; physicalAlpha < 6 && writeOk; ++physicalAlpha)
		writeOk = WriteControllerValueVerified(m_IDNAlpha, physicalAlpha + 1,
			candidateAlpha[static_cast<size_t>(physicalAlpha)], 0.00001, failure);
	for (int joint = 1; joint <= 5 && writeOk; ++joint)
	{
		if (joint == 5 && !m_R61Q6CandidateActive) continue;
		writeOk = WriteControllerValueVerified(m_IDNZeroEncoderValue, joint + 1,
			candidateZero[static_cast<size_t>(joint)], 0.01, failure);
	}

	if (!writeOk)
	{
		AppendR6Monitor(_T("写回事务失败：") + failure + _T(" 正在恢复写入前快照；不会执行21.23 save。"));
		CString ignored;
		for (int joint = 1; joint <= 4; ++joint)
			WriteControllerValueVerified(m_IDND, joint + 1, beforeD[static_cast<size_t>(joint)], 0.0005, ignored);
		for (int physicalAlpha = 0; physicalAlpha < 6; ++physicalAlpha)
			WriteControllerValueVerified(m_IDNAlpha, physicalAlpha + 1,
				beforeAlpha[static_cast<size_t>(physicalAlpha)], 0.00001, ignored);
		for (int joint = 1; joint <= 5; ++joint)
			WriteControllerValueVerified(m_IDNZeroEncoderValue, joint + 1,
				beforeZero[static_cast<size_t>(joint)], 0.01, ignored);
		MessageBox(_T("写回或回读校验失败，已尝试恢复写入前快照，未持久化。请查看右侧日志。"),
			_T("R7.1写回失败"), MB_OK | MB_ICONERROR);
		return;
	}

	if (WriteIDNValue(m_IDNSave, m_IndexStr, m_IDNSaveOption) != 0)
	{
		AppendR6Monitor(_T("所有逐项回读通过，但控制器21.23 save失败。参数可能只在运行内存中；停止后续操作并人工检查。"));
		MessageBox(_T("参数逐项写入成功，但持久化保存失败。请勿继续测试，先检查控制器。"),
			_T("R7.1保存失败"), MB_OK | MB_ICONERROR);
		return;
	}
	AppendR6Monitor(_T("21.23 save返回成功；现在重新读取D/Alpha/零位作为闭环证据；公共MDH算法输入保持不变。"));
	if (!ReloadControllerModelAsInput(_T("持久化保存后回读"), false))
	{
		MessageBox(_T("保存命令成功，但保存后完整回读失败。不要开始下一轮，请查看右侧日志。"),
			_T("R7.1回读失败"), MB_OK | MB_ICONERROR);
		return;
	}

	_mkdir("./Result");
	const std::string stamp = m_FinishTime.empty() ? "manual" : m_FinishTime;
	const std::string path = "./Result/R71Writeback_" + stamp + ".csv";
	std::ofstream out(path, std::ios::out | std::ios::trunc);
	if (out)
	{
		out << std::setprecision(15);
		out << "build," << CalibrationV3BuildId() << "\n";
		out << "calibration_input_source," << static_cast<const char*>(CT2A(m_R65InputSource)) << "\n";
		out << "writeback_baseline_source,controller_readback\n";
		out << "rope_bias_mm," << (m_R6LastReport.ropeBiasM * 1000.0) << "\n";
		out << "rope_bias_written_to_mdh,0\n";
		out << "parameter,index,before,candidate,readback\n";
		for (int i = 0; i < 7; ++i) out << "D," << (i + 1) << ',' << beforeD[i] << ',' << candidateD[i] << ',' << m_R65ControllerDmm[i] << "\n";
		for (int i = 0; i < 7; ++i) out << "AlphaLegacy," << (i + 1) << ',' << beforeAlpha[i] << ',' << candidateAlpha[i] << ',' << m_R65ControllerLegacyAlphaDeg[i] << "\n";
		for (int i = 0; i < 7; ++i) out << "ZeroCount," << (i + 1) << ',' << beforeZero[i] << ',' << candidateZero[i] << ',' << m_R65ControllerZeroCounts[i] << "\n";
	}
	AppendR6Monitor(_T("R7.1闭环完成：保存后的控制器回读仅作为闭环证据；下一次点‘开始’会再次强制恢复公共HE3_GY-cfg.txt作为算法输入。写回记录：") + CString(path.c_str()));
	m_SaveBtn.EnableWindow(FALSE);
	MessageBox(_T("保存和逐项回读成功；bL未写入MDH。下一轮开始时仍会使用公共HE3_GY-cfg.txt，并且必须重新采集，不能用旧拉线数据宣称复验精度。"),
		_T("R7.1闭环完成"), MB_OK | MB_ICONINFORMATION);
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
	// Preserve the exact file-loaded model before any controller snapshot can
	// replace it. R7.1 always restores this immutable public-config copy before motion.
	m_R7OriginalConfigDH = m_ThisRbtDH;
	m_R7OriginalConfigValid = true;
	m_R7OriginalConfigSource.Format(_T("公共MDH（%S/%S%S）"),
		_rbtConfigPath.c_str(), rbtType.c_str(), _configExtension.c_str());
	m_R65InputSource = m_R7OriginalConfigSource;

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
	ShowCurrentMdh(_T("机器人配置加载完成"));
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

	FILE *fp = NULL;

	string path = "./log/" + time + "_log.txt";

	if (fopen_s(&fp, path.c_str(), "at") != 0 || fp == NULL)
	{
		OutputDebugString(_T("WriteLog: cannot open log file.\n"));
		return;
	}
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
	CString pointInfo;
	const double stdMm = (wParam < m_R5EncoderStdCounts.size())
		? m_R5EncoderStdCounts[static_cast<size_t>(wParam)] / 284.94 : 0.0;
	const double meanCounts = (wParam < m_R5EncoderMeanCounts.size())
		? m_R5EncoderMeanCounts[static_cast<size_t>(wParam)] : static_cast<double>(lParam);
	CString actual = (wParam < static_cast<WPARAM>(m_TotalNum)) ? m_RobotHere[wParam] : _T("");
	CString target = (wParam < static_cast<WPARAM>(m_TotalNum)) ? m_List.GetItemText(static_cast<int>(wParam), 1) : _T("");
	pointInfo.Format(_T("点位 %d/%d：输入来源=%s；目标关节=%s；实际关节=%s；"
		"拉线mean=%.3f cnt，home=%d cnt，距离=%.6f mm，样本=%d，std=%.6f mm"),
		static_cast<int>(wParam) + 1, m_TotalNum,
		m_R65InputSource.GetString(), target.GetString(), actual.GetString(),
		meanCounts, m_EncoderHomeValue, EncoderValue, m_R5SamplesPerPose, stdMm);
	AppendR6Monitor(pointInfo);

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
	CString communication;
	communication.Format(_T("通信[%ld] %s"), static_cast<long>(wParam), asd.GetString());
	AppendR6Monitor(communication, false);
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
