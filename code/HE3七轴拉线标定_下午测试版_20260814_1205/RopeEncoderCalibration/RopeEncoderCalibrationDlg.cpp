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
#include "MathematicalDLL.h"
#include "FileOperation.h"
#include "websocket_endpoint.h"
#include <WinSock2.h>
#include "QKMLinkComm.h"
#include "RoughCalibrationDlg.h"
#include <string>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <mutex>

using namespace std;
//websocket 的应用
using namespace kagula;
#define PI 3.1415926
// Controlled HE3 machine-test build. Communication and the existing low-speed
// 60-pose acquisition workflow are enabled, but seven-axis parameter write-back
// remains locked until its controller IDN mapping and unit conversion are
// defined and an accepted CalibrationV2 result is available.
static const bool kOfflinePreviewMode = false;
static const bool kMachineTestMode = true;
static const bool kEnableV2WriteBack = false;
// This is a rope-model residual gate, not a claim about 3-D absolute accuracy.
// A run above the limit is kept as a diagnostic report and is never accepted
// as a successful HE3 calibration.
static const double kAcceptedRopeMaeMeters = 0.001;
static bool gQKMLinkReady = false;

static std::mutex gMachineLogMutex;
static std::string gMachineLogPath;

static std::string MachineTimestamp(bool fileSafe = false)
{
	SYSTEMTIME st;
	GetLocalTime(&st);
	char buffer[64] = {};
	if (fileSafe)
		sprintf_s(buffer, "%04u%02u%02u_%02u%02u%02u_%03u", st.wYear, st.wMonth, st.wDay,
			st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
	else
		sprintf_s(buffer, "%04u-%02u-%02u %02u:%02u:%02u.%03u", st.wYear, st.wMonth, st.wDay,
			st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
	return buffer;
}

static void MachineTrace(const char* stage, const std::string& message)
{
	std::lock_guard<std::mutex> lock(gMachineLogMutex);
	if (gMachineLogPath.empty())
	{
		_mkdir("./log");
		gMachineLogPath = "./log/MachineTest_" + MachineTimestamp(true) + ".log";
	}
	std::ostringstream line;
	line << MachineTimestamp(false) << " [T" << GetCurrentThreadId() << "] [" << stage << "] " << message << "\n";
	const std::string text = line.str();
	OutputDebugStringA(text.c_str());
	std::ofstream output(gMachineLogPath.c_str(), std::ios::out | std::ios::app);
	if (output.is_open())
		output << text;
}

static void MachineTrace(const char* stage, const CString& message)
{
	CStringA narrow(message);
	MachineTrace(stage, std::string(narrow.GetString()));
}

#pragma region 全局变量
//软件版本
CString m_Version = _T(" V4.0.1");
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
// V2 results are diagnostic-only until laser/production acceptance is complete.
bool m_V2WriteBackAllowed = false;
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
CString *m_RobotHere = nullptr;
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
	delete[] m_RobotHere;
	m_RobotHere = nullptr;
	m_ThisRbt.ResetArrays();
	if (m_MoveEvent != NULL && m_MoveEvent != INVALID_HANDLE_VALUE)
		CloseHandle(m_MoveEvent);
	if (m_WhereEvent != NULL && m_WhereEvent != INVALID_HANDLE_VALUE)
		CloseHandle(m_WhereEvent);
	if (m_IDNEvent != NULL && m_IDNEvent != INVALID_HANDLE_VALUE)
		CloseHandle(m_IDNEvent);
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
	ON_MESSAGE(WM_GET_ENCODER, &RopeEncoderCalibrationDlg::OnEncoderValueShow)
	ON_MESSAGE(WM_MOVE_ZERO, &RopeEncoderCalibrationDlg::OnMoveZero)
	ON_CBN_SELCHANGE(IDC_COMBO2, &RopeEncoderCalibrationDlg::OnSelchangeCombo2)
	ON_EN_CHANGE(IDC_EDIT1, &RopeEncoderCalibrationDlg::OnEnChangeEdit1)
END_MESSAGE_MAP()

LRESULT RopeEncoderCalibrationDlg::OnEncoderValueShow(WPARAM wParam, LPARAM lParam)
{
	if (m_ActiveRoughCalibrationDlg == nullptr
		|| !::IsWindow(m_ActiveRoughCalibrationDlg->GetSafeHwnd()))
		return 0;
	return m_ActiveRoughCalibrationDlg->OnEncoderValueShow(wParam, lParam);
}

//初始化
BOOL RopeEncoderCalibrationDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	wchar_t workingDirectory[MAX_PATH] = {};
	GetCurrentDirectory(MAX_PATH, workingDirectory);
	MachineTrace("SESSION", std::string("application start; build=") + CalibrationBuildId()
		+ "; mode=" + (kMachineTestMode ? "CONTROLLED_MACHINE_TEST" : "OFFLINE_PREVIEW")
		+ "; v2WriteBack=" + (kEnableV2WriteBack ? "enabled" : "disabled")
		+ "; workingDirectory=" + CStringA(workingDirectory).GetString());

	CString windowTitle;

	CString tempCstring;
	AfxGetMainWnd()->GetWindowText(windowTitle);
	windowTitle += m_Version;
	AfxGetMainWnd()->SetWindowText(windowTitle);

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

	// Do not depend on filesystem enumeration order.  The controlled test build
	// must open on the HE3 seven-axis pull-wire workflow explicitly.
	const int he3Index = m_RbtTypeCombo.FindStringExact(-1, _T("HE3_GY"));
	m_RbtTypeCombo.SetCurSel(he3Index >= 0 ? he3Index : 0);
	m_CalibTypeCombo.SetCurSel(0);
	
	OnCbnSelchangeCombo1();

	if (m_List.GetItemCount() == 0) {
		GetDlgItem(IDC_BUTTON1)->EnableWindow(false);
	}

	m_TotalNum = m_ThisRbt.CalLocationNumber;
	m_RobotAxis = m_ThisRbt.RobotAxis;
	m_RobotType = m_ThisRbt.RobotType;

	//通讯初始化
	if (!kOfflinePreviewMode)
	{
		gQKMLinkReady = QKMLinkInit();
		MachineTrace("COMM_INIT", gQKMLinkReady ? "QKMLink initialized" :
			"QKMLink initialization FAILED; HRESULT=" + std::to_string(QKMLinkLastInitError()));
		if (!gQKMLinkReady)
		{
			MessageBox(QKMLinkLastInitErrorText(), _T("QKMLink通信错误"), MB_OK | MB_ICONERROR);
			GetDlgItem(IDC_BUTTON1)->EnableWindow(FALSE);
			m_StartBtn.EnableWindow(FALSE);
		}
		try
		{
			endpoint.init();
			MachineTrace("COMM_INIT", "websocket endpoint initialized");
		}
		catch (const std::exception& ex)
		{
			MachineTrace("COMM_INIT", std::string("websocket initialization exception: ") + ex.what());
			MessageBox(_T("WebSocket初始化失败，禁止进行实机测试。"), _T("通信错误"), MB_OK | MB_ICONERROR);
			return FALSE;
		}
	}

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

	CString buildMessage;
	buildMessage.Format(_T("[GUI] %s. Loaded %S; log=%S; V2 write-back=%s\n"),
		kMachineTestMode ? _T("CONTROLLED MACHINE TEST MODE") : _T("Offline preview mode"),
		CalibrationBuildId(), gMachineLogPath.c_str(), kEnableV2WriteBack ? _T("ENABLED") : _T("LOCKED"));
	OutputDebugString(buildMessage);
	if (kOfflinePreviewMode)
	{
		m_ConnectBtn.EnableWindow(false);
		m_StartBtn.EnableWindow(false);
		m_ContinueBtn.EnableWindow(false);
		m_PauseBtn.EnableWindow(false);
		m_StopBtn.EnableWindow(false);
		m_LocationZero.EnableWindow(false);
		m_EncoderBtn.EnableWindow(false);
		m_SaveBtn.EnableWindow(false);
	}

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
	int oldEncoderValue = 0;
	int newEncoderValue = 0;
	string RecieveStr;

	if (m_Client.State() <= 0) {
		m_Client.Connect("ws://" + m_CellIPStr);
	}

	m_Client.Send(CmdStr);
	Sleep(500);
	m_Client.Send(CmdStr);

	//通过间隔一秒获取两次值,如果两次值一致,说明机器人已经停稳,没有抖动
	RecieveStr = m_Client.Receive();
	oldEncoderValue = DecodeEncoder(RecieveStr);
	Sleep(1000);
	m_Client.Send(CmdStr);
	RecieveStr = m_Client.Receive();
	newEncoderValue = DecodeEncoder(RecieveStr);

	if (newEncoderValue == oldEncoderValue) {
		m_Client.Clear();
		return newEncoderValue;
	}
	else
	{
		m_ErrorTime++;
		m_Client.Clear();
		if (m_ErrorTime > 20) {
			m_ErrorTime = 0;
			return -1;
		}
		return GetEncoderValue();
	}
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
	if (!gQKMLinkReady)
	{
		MachineTrace("CONNECT_BLOCKED", "QKMLink COM object is not initialized; HRESULT="
			+ std::to_string(QKMLinkLastInitError()));
		MessageBox(QKMLinkLastInitErrorText(), _T("禁止连接"), MB_OK | MB_ICONERROR);
		return;
	}
	if (kOfflinePreviewMode)
	{
		MessageBox(_T("当前版本为离线预览模式，禁止连接机器人。"), _T("安全保护"), MB_OK | MB_ICONWARNING);
		return;
	}
	// TODO: Add your control notification handler code here
	CString strIP;
	BYTE IP0, IP1, IP2, IP3;
	m_RbtIPAdr.GetAddress(IP0, IP1, IP2, IP3);
	strIP.Format(_T("%d.%d.%d.%d"), IP0, IP1, IP2, IP3);
	m_RbtIPStr = strIP;

	m_CellIPAdr.GetAddress(IP0, IP1, IP2, IP3);
	strIP.Format(_T("%d.%d.%d.%d"), IP0, IP1, IP2, IP3);
	m_CellIPStr = CT2A(strIP.GetBuffer(0));
	CString selectedRobot;
	m_RbtTypeCombo.GetLBText(m_RbtTypeCombo.GetCurSel(), selectedRobot);
	if (selectedRobot.CompareNoCase(_T("HE3_GY")) != 0 || m_RobotType != Cobot || m_RobotAxis != 7 || m_TotalNum != 60)
	{
		MachineTrace("CONNECT_BLOCKED", "controlled machine test requires HE3_GY / Cobot(type 3) / 7 axes / 60 poses");
		MessageBox(_T("当前受控实机版本只允许选择 HE3_GY（7轴、60点）。"), _T("配置不匹配"), MB_OK | MB_ICONERROR);
		return;
	}
	CString confirmation;
	confirmation.Format(_T("即将连接实机，但此步骤不会上电或运动。\n\n机器人配置：%s\n轴数：%d\n点位数：%d\n机器人IP：%s\nCell IP：%S\n\n确认网络和机器人型号正确吗？"),
		selectedRobot.GetString(), m_RobotAxis, m_TotalNum, m_RbtIPStr.GetString(), m_CellIPStr.c_str());
	if (MessageBox(confirmation, _T("受控实机测试：连接确认"), MB_YESNO | MB_ICONWARNING | MB_DEFBUTTON2) != IDYES)
	{
		MachineTrace("CONNECT", "cancelled by operator before communication");
		return;
	}
	MachineTrace("CONNECT", std::string("requested robot=") + CStringA(selectedRobot).GetString()
		+ "; robotIP=" + CStringA(m_RbtIPStr).GetString() + "; cellIP=" + m_CellIPStr
		+ "; axis=" + std::to_string(m_RobotAxis) + "; samples=" + std::to_string(m_TotalNum));

	// TODO: Add your control notification handler code here
	//websocket 只要地址是对的,就会认为连接成功;所以需要发送指令来确认是否真正的连接成功,但是发送不能太快,不然会造成状态错乱
	int stt = endpoint.Connect("ws://" + m_CellIPStr);
	MachineTrace("CONNECT", "websocket Connect returned " + std::to_string(stt));
	if (stt < 0)
	{
		MessageBox(_T("连接拉线编码器Cell失败，禁止继续。"), _T("连接失败"), MB_OK | MB_ICONERROR);
		return;
	}

	m_IsConnect = true;
	m_Client = endpoint;

	m_IsConnect = QKMLinkConnect(m_RbtIPStr, this);
	MachineTrace("CONNECT", std::string("QKMLinkConnect returned ") + (m_IsConnect ? "true" : "false"));

	if (!m_IsConnect)
	{
		m_Client.Close();
		MachineTrace("CONNECT", "robot connection failed; websocket closed");
		MessageBox(_T("连接机器人失败!"));
		return;
	}

	m_ConnectBtn.EnableWindow(!m_IsConnect);
	m_DisconnectBtn.EnableWindow(m_IsConnect);
	m_CalibTypeCombo.SelectString(0, _T("拉线编码器"));
	m_CalibTypeCombo.EnableWindow(false);
	m_RbtTypeCombo.EnableWindow(m_IsConnect);
	m_EncoderBtn.EnableWindow(m_IsConnect);
	m_LocationZero.EnableWindow(m_IsConnect);
	m_CalibTypeBtn.EnableWindow(m_IsConnect);
	m_RbtSpeed.EnableWindow(m_IsConnect);

	QKMLinkEventReset();
	SendInit();
	MachineTrace("CONNECT", "communication setup commands sent; robot remains unpowered until Start confirmation");
	QKMLinkEventReset();
	m_KeepConnect = true;
	KeepConnectThread = AfxBeginThread(RopeEncoderCalibrationDlg::KeepConnecting, (LPVOID)this);
	
	m_EncoderHomeStr.EnableWindow(true);
	m_RbtTypeCombo.EnableWindow(false);
}

//开始按钮,开始进行标定动作
void RopeEncoderCalibrationDlg::OnBnClickedButton2()
{
	if (!gQKMLinkReady)
	{
		MachineTrace("START_BLOCKED", "QKMLink COM object is not initialized");
		MessageBox(QKMLinkLastInitErrorText(), _T("禁止上电"), MB_OK | MB_ICONERROR);
		return;
	}
	if (kOfflinePreviewMode)
	{
		MessageBox(_T("当前版本为离线预览模式，禁止上电和自动运动。"), _T("安全保护"), MB_OK | MB_ICONWARNING);
		return;
	}
	if (!m_IsConnect)
	{
		MessageBox(_T("尚未连接机器人，禁止开始。"), _T("安全保护"), MB_OK | MB_ICONERROR);
		return;
	}
	CString selectedRobot;
	m_RbtTypeCombo.GetLBText(m_RbtTypeCombo.GetCurSel(), selectedRobot);
	if (selectedRobot.CompareNoCase(_T("HE3_GY")) != 0 || m_RobotType != Cobot || m_RobotAxis != 7 || m_TotalNum != 60)
	{
		MachineTrace("START_BLOCKED", "expected HE3 7-axis/60-pose configuration");
		MessageBox(_T("当前受控实机版本只允许 HE3 七轴、60点配置。"), _T("配置不匹配"), MB_OK | MB_ICONERROR);
		return;
	}
	CString encoderHome;
	m_EncoderHomeStr.GetWindowTextW(encoderHome);
	if (encoderHome.IsEmpty() || encoderHome.SpanIncluding(_T("-0123456789")) != encoderHome)
	{
		MachineTrace("START_BLOCKED", "encoder home value is missing or invalid");
		MessageBox(_T("请先读取并确认拉线编码器零点值。"), _T("数据不完整"), MB_OK | MB_ICONERROR);
		return;
	}
	CString speedText;
	m_RbtSpeed.GetWindowTextW(speedText);
	CString startConfirmation;
	startConfirmation.Format(_T("下一步将执行：\n1. 机器人上电\n2. Robot.Home 1 回零\n3. 人工确认回零完成\n4. 以速度 %s 自动运动60点并采集\n\n请确认：急停有效、工作区无人、拉线无缠绕、工具安装牢固、所有点位已做碰撞检查。"), speedText.GetString());
	if (MessageBox(startConfirmation, _T("受控实机测试：上电与运动确认"), MB_YESNO | MB_ICONSTOP | MB_DEFBUTTON2) != IDYES)
	{
		MachineTrace("START", "cancelled by operator before power enable");
		return;
	}
	MachineTrace("START", std::string("operator authorized power/home/60 poses; speed=") + CStringA(speedText).GetString()
		+ "; encoderHome=" + CStringA(encoderHome).GetString());
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
	if (SendCmd(sSend) < 0)
	{
		MachineTrace("POWER", "power enable command failed");
		MessageBox(_T("上电指令发送失败。"), _T("实机测试终止"), MB_OK | MB_ICONERROR);
		return;
	}
	MachineTrace("POWER", "Robot.PowerEnable command sent");

	sSend = (CString)m_CMDHome.c_str();
	if (SendCmd(sSend) < 0)
	{
		MachineTrace("HOME", "Robot.Home command failed");
		PowerOff();
		MessageBox(_T("回零指令发送失败，已尝试下电。"), _T("实机测试终止"), MB_OK | MB_ICONERROR);
		return;
	}
	MachineTrace("HOME", "Robot.Home command sent; waiting for operator verification");
	if (MessageBox(_T("请观察机器人状态。仅在机器人已经完成回零、无报警、拉线状态正常时点击“是”开始60点；点击“否”将终止并下电。"),
		_T("确认回零完成"), MB_YESNO | MB_ICONWARNING | MB_DEFBUTTON2) != IDYES)
	{
		MachineTrace("HOME", "operator did not confirm successful homing; aborting and powering off");
		PowerOff();
		return;
	}
	MachineTrace("HOME", "operator confirmed homing completed");

	m_RbtSpeed.GetWindowTextW(sSend);
	sSend = (CString)(m_CMDRbtSpeed).c_str() + sSend;
	SendCmd(sSend);

	m_LocIdx = 0;
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
	if (kOfflinePreviewMode)
	{
		MessageBox(_T("离线预览模式禁止继续机器人运动。"), _T("安全保护"), MB_OK | MB_ICONWARNING);
		return;
	}
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
	if (kOfflinePreviewMode)
	{
		MessageBox(_T("离线预览模式没有可暂停的机器人运动。"), _T("安全保护"), MB_OK | MB_ICONWARNING);
		return;
	}
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
	if (kOfflinePreviewMode)
	{
		MessageBox(_T("离线预览模式没有可停止的机器人运动。"), _T("安全保护"), MB_OK | MB_ICONWARNING);
		return;
	}
	// TODO: 在此添加控件通知处理程序代码
	m_ThreadRun = false;
	MachineTrace("STOP", "operator requested stop; motion loop flag cleared");
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
	if (RunThread != NULL && RunThread->m_hThread != NULL)
		ResumeThread(RunThread->m_hThread);
	if (KeepConnectThread != NULL && KeepConnectThread->m_hThread != NULL)
		ResumeThread(KeepConnectThread->m_hThread);
	MachineTrace("STOP", "abort command sent; unsafe TerminateThread call intentionally removed");
	m_ContinueBtn.EnableWindow(!m_IsConnect);
	m_PauseBtn.EnableWindow(!m_IsConnect);
	m_DisconnectBtn.EnableWindow(m_IsConnect);
	m_StopBtn.EnableWindow(!m_IsConnect);
	m_LocationZero.EnableWindow(m_IsConnect);
}

//获取编码器的值,用于获取零点值
void RopeEncoderCalibrationDlg::OnBnClickedButton6()
{
	if (kOfflinePreviewMode)
	{
		MessageBox(_T("离线预览模式禁止连接或读取拉线编码器。"), _T("安全保护"), MB_OK | MB_ICONWARNING);
		return;
	}
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

//计算按钮
void RopeEncoderCalibrationDlg::OnBnClickedButton7()
{
	CString selectitem;
	double* oldErrorData = NULL;
	double* nowErrorData = NULL;
	double** oldError = &oldErrorData;
	double** nowError = &nowErrorData;

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

	CString JointOffsetCstr, str, show;
	VectorXd EncoderValue(m_TotalNum);
	CString EncoderValueStr;
	for (int ii = 0; ii < m_TotalNum; ii++) {
		EncoderValueStr = m_List.GetItemText(ii, 2);
		EncoderValue[ii] = atof(CT2A(EncoderValueStr.GetBuffer(0)));
	}

	//单位转换为 m
	EncoderValue *= 0.001;
	// CalibrationV2 receives the raw metre value; the named fixture offset is
	// supplied through CalibrationV2Options instead of being hidden in the data.

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

			// 6. Legacy API output pointers (CalibrationV2 uses caller-owned vectors).
			output.Format(_T("oldError 输出指针地址 = %p\n"), oldError);
			OutputDebugString(output);
			output.Format(_T("nowError 输出指针地址 = %p\n"), nowError);
			OutputDebugString(output);
			// 注意：oldError[i] 和 nowError[i] 在入参时未初始化，打印无意义，故省略

			OutputDebugString(_T("=== Calibration 入参结束 ===\n"));
		}

		if (m_RobotAxis == 6 || m_RobotAxis == 7)
		{
			const int parameterCount = m_RobotAxis * 4;
			vector<double> mdh(m_RobotAxis * 5, 0.0);
			vector<double> joints(m_RobotAxis * m_TotalNum, 0.0);
			vector<double> lengths(m_TotalNum, 0.0);
			vector<int> requested(parameterCount, 0);
			vector<double> updatedMdh(m_RobotAxis * 5, 0.0);
			vector<double> parameterDelta(parameterCount, 0.0);
			vector<int> parameterState(parameterCount, CALIBRATION_PARAMETER_FROZEN);
			vector<double> zeroOffsets(m_RobotAxis, 0.0);
			vector<double> beforeResiduals(m_TotalNum, 0.0);
			vector<double> afterResiduals(m_TotalNum, 0.0);

			for (int row = 0; row < 5; ++row)
				for (int axis = 0; axis < m_RobotAxis; ++axis)
					mdh[row * m_RobotAxis + axis] = m_ThisRbtDH(row, axis);
			// m_ThisRbtDH contains ToolOffset folded into the final d value.
			// V2 models it as an explicit flange-to-rope transform.
			mdh[m_RobotAxis + m_RobotAxis - 1] -= m_ThisRbt.ToolOffset;
			for (int axis = 0; axis < m_RobotAxis; ++axis)
				for (int sample = 0; sample < m_TotalNum; ++sample)
					joints[axis * m_TotalNum + sample] = m_RbtJoint(axis, sample);
			for (int sample = 0; sample < m_TotalNum; ++sample)
				lengths[sample] = EncoderValue(sample);

			if (m_RobotAxis == 7)
			{
				// HE3_GY v1: d2,d3,d4,d5,d7 and q2,q3,q4,q5.
				// d6 is frozen because d6/d7 have identical rope sensitivity;
				// q1/q6/q7 are frozen because a single unknown anchor cannot
				// independently observe them with the current axial tool point.
				const int dAxes[] = { 1, 2, 3, 4, 6 };
				const int qAxes[] = { 1, 2, 3, 4 };
				for (int i = 0; i < _countof(dAxes); ++i)
					requested[m_RobotAxis + dAxes[i]] = 1;
				for (int i = 0; i < _countof(qAxes); ++i)
					requested[m_RobotAxis * 3 + qAxes[i]] = 1;
			}
			else
			{
				// Preserve the legacy six-axis parameter intent while V2 performs
				// observability filtering and keeps write-back disabled.
				requested[2] = requested[3] = 1;
				const int dAxes[] = { 0, 3, 4, 5 };
				for (int i = 0; i < _countof(dAxes); ++i)
					requested[m_RobotAxis + dAxes[i]] = 1;
				for (int axis = 0; axis < m_RobotAxis; ++axis)
					requested[m_RobotAxis * 3 + axis] = 1;
			}

			CalibrationV2Options options = {};
			options.structSize = sizeof(CalibrationV2Options);
			options.maxIterations = 50;
			options.toolOffset[2] = m_ThisRbt.ToolOffset;
			options.ropeLengthOffset = m_ThisRbt.RopeLengthOffset;
			options.rankTolerance = 1.0e-8;
			options.maxConditionNumber = 1.0e4;
			options.huberDelta = 0.002;
			options.initialDamping = 1.0e-3;
			options.lengthParameterScale = 0.001;
			options.angleParameterScale = 0.001;
			options.maxLengthCorrection = 0.005;
			options.maxAngleCorrection = 1.0 * PI / 180.0;
			options.useValidationSplit = 1;

			CalibrationV2Result v2Result = {};
			v2Result.structSize = sizeof(CalibrationV2Result);
			const int status = CalibrationV2(
				m_RobotType, m_RobotAxis, m_TotalNum,
				mdh.data(), joints.data(), lengths.data(), requested.data(), &options,
				updatedMdh.data(), parameterDelta.data(), parameterState.data(), zeroOffsets.data(),
				beforeResiduals.data(), afterResiduals.data(), &v2Result);

			CString metric;
			metric.Format(_T("%.3lf"), v2Result.beforeMae * 1000.0);
			m_ErrorBefore.SetWindowTextW(metric);
			metric.Format(_T("%.3lf"), v2Result.afterMae * 1000.0);
			m_ErrorAfter.SetWindowTextW(metric);
			m_A1.SetWindowTextW(_T("V2 report"));
			m_A2.SetWindowTextW(_T("V2 report"));
			m_D1.SetWindowTextW(_T("V2 report"));
			m_D2.SetWindowTextW(_T("V2 report"));
			m_D3.SetWindowTextW(_T("V2 report"));
			m_D4.SetWindowTextW(_T("V2 report"));

			CString jointSummary;
			jointSummary.Format(_T("V2 status=%d, active=%d, rank=%d; "),
				status, v2Result.activeParameterCount, v2Result.numericalRank);
			for (int axis = 0; axis < m_RobotAxis; ++axis)
			{
				CString item;
				const int state = parameterState[m_RobotAxis * 3 + axis];
				if (state == CALIBRATION_PARAMETER_ACTIVE)
					item.Format(_T("J%d=%.4lfdeg; "), axis + 1, zeroOffsets[axis] * 180.0 / PI);
				else
					item.Format(_T("J%d[state=%d]; "), axis + 1, state);
				jointSummary += item;
			}
			m_JointShow.SetWindowTextW(jointSummary);

			CreateDirectoryA("./Result", NULL);
			const string reportPath = "./Result/CalibrationV2_" + rbtType + "_" + m_FinishTime + ".txt";
			ofstream report(reportPath.c_str(), ios::out | ios::trunc);
			if (report.is_open())
			{
				report << "Build = " << CalibrationBuildId() << "\n";
				report << "Status = " << status << "\nMessage = " << v2Result.message << "\n";
				report << "Axis = " << m_RobotAxis << "\nSamples = " << m_TotalNum << "\n";
				report << "Tool offset(m) = " << options.toolOffset[2] << "\n";
				report << "Rope length offset(m) = " << options.ropeLengthOffset << "\n";
				report << "Anchor(m) = " << v2Result.anchor[0] << ", " << v2Result.anchor[1] << ", " << v2Result.anchor[2] << "\n";
				report << "Active parameters = " << v2Result.activeParameterCount << "\nRank = " << v2Result.numericalRank
					<< "\nCondition = " << v2Result.conditionNumber << "\n";
				report << "MAE before/after(mm) = " << v2Result.beforeMae * 1000.0 << ", " << v2Result.afterMae * 1000.0 << "\n";
				report << "Validation MAE before/after(mm) = " << v2Result.validationBeforeMae * 1000.0
					<< ", " << v2Result.validationAfterMae * 1000.0 << "\n";
				report << "Rope MAE acceptance limit(mm) = " << kAcceptedRopeMaeMeters * 1000.0 << "\n";
				const char* groups[4] = { "a", "d", "alpha", "q0" };
				for (int group = 0; group < 4; ++group)
					for (int axis = 0; axis < m_RobotAxis; ++axis)
					{
						const int index = group * m_RobotAxis + axis;
						report << groups[group] << (axis + 1) << ": state=" << parameterState[index]
							<< ", requested=" << requested[index] << ", delta=" << parameterDelta[index] << "\n";
					}
				const char* mdhGroups[5] = { "a", "d", "alpha", "theta", "beta" };
				for (int group = 0; group < 5; ++group)
					for (int axis = 0; axis < m_RobotAxis; ++axis)
					{
						const int index = group * m_RobotAxis + axis;
						report << "mdh." << mdhGroups[group] << (axis + 1)
							<< ": before=" << mdh[index] << ", after=" << updatedMdh[index] << "\n";
					}
				for (int axis = 0; axis < m_RobotAxis; ++axis)
					report << "zeroOffset.q" << (axis + 1) << "(rad)=" << zeroOffsets[axis]
						<< ", (deg)=" << zeroOffsets[axis] * 180.0 / PI << "\n";
				for (int sample = 0; sample < m_TotalNum; ++sample)
				{
					report << "sample[" << sample << "]: measuredRaw(m)=" << lengths[sample]
						<< ", measuredEffective(m)=" << lengths[sample] + options.ropeLengthOffset
						<< ", residual(m)=" << beforeResiduals[sample] << " -> " << afterResiduals[sample]
						<< ", q(rad)=";
					for (int axis = 0; axis < m_RobotAxis; ++axis)
						report << (axis == 0 ? "" : ",") << joints[axis * m_TotalNum + sample];
					report << "\n";
				}
				report << "WriteBackFeatureEnabled = " << (kEnableV2WriteBack ? 1 : 0) << "\n";
				report << "WriteBackPerformed = 0\n";
				report << "WriteBackBlockReason = HE3 seven-axis controller IDN/unit/sign mapping is not verified\n";
			}

			const bool validationAccepted = v2Result.validationSampleCount == 0
				|| v2Result.validationAfterMae <= kAcceptedRopeMaeMeters;
			const bool residualAccepted = status == CALIBRATION_V2_OK
				&& v2Result.afterMae <= kAcceptedRopeMaeMeters
				&& validationAccepted;
			CString resultMessage;
			if (residualAccepted)
			{
				resultMessage.Format(_T("HE3七轴标定计算通过。\n\n拉线平均残差：%.3lf mm\n验证点平均残差：%.3lf mm\n\n结果报告：%S\n参数写回仍保持锁定。"),
					v2Result.afterMae * 1000.0, v2Result.validationAfterMae * 1000.0, reportPath.c_str());
				MessageBox(resultMessage, _T("CalibrationV2 计算通过"), MB_OK | MB_ICONINFORMATION);
			}
			else
			{
				resultMessage.Format(_T("HE3七轴标定计算未通过残差/安全验收。\n\n状态码：%d\n拉线平均残差：%.3lf mm\n验证点平均残差：%.3lf mm\n验收上限：%.3lf mm\n\n本次结果只写诊断报告，不会写回机器人。\n报告：%S"),
					status, v2Result.afterMae * 1000.0, v2Result.validationAfterMae * 1000.0,
					kAcceptedRopeMaeMeters * 1000.0, reportPath.c_str());
				MessageBox(resultMessage, _T("CalibrationV2 结果被拒绝"), MB_OK | MB_ICONWARNING);
			}

			CString debugSummary;
			debugSummary.Format(_T("[GUI CalibrationV2] build=%S, status=%d, MAE(mm)=%.3lf -> %.3lf, report=%S\n"),
				CalibrationBuildId(), status, v2Result.beforeMae * 1000.0, v2Result.afterMae * 1000.0, reportPath.c_str());
			OutputDebugString(debugSummary);
			MachineTrace("CALIBRATION", std::string("status=") + std::to_string(status)
				+ "; message=" + v2Result.message
				+ "; active=" + std::to_string(v2Result.activeParameterCount)
				+ "; rank=" + std::to_string(v2Result.numericalRank)
				+ "; condition=" + std::to_string(v2Result.conditionNumber)
				+ "; maeBeforeMm=" + std::to_string(v2Result.beforeMae * 1000.0)
				+ "; maeAfterMm=" + std::to_string(v2Result.afterMae * 1000.0)
				+ "; validationBeforeMm=" + std::to_string(v2Result.validationBeforeMae * 1000.0)
				+ "; validationAfterMm=" + std::to_string(v2Result.validationAfterMae * 1000.0)
				+ "; report=" + reportPath);
			m_V2WriteBackAllowed = kEnableV2WriteBack
				&& residualAccepted;
			m_SaveBtn.EnableWindow(false);
			m_CalibBtn.EnableWindow(false);
			return;
		}

		// Only the legacy non-6/7-axis path reaches this point.
		EncoderValue += VectorXd::Ones(m_TotalNum) * m_ThisRbt.RopeLengthOffset;
		Calibration(m_RobotType, m_RobotAxis, m_ThisRbtDH, m_RbtJoint, EncoderValue, m_CalibResult, oldError, nowError);

		//单位转换从 m 转为 mm
		for (int ii = 0; ii < m_TotalNum; ii++) {
			totalOld += abs(oldError[0][ii]) * 1000;
			totalNow += abs(nowError[0][ii]) * 1000;
		}
		aveOld = totalOld / m_TotalNum;
		aveNow = totalNow / m_TotalNum;
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

	delete[] oldErrorData;
	delete[] nowErrorData;
	m_SaveBtn.EnableWindow(true);
	m_CalibBtn.EnableWindow(false);
}

//保存按钮,用于保存末端距离拉线编码器的距离
void RopeEncoderCalibrationDlg::OnBnClickedButton8()
{
	if (kOfflinePreviewMode || ((m_RobotAxis == 6 || m_RobotAxis == 7) && !m_V2WriteBackAllowed))
	{
		MachineTrace("WRITEBACK_BLOCKED", std::string("offline=") + (kOfflinePreviewMode ? "true" : "false")
			+ "; featureEnabled=" + (kEnableV2WriteBack ? "true" : "false")
			+ "; resultAllowed=" + (m_V2WriteBackAllowed ? "true" : "false")
			+ "; reason=7-axis controller IDN mapping and conversion are not yet verified");
		MessageBox(_T("CalibrationV2结果目前仅允许离线诊断，尚未通过激光/生产验收，禁止写回机器人。"),
			_T("安全保护"), MB_OK | MB_ICONWARNING);
		m_SaveBtn.EnableWindow(false);
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

		RoughCalibrationDlg roughCalibDlg(this);
		m_ActiveRoughCalibrationDlg = &roughCalibDlg;
		roughCalibDlg.DoModal();
		m_ActiveRoughCalibrationDlg = nullptr;
		m_CalibTypeCombo.SelectString(0, _T("拉线编码器"));
		DeleteTemp();
	}
}

//回机器人零点
void RopeEncoderCalibrationDlg::OnBnClickedButton10()
{
	if (kOfflinePreviewMode)
	{
		MessageBox(_T("离线预览模式禁止机器人回零。"), _T("安全保护"), MB_OK | MB_ICONWARNING);
		return;
	}
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
	if (kOfflinePreviewMode)
	{
		MessageBox(_T("离线预览模式未建立机器人连接。"), _T("安全保护"), MB_OK | MB_ICONINFORMATION);
		return;
	}
	MachineTrace("DISCONNECT", "operator requested disconnect");
	m_ThreadRun = false;
	m_KeepConnect = false;
	if (KeepConnectThread != NULL && KeepConnectThread->m_hThread != NULL)
		ResumeThread(KeepConnectThread->m_hThread);
	PowerOff();
	QKMLinkDisconnect();
	m_Client.Close();
	m_IsConnect = false;
	MachineTrace("DISCONNECT", "power-off requested and communication disconnected");
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
	delete[] m_RobotHere;
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
	char* currentPath = getcwd(NULL, 0);
	string curPath = currentPath != NULL ? currentPath : "";
	free(currentPath);
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
	GetDlgItem(IDC_BUTTON1)->EnableWindow(!kOfflinePreviewMode);

	string msg = "Print Robot Cfg:\n";
	msg += (" Robot Type(0:Scara; 1:Delta; 2:SixAxis; 3:Cobot) = ") + to_string(m_RobotType) + "\n";
	msg += (" Robot Axis = ") + to_string(m_RobotAxis) + "\n";
	msg += (" Calbration location number = ") + to_string(m_TotalNum) + "\n";
	msg += (" Tool offset(length) = ") + to_string(m_ThisRbt.ToolOffset) + "\n";
	msg += (" Rope length offset = ") + to_string(m_ThisRbt.RopeLengthOffset) + "\n";
	msg += (" mDH a = ") + rowToString(m_ThisRbtDH, 0) + "\n";
	msg += (" mDH alpha = ") + rowToString(m_ThisRbtDH, 2) + "\n";
	msg += (" mDH d = ") + rowToString(m_ThisRbtDH, 1) + "\n";
	msg += (" mDH theta = ") + rowToString(m_ThisRbtDH, 3) + "\n";
	msg += (" mDH beta = ") + rowToString(m_ThisRbtDH, 4) + "\n";
	msg += (" Move direction = ") + rowToString(m_ThisRbtDH, 5) + "\n";
	msg += (" Encoder line number = ") + rowToString(m_ThisRbtDH, 6) + "\n";
	WriteLog(-999999, const_cast<char*>(msg.data()));
	MachineTrace("CONFIG", "robot=" + rbtType + "; type=" + std::to_string(m_RobotType)
		+ "; axis=" + std::to_string(m_RobotAxis) + "; samples=" + std::to_string(m_TotalNum)
		+ "; toolOffsetM=" + std::to_string(m_ThisRbt.ToolOffset)
		+ "; ropeOffsetM=" + std::to_string(m_ThisRbt.RopeLengthOffset)
		+ "; motionTimeoutMs=" + std::to_string(m_WaitTime)
		+ "; feedbackTimeoutMs=" + std::to_string(m_ShortWaitTime)
		+ "; pointFile=" + m_FilePath);
	MachineTrace("CONFIG", "mdh.a=" + rowToString(m_ThisRbtDH, 0));
	MachineTrace("CONFIG", "mdh.dWithTool=" + rowToString(m_ThisRbtDH, 1));
	MachineTrace("CONFIG", "mdh.alpha=" + rowToString(m_ThisRbtDH, 2));
	for (int point = 0; point < m_TotalNum; ++point)
		MachineTrace("CONFIG_POSE", "index=" + std::to_string(point + 1) + "; targetDeg="
			+ CStringA(m_List.GetItemText(point, 1)).GetString());
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
		RoughCalibrationDlg roughCalibDlg(this);
		m_ActiveRoughCalibrationDlg = &roughCalibDlg;
		roughCalibDlg.DoModal();
		m_ActiveRoughCalibrationDlg = nullptr;
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
	MachineTrace("TX", msg);
	long lPacketID = QKMLinkSend(msg);
	MachineTrace("TX_RESULT", "packetId=" + std::to_string(lPacketID));

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
	MachineTrace("SETUP", "begin non-motion communication setup");

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

	sSend = (CString)(m_CMDDefineLoc + m_LocName).c_str();
	SendCmd(sSend);

	sSend = (CString)(m_CMDProfile).c_str();
	SendCmd(sSend);

	sSend = (CString)(m_CMDProfileSet).c_str();
	SendCmd(sSend);
	MachineTrace("SETUP", "non-motion communication setup complete; no power/home command sent");
}

//运动点位函数
void RopeEncoderCalibrationDlg::MoveLocation(int target)
{
	// 1.获取指令
	CString sSend, sResult;
	BSTR bsSend = NULL, bsCommand = NULL, bsState = NULL, bsSourceID = NULL, bsDestinationID = NULL;
	sSend = m_List.GetItemText(target, 1);
	MachineTrace("POSE_COMMAND", "index=" + std::to_string(target + 1) + "/" + std::to_string(m_TotalNum)
		+ "; targetDeg=" + CStringA(sSend).GetString());

	sSend = (CString)m_LocName.c_str() + "=" + sSend;
	SendCmd(sSend);

	sSend = (CString)m_MoveCmd.c_str();
	TargetPacketID = SendCmd(sSend);
	SetTargetPacketID(TargetPacketID);
	m_MoveReceive = true;
	MachineTrace("POSE_COMMAND", "index=" + std::to_string(target + 1) + "; movePacketId=" + std::to_string(TargetPacketID));
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
	MachineTrace("POSE_MEASUREMENT", "index=" + std::to_string(static_cast<int>(wParam) + 1)
		+ "; encoderRaw=" + std::to_string(static_cast<long long>(lParam))
		+ "; encoderHome=" + std::to_string(m_EncoderHomeValue)
		+ "; ropeDeltaMm=" + std::to_string(EncoderValue));
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
	MachineTrace("ACQUISITION_COMPLETE", "saved joint and rope data; robot=" + rbtType
		+ "; samples=" + std::to_string(m_TotalNum) + "; finish=" + m_FinishTime);
	delete[] list;
	const int powerOffResult = PowerOff();
	MachineTrace("POWER_OFF", "automatic power-off after acquisition returned " + std::to_string(powerOffResult));
	if (powerOffResult != 0)
	{
		MessageBox(_T("60点数据已经保存，但自动下电未收到成功返回。请立即检查控制器伺服状态，必要时人工下电。"),
			_T("下电状态需要确认"), MB_OK | MB_ICONWARNING);
	}
	MachineTrace("CALIBRATION", "starting CalibrationV2 automatically from the captured 60-pose dataset");
	OnBnClickedButton7();

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
	MachineTrace("SESSION", "window close requested");
	m_ThreadRun = false;
	m_KeepConnect = false;
	if (m_IsConnect)
	{
		CString abortCommand;
		if (iAPIVersionType == APIVersionType_1)
			abortCommand = (CString)m_CMDAbroted.c_str();
		else
			abortCommand = (CString)m_CMDThreadAbroted.c_str();
		SendCmd(abortCommand);
		PowerOff();
		QKMLinkDisconnect();
		m_IsConnect = false;
		MachineTrace("SESSION", "connected session aborted, power-off requested, QKMLink disconnected");
	}
	// Offline preview deliberately skips endpoint.init(). Calling dispose() in
	// that state dereferences the null websocket worker thread during WM_CLOSE.
	// Keep shutdown symmetric with initialization: dispose only after the
	// communication layer has actually been enabled.
	if (!kOfflinePreviewMode)
	{
		endpoint.dispose();
	}
	MachineTrace("SESSION", "shutdown complete");

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
	MachineTrace("RX", "packetId=" + std::to_string(static_cast<long>(wParam)) + "; " + std::string(msg));

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
			MachineTrace("POSE_BEGIN", "index=" + std::to_string(i + 1) + "/" + std::to_string(m_TotalNum));
			if (!m_ThreadRun) {
				MachineTrace("POSE_ABORT", "operator/thread stop before index=" + std::to_string(i + 1));
				PostMessageA(pDlg->m_hWnd, WM_BUTTON_ENABLE, (WPARAM)i, (LPARAM)i);
				return 0;
			}
			ResumeThread(KeepConnectThread->m_hThread);
			PostMessageA(pDlg->m_hWnd, WM_ROBOT_MOVE, (WPARAM)i, (LPARAM)i);
			if (WAIT_TIMEOUT == WaitForSingleObject(m_MoveEvent, m_WaitTime)) {
				MachineTrace("POSE_TIMEOUT", "move timeout at index=" + std::to_string(i + 1));
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
				MachineTrace("POSE_TIMEOUT", "rope encoder read failed at index=" + std::to_string(i + 1));
				PostMessageA(pDlg->m_hWnd, WM_TIME_OUT, (WPARAM)m_EncoderTimeout, (LPARAM)m_EncoderTimeout);
				PostMessageA(pDlg->m_hWnd, WM_BUTTON_ENABLE, (WPARAM)i, (LPARAM)i);
				return -1;
			}

			PostMessageA(pDlg->m_hWnd, WM_GET_ROBOT_JOINT, (WPARAM)i, (LPARAM)EncoderValue);
			if (WAIT_TIMEOUT == WaitForSingleObject(m_WhereEvent, m_ShortWaitTime)) {
				MachineTrace("POSE_TIMEOUT", "robot actual joint read failed at index=" + std::to_string(i + 1));
				if (m_ThreadRun) {
					PostMessageA(pDlg->m_hWnd, WM_TIME_OUT, (WPARAM)m_EncoderTimeout, (LPARAM)m_EncoderTimeout);
				}
				PostMessageA(pDlg->m_hWnd, WM_BUTTON_ENABLE, (WPARAM)i, (LPARAM)i);
				return -1;
			}

			ResetEvent(m_WhereEvent);
			ResetEvent(m_MoveEvent);
			PostMessageA(pDlg->m_hWnd, WM_DISPLAY_CHANGE, (WPARAM)i, (LPARAM)EncoderValue);
			MachineTrace("POSE_END", "index=" + std::to_string(i + 1)
				+ "; encoderRaw=" + std::to_string(EncoderValue) + "; robot feedback received");

		}

		ResumeThread(KeepConnectThread->m_hThread);
		PostMessageA(pDlg->m_hWnd, WM_ENABLE_BTN, (WPARAM)NULL, (LPARAM)EncoderValue);
		PostMessageA(pDlg->m_hWnd, WM_SAVE_JOINT, (WPARAM)NULL, (LPARAM)EncoderValue);
		MachineTrace("RUN_COMPLETE", "all poses acquired; waiting for GUI save and CalibrationV2 calculation");

		return 0;
	}
	catch (const std::exception&)
	{
		return -1;
	}
	OutputDebugString(_T("break"));
}
#pragma endregion
