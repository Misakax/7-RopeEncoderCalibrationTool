// RopeEncoderCalibrationDlg.cpp : 实现文件
//两个界面通过使用同一个对象进行 QKMLink 通讯

#include "stdafx.h"
#include "QKMLinkComm.h"
#include "afxdialogex.h"
#include "qkmlinklib_i.h"
#include <direct.h> 
#include <fstream>
#include <iostream>
#include "afxcmn.h"
#include "afxwin.h"
#include "RopeEncoderCalibrationDlg.h"
#include "RoughCalibrationDlg.h"
#include <comdef.h>

using namespace std;

#pragma region 全局变量

//定义 QKMLink 客户端
CComPtr<IQKMLinkClient> g_IQKMLinkClient = NULL;
// 声明 QKMLink 客户端的接收线程
DWORD WINAPI ClientRecieveData(PVOID lParam);
//接收线程的句柄
HANDLE hClientRecieve;
//运动的事件
extern HANDLE m_MoveEvent;
//运动的事件
extern HANDLE m_WhereEvent;
//运动的事件
extern HANDLE m_GetEncoderEvent;
//IDN 读写的事件
extern HANDLE m_IDNEvent;
//等待的目标 packetID
int m_TargetPacketID = -1;
//接受运动反馈的判断位
extern bool m_MoveReceive;
//获取机器人当前角度的判断位
extern bool m_Where;
//获取固件版本判断位
extern bool m_SystemVersion;
//获取 IDN 值判断位
extern bool m_bGeneralWaitValue;
//获取 IDN 值
extern string m_sGeneralWaitValue;
//固件版本字符串
extern CString m_SystemVersionStr;
//获取编码器值的判断位
extern bool m_GetEncoder;
//获取轴的索引
extern int m_AxisIdx;
//点位索引
extern int m_LocIdx;
//本次标定要求的点位总数
extern int m_TotalNum;
//点位数组
extern CString *m_RobotHere;
//机器人索引
extern int m_RobotIdx;
//机器人轴数
extern int m_RobotAxis;
//运动反馈判断为,当判断位为2，证明运动完成
int _moveFeedback = 0;
//通讯线程循环判断为
bool _threadRun = true;
HRESULT g_QKMLinkInitError = S_OK;
#pragma endregion

long QKMLinkLastInitError()
{
	return static_cast<long>(g_QKMLinkInitError);
}

CString QKMLinkLastInitErrorText()
{
	_com_error error(g_QKMLinkInitError);
	CString message;
	message.Format(_T("QKMLinkClient 创建失败。\n\nHRESULT: 0x%08X (%ld)\n系统说明: %s\nCLSID: {662E9227-73A3-4DA4-B22E-99F8E4C143DD}\n进程架构: Win32/x86\n\n请确认原版在当前电脑上此刻也能连接；若错误为 0x80040154，需要安装或重新注册与原版匹配的 32 位 QKMLink 组件。"),
		static_cast<unsigned long>(g_QKMLinkInitError), static_cast<long>(g_QKMLinkInitError), error.ErrorMessage());
	return message;
}

//QKMLink 通信初始化
bool QKMLinkInit() {

	const HRESULT comInit = CoInitialize(NULL);
	if (FAILED(comInit) && comInit != RPC_E_CHANGED_MODE)
	{
		g_QKMLinkInitError = comInit;
		CString diagnostic = QKMLinkLastInitErrorText();
		OutputDebugString(_T("[QKMLinkInit] CoInitialize failed: "));
		OutputDebugString(diagnostic);
		OutputDebugString(_T("\n"));
		return FALSE;
	}
	g_IQKMLinkClient.Release();
	g_QKMLinkInitError = CoCreateInstance(CLSID_QKMLinkClient, NULL, CLSCTX_INPROC_SERVER,
		IID_IQKMLinkClient, reinterpret_cast<void**>(&g_IQKMLinkClient));
	if (FAILED(g_QKMLinkInitError))
	{
		CString diagnostic = QKMLinkLastInitErrorText();
		OutputDebugString(_T("[QKMLinkInit] CoCreateInstance failed: "));
		OutputDebugString(diagnostic);
		OutputDebugString(_T("\n"));
		return FALSE;
	}
	g_QKMLinkInitError = S_OK;
	return TRUE;
}

//连接
bool QKMLinkConnect(CString strIP, PVOID lParam) {
	long lResult = -1;
	// 1.配置客户端
	DWORD dwThreadID;
	if (g_IQKMLinkClient == NULL)
		return false;

	CComBSTR bsServerAddr(strIP);
	CComBSTR bsSourceID(L"0001:RopeEncoder");

	// （1）设置需要连接的服务器IP地址
	if (FAILED(g_IQKMLinkClient->put_ServerAddress(bsServerAddr))) return false;
	// （2）设置需要连接的服务器IP端口
	if (FAILED(g_IQKMLinkClient->put_ServerPort(2088))) return false;
	// （3）设置本客户端的ID号
	if (FAILED(g_IQKMLinkClient->put_SourceID(bsSourceID))) return false;
	// （4）设置接收模式：0，使用线程接收；1，使用事件回调接收
	if (FAILED(g_IQKMLinkClient->put_NoticeMode(0))) return false;
	// （5）是否使用debug模式
	if (FAILED(g_IQKMLinkClient->put_DebugMode(VARIANT_FALSE))) return false;
	// （6）连接服务器
	if (FAILED(g_IQKMLinkClient->Connect(&lResult)) || lResult < 0)
	{
		//MessageBox(_T("Local client connect to remote server fail!"));
		return false;
	}
	else
	{
		//MessageBox(_T("Local client connect to remote server success!"));
	}
	long lCurrentStateCount = 0;
	while(true)
	{
		//当前状态 1：已连接且已交换 ID， 0：已连接但未交换 ID，- 1：未连接
		if (FAILED(g_IQKMLinkClient->CurrentState(&lResult)))
			return false;
		if (lResult == 1)
		{//已连接且已交换ID
			break;
		}
		if ( (lCurrentStateCount >= 100) || (lResult == -1))
		{//等待状态切换为"已连接且已交换ID"超时 或者 未连接
			return false;
		}
		lCurrentStateCount = lCurrentStateCount + 1;
		Sleep(20);
	}

	_threadRun = true;
	// 2.创建客户端接收线程。句柄在断开连接时回收。
	hClientRecieve = CreateThread(NULL, 0, ClientRecieveData, lParam, 0, &dwThreadID);
	if (hClientRecieve == NULL)
	{
		g_IQKMLinkClient->Close();
		return false;
	}

	return true;
}

//断开连接
bool QKMLinkDisconnect() {
	_threadRun = false;
	HRESULT closeResult = S_OK;
	if (g_IQKMLinkClient != NULL)
		closeResult = g_IQKMLinkClient->Close();
	if (hClientRecieve != NULL)
	{
		const DWORD waitResult = WaitForSingleObject(hClientRecieve, 2000);
		if (waitResult == WAIT_TIMEOUT)
			return false;
		CloseHandle(hClientRecieve);
		hClientRecieve = NULL;
	}
	return SUCCEEDED(closeResult);
}

//解析当前角度
int DecodeRobotWhereAngle(CString tMessage)
{
	try
	{
		if (m_RobotHere == nullptr || m_LocIdx < 0 || m_LocIdx >= m_TotalNum || m_RobotAxis <= 0)
			return -2;
		CString locResult;
		char spilt = ',';
		int tIdx = 0;
		int errCode = 0;
		const CString errCodeText = tMessage.Left(tMessage.Find(' '));
		errCode = _ttoi(errCodeText);
		if (errCode < 0) {
			return errCode;
		}

		tMessage = tMessage.Right(tMessage.GetLength() - tMessage.Find(' '));

		for (int ii = 0; ii < m_RobotAxis; ii++)
		{
			tIdx = tMessage.Find(spilt, tIdx + 1);
		}

		if (tIdx <= 0)
			return -3;
		locResult = tMessage.Left(tIdx);

		locResult = locResult.TrimLeft();

		locResult.Replace(spilt, ' ');
		m_RobotHere[m_LocIdx] = locResult;
		return 0;
	}
	catch (const std::exception&)
	{
		return -1;
	}
}

//解析编码器值
CString DecodeEncoderValue(CString msg)
{
	try
	{
		CString locResult;
		char spilt = ',';
		int tIdx = 0;
		int errCode = 0;
		msg.Left(msg.Find(' ')).Format(_T("%d"), errCode);
		if (errCode < 0) {
			return NULL;
		}

		msg = msg.Right(msg.GetLength() - msg.Find(' '));

		locResult = msg;

		locResult = locResult.TrimLeft();

		return locResult;
	}
	catch (const std::exception&)
	{
		return NULL;
	}
}

//QKMLink 接受线程
DWORD WINAPI ClientRecieveData(PVOID lParam)
{
	try
	{
		RopeEncoderCalibrationDlg *dlgLinkTest = (RopeEncoderCalibrationDlg*)lParam;
		if (dlgLinkTest == nullptr)
			return ERROR_INVALID_PARAMETER;

		// 1.配置COM环境，在线程中使用QKMLink需要使用CoInitialize，线程退出时调用
		// CoUninitialize()释放COM环境
		HRESULT hr = E_FAIL;
		hr = CoInitialize(NULL);	//工作线程使用COM，必须调用此函数初始化COM库
		int moveFeedback = 0;
		if (SUCCEEDED(hr))
		{
			while (_threadRun)
			{
				if (!::IsWindow(dlgLinkTest->GetSafeHwnd()))
					break;
				// 2.接收从机器人端发送过来的信息，包括反馈的信息和主动的信息
				BSTR bsMessage = NULL, bsCommand = NULL;
				IQKMLinkResult* pIResultClientRcv = NULL;
				CString sMessage, sTemp;
				long lErrorCode = -1;

				if (g_IQKMLinkClient == NULL)
					break;
				const HRESULT receiveHr = g_IQKMLinkClient->Receive(&bsMessage, &pIResultClientRcv, &lErrorCode);
				if (FAILED(receiveHr) || lErrorCode != 0 || pIResultClientRcv == NULL)
				{
					::SysFreeString(bsMessage);
					if (pIResultClientRcv != NULL) pIResultClientRcv->Release();
					if (_threadRun) Sleep(10);
					continue;
				}
				sMessage = bsMessage;
				if (sMessage.GetLength() >= 2)
				{
					sMessage = sMessage.Right(sMessage.GetLength() - 1);
					sMessage = sMessage.Left(sMessage.GetLength() - 1);
				}

				OutputDebugString(sMessage + "\n");

				// 处理error code = 0的情况，其他情况不需要处理
				if (lErrorCode == 0)
				{
					if (receiveHr == S_OK && pIResultClientRcv != NULL)
					{
						// 3.获取Result接口的属性，并显示
						CString sShowResult;
						long lPacketID, lSequenceNum, lResultCode;
						pIResultClientRcv->get_Code(&lResultCode);
						pIResultClientRcv->get_Command(&bsCommand);
						pIResultClientRcv->get_PacketID(&lPacketID);
						pIResultClientRcv->get_SequenceNum(&lSequenceNum);
						if ((lPacketID == -2) && (sMessage == ""))//屏蔽心跳包信息
						{
							
						}
						else
						{
							SendMessage(dlgLinkTest->m_hWnd, WM_WRITE_LOG, (WPARAM)lPacketID, (LPARAM)&sMessage);
							if (lPacketID == m_TargetPacketID && lSequenceNum == 0)
							{
								if (m_MoveReceive)
								{
									moveFeedback++;
									if (moveFeedback % 2 == 0)
									{
										if (sMessage == "1") {
											moveFeedback = 0;
											SetEvent(m_MoveEvent);
											m_MoveReceive = false;
										}
									}
								}
								if (m_SystemVersion == true)
								{
									CString testA = _T("PPPP");
									SendMessage(dlgLinkTest->m_hWnd, WM_WRITE_LOG, (WPARAM)lPacketID, (LPARAM)&testA);
									m_SystemVersionStr = sMessage;
									m_SystemVersion = false;
								}
								if (m_Where)
								{
									// Reject duplicate/late feedback before indexing the sample array.
									// The legacy callback wrote first and checked later, which could
									// write past the 60-element HE3 buffer.
									if (m_RobotHere == nullptr || m_LocIdx < 0 || m_LocIdx >= m_TotalNum)
									{
										m_Where = false;
									}
									else
									{
										const int stt = DecodeRobotWhereAngle(sMessage);
										if (stt == 0 && !m_RobotHere[m_LocIdx].IsEmpty()) {
											m_LocIdx++;
											SetEvent(m_WhereEvent);
											m_Where = false;
										}
										else
										{
											m_Where = false;
											PostMessageA(dlgLinkTest->m_hWnd, WM_ENABLE_BTN, (WPARAM)NULL, (LPARAM)NULL);
										}
									}
								}
								if(m_bGeneralWaitValue == true)
								{
									m_sGeneralWaitValue = CT2A(sMessage.GetString());
									m_bGeneralWaitValue = false;
									SetEvent(m_IDNEvent);
								}
								if (m_GetEncoder) {
									sTemp = DecodeEncoderValue(sMessage);
									SendMessage(dlgLinkTest->m_hWnd, WM_GET_ENCODER, (WPARAM)m_AxisIdx, (LPARAM)&sTemp);
									SetEvent(m_WhereEvent);
								}

							}
						}
					}
				}

				// 5.释放BSTR资源
				::SysFreeString(bsMessage);
				::SysFreeString(bsCommand);

				// 6.释放QKMLinkResult
				if (pIResultClientRcv != NULL)
					pIResultClientRcv->Release();
			}

			// 7.线程退出时，释放线程的COM环境
			CoUninitialize();
		}
		return 0;
	}
	catch (const std::exception&)
	{
		return -1;
	}
	
}

//QKMLink 发送函数
int QKMLinkSend(CString msg) {
	if (g_IQKMLinkClient == NULL)
		return -1;
	CString sSend;
	long lPacketID = -1;
	sSend = '[' + msg + ']';
	CComBSTR bsSend(sSend);
	CComBSTR bsMode(L"ACKOFF");

	IQKMLinkResult* pIResultClientSend = NULL;
	const HRESULT sendHr = g_IQKMLinkClient->Send(bsSend, bsMode, 5, &pIResultClientSend);
	if (FAILED(sendHr) || pIResultClientSend == NULL)
		return -1;
	const HRESULT packetHr = pIResultClientSend->get_PacketID(&lPacketID);
	pIResultClientSend->Release();
	if (FAILED(packetHr))
		return -1;

	return lPacketID;
}

//设置 Packetid
bool QKMLinkSetTargetPacketID(int target) {
	try
	{
		m_TargetPacketID = target;
		return true;
	}
	catch (const std::exception&)
	{
		return false;
	}
}

//QKMLink 事件重置
bool QKMLinkEventReset() {
	ResetEvent(m_MoveEvent);
	ResetEvent(m_WhereEvent);
	ResetEvent(m_IDNEvent);
	m_TargetPacketID = -1;
	_moveFeedback = 0;
	m_GetEncoder = false;
	m_Where = false;
	m_MoveReceive = false;
	m_bGeneralWaitValue = false;
	m_sGeneralWaitValue = "";
	return true;
}
