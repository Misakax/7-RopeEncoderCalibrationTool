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
#pragma endregion

//QKMLink 通信初始化
bool QKMLinkInit() {

	HRESULT hr = E_FAIL;
	hr = CoInitialize(NULL);
	hr = CoCreateInstance(CLSID_QKMLinkClient, NULL, CLSCTX_INPROC_SERVER, IID_IQKMLinkClient, (void**)&g_IQKMLinkClient);
	if (FAILED(hr))
	{
		AfxMessageBox(_T("Failed"));
		return FALSE;
	}
}

//连接
bool QKMLinkConnect(CString strIP, PVOID lParam) {
	long lResult;
	// 1.配置客户端
	DWORD dwThreadID;

	BSTR bsServerAddr = ::SysAllocString(strIP);
	BSTR bsSourceID = ::SysAllocString(L"0001:RopeEncoder");

	// （1）设置需要连接的服务器IP地址
	g_IQKMLinkClient->put_ServerAddress(bsServerAddr);
	// （2）设置需要连接的服务器IP端口
	g_IQKMLinkClient->put_ServerPort(2088);
	// （3）设置本客户端的ID号
	g_IQKMLinkClient->put_SourceID(bsSourceID);
	// （4）设置接收模式：0，使用线程接收；1，使用事件回调接收
	g_IQKMLinkClient->put_NoticeMode(0);
	// （5）是否使用debug模式
	g_IQKMLinkClient->put_DebugMode(VARIANT_FALSE);
	// （6）连接服务器
	g_IQKMLinkClient->Connect(&lResult);
	if (lResult < 0)
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
		g_IQKMLinkClient->CurrentState(&lResult);
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
	// 2.创建客户端接收线程，并释放线程句柄
	hClientRecieve = CreateThread(NULL, 0, ClientRecieveData, lParam, 0, &dwThreadID);
	

	// 3.释放BSTR资源
	::SysFreeString(bsServerAddr);
	::SysFreeString(bsSourceID);

	return true;
}

//断开连接
bool QKMLinkDisconnect() {
	try
	{
		
		_threadRun = false;		
		TerminateThread(hClientRecieve, 0);
		g_IQKMLinkClient->Close();
		CloseHandle(hClientRecieve);
		return true;
	}
	catch (const std::exception&)
	{
		return false;
	}
}

//解析当前角度
int DecodeRobotWhereAngle(CString tMessage)
{
	try
	{
		const int firstSpace = tMessage.Find(_T(' '));
		if (firstSpace <= 0 || firstSpace >= tMessage.GetLength() - 1)
		{
			return -1;
		}

		const int errCode = _ttoi(tMessage.Left(firstSpace));
		if (errCode < 0)
		{
			return errCode;
		}

		CString payload = tMessage.Mid(firstSpace + 1);
		payload.Trim();

		CString locResult;
		int begin = 0;
		for (int ii = 0; ii < m_RobotAxis; ++ii)
		{
			int comma = payload.Find(_T(','), begin);
			if (comma < 0)
			{
				if (ii != m_RobotAxis - 1)
				{
					return -1;
				}
				comma = payload.GetLength();
			}

			CString value = payload.Mid(begin, comma - begin);
			value.Trim();
			if (value.IsEmpty())
			{
				return -1;
			}

			if (!locResult.IsEmpty())
			{
				locResult += _T(" ");
			}
			locResult += value;
			begin = comma + 1;
		}

		if (m_LocIdx < 0)
		{
			return -1;
		}

		m_RobotHere[m_LocIdx] = locResult;
		return 0;
	}
	catch (...)
	{
		return -1;
	}
}

CString DecodeEncoderValue(CString msg)
{
	try
	{
		const int firstSpace = msg.Find(_T(' '));
		if (firstSpace <= 0 || firstSpace >= msg.GetLength() - 1)
		{
			return _T("");
		}

		const int errCode = _ttoi(msg.Left(firstSpace));
		if (errCode < 0)
		{
			return _T("");
		}

		CString result = msg.Mid(firstSpace + 1);
		result.Trim();
		return result;
	}
	catch (...)
	{
		return _T("");
	}
}

// QKMLink receive thread
DWORD WINAPI ClientRecieveData(PVOID lParam)
{
	RopeEncoderCalibrationDlg* dlgLinkTest = (RopeEncoderCalibrationDlg*)lParam;

	HRESULT hrCo = CoInitialize(NULL);
	if (FAILED(hrCo))
	{
		return -1;
	}

	int moveFeedback = 0;

	while (_threadRun)
	{
		BSTR bsMessage = NULL;
		BSTR bsCommand = NULL;
		IQKMLinkResult* pIResultClientRcv = NULL;
		long lErrorCode = -1;

		HRESULT hrReceive = g_IQKMLinkClient->Receive(
			&bsMessage, &pIResultClientRcv, &lErrorCode);

		CString sMessage;
		if (bsMessage != NULL)
		{
			sMessage = bsMessage;
		}

		// QKMLink normally wraps payload in '[' and ']'.
		// Never use a negative CString Right/Left count on malformed input.
		const int rawLength = sMessage.GetLength();
		if (rawLength >= 2 &&
			sMessage.GetAt(0) == _T('[') &&
			sMessage.GetAt(rawLength - 1) == _T(']'))
		{
			sMessage = sMessage.Mid(1, rawLength - 2);
		}

		if (FAILED(hrReceive) || lErrorCode != 0 || pIResultClientRcv == NULL)
		{
			if (bsMessage != NULL)
			{
				::SysFreeString(bsMessage);
			}
			if (bsCommand != NULL)
			{
				::SysFreeString(bsCommand);
			}
			if (pIResultClientRcv != NULL)
			{
				pIResultClientRcv->Release();
			}
			Sleep(1);
			continue;
		}

		long lPacketID = -1;
		long lSequenceNum = -1;
		long lResultCode = -1;
		pIResultClientRcv->get_Code(&lResultCode);
		pIResultClientRcv->get_Command(&bsCommand);
		pIResultClientRcv->get_PacketID(&lPacketID);
		pIResultClientRcv->get_SequenceNum(&lSequenceNum);

		OutputDebugString(sMessage + _T("\n"));

		bool skipRemainingHandlers = false;

		if (!((lPacketID == -2) && sMessage.IsEmpty()))
		{
			SendMessage(dlgLinkTest->m_hWnd, WM_WRITE_LOG,
				(WPARAM)lPacketID, (LPARAM)&sMessage);

			if (lPacketID == m_TargetPacketID && lSequenceNum == 0)
			{
				if (m_MoveReceive)
				{
					++moveFeedback;
					if ((moveFeedback % 2) == 0)
					{
						if (sMessage == _T("1"))
						{
							moveFeedback = 0;
							SetEvent(m_MoveEvent);
							m_MoveReceive = false;
						}
						else
						{
							// Preserve old protocol behaviour but still clean up resources.
							skipRemainingHandlers = true;
						}
					}
				}

				if (!skipRemainingHandlers && m_SystemVersion)
				{
					CString testA = _T("PPPP");
					SendMessage(dlgLinkTest->m_hWnd, WM_WRITE_LOG,
						(WPARAM)lPacketID, (LPARAM)&testA);
					m_SystemVersionStr = sMessage;
					m_SystemVersion = false;
				}

				if (!skipRemainingHandlers && m_Where)
				{
					if (m_LocIdx >= 0)
					{
						m_RobotHere[m_LocIdx] = sMessage;
						const int decodeStatus = DecodeRobotWhereAngle(sMessage);
						if (decodeStatus == 0 && !m_RobotHere[m_LocIdx].IsEmpty())
						{
							++m_LocIdx;
							SetEvent(m_WhereEvent);
							m_Where = false;
						}
						else
						{
							m_Where = false;
							PostMessageA(dlgLinkTest->m_hWnd, WM_ENABLE_BTN,
								(WPARAM)NULL, (LPARAM)NULL);
						}
					}
					else
					{
						m_Where = false;
					}
				}

				if (!skipRemainingHandlers && m_bGeneralWaitValue)
				{
					m_sGeneralWaitValue = CT2A(sMessage.GetString());
					m_bGeneralWaitValue = false;
					SetEvent(m_IDNEvent);
				}

				if (!skipRemainingHandlers && m_GetEncoder)
				{
					CString sTemp = DecodeEncoderValue(sMessage);
					SendMessage(dlgLinkTest->m_hWnd, WM_GET_ENCODER,
						(WPARAM)m_AxisIdx, (LPARAM)&sTemp);
					SetEvent(m_WhereEvent);
				}
			}
		}

		if (bsMessage != NULL)
		{
			::SysFreeString(bsMessage);
		}
		if (bsCommand != NULL)
		{
			::SysFreeString(bsCommand);
		}
		if (pIResultClientRcv != NULL)
		{
			pIResultClientRcv->Release();
		}
	}

	CoUninitialize();
	return 0;
}

//QKMLink 发送函数
int QKMLinkSend(CString msg) {
	if (g_IQKMLinkClient == NULL)
	{
		return -1;
	}

	CString sSend = _T("[") + msg + _T("]");
	BSTR bsSend = sSend.AllocSysString();
	BSTR bsMode = ::SysAllocString(L"ACKOFF");
	IQKMLinkResult* pIResultClientSend = NULL;
	long lPacketID = -1;

	HRESULT hrSend = g_IQKMLinkClient->Send(
		bsSend, bsMode, 5, &pIResultClientSend);

	if (SUCCEEDED(hrSend) && pIResultClientSend != NULL)
	{
		pIResultClientSend->get_PacketID(&lPacketID);
	}

	if (pIResultClientSend != NULL)
	{
		pIResultClientSend->Release();
	}
	if (bsSend != NULL)
	{
		::SysFreeString(bsSend);
	}
	if (bsMode != NULL)
	{
		::SysFreeString(bsMode);
	}

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