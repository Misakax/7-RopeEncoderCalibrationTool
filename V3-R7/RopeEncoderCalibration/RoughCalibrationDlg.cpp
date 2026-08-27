// RoughCalibrationDlg.cpp : 实现文件
//通过先示教机器人到零点位置,获取编码器值,并对值进行保存和上传替换

#include "stdafx.h"
#include "RopeEncoderCalibration.h"
#include "RopeEncoderCalibrationDlg.h"
#include "RoughCalibrationDlg.h"
#include "afxdialogex.h"
#include "QKMLinkComm.h"

#define EDIT_ID 10000
#define BUTTON_ID 12000

CPtrArray m_MyStatics;
CPtrArray m_MyEdits;
CPtrArray m_MyButtons;

CToolTipCtrl m_ToolTip;

using namespace std;

#pragma region 变量
// RoughCalibrationDlg 对话框
//获取轴绝对编码器值指令
CString m_CMDGetEncoder = _T("Robot.Encoder 1,");
//设置 IDN 指令
CString m_CMDIDNString = _T("SystemIDNString ");
//保存配置文件
CString m_CMDIDNSave = _T("System.Save ");
//轴关节索引
int m_AxisIdx = 0;
//获取关节角判断位
bool m_GetEncoder = false;
//目标 packetid
int tTargetPacketID = -1;
//当前界面
RoughCalibrationDlg *thisDlg;
//机器人轴数
extern int m_RobotAxis;
//机器人类型
extern int m_RobotType;
//机器人类型枚举
extern enum RobotType {
	Scara,
	Delta,
	SixAxis,
	Cobot
};
#pragma endregion

IMPLEMENT_DYNAMIC(RoughCalibrationDlg, CDialogEx)

RoughCalibrationDlg::RoughCalibrationDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_DIALOG2, pParent)
{

}

RoughCalibrationDlg::~RoughCalibrationDlg()
{
}

void RoughCalibrationDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_EDIT1, m_EncoderValue1);
	DDX_Control(pDX, IDC_EDIT2, m_EncoderValue2);
	DDX_Control(pDX, IDC_EDIT3, m_EncoderValue3);
	DDX_Control(pDX, IDC_EDIT4, m_EncoderValue4);
	DDX_Control(pDX, IDC_EDIT5, m_EncoderValue5);
	DDX_Control(pDX, IDC_EDIT6, m_EncoderValue6);
	DDX_Control(pDX, IDC_BUTTON1, m_EncoderBtn1);
	DDX_Control(pDX, IDC_BUTTON2, m_EncoderBtn2);
	DDX_Control(pDX, IDC_BUTTON3, m_EncoderBtn3);
	DDX_Control(pDX, IDC_BUTTON4, m_EncoderBtn4);
	DDX_Control(pDX, IDC_BUTTON5, m_EncoderBtn5);
	DDX_Control(pDX, IDC_BUTTON6, m_EncoderBtn6);
}

BEGIN_MESSAGE_MAP(RoughCalibrationDlg, CDialogEx)
	ON_BN_CLICKED(IDC_BUTTON7, &RoughCalibrationDlg::OnBnClickedButton7)
	ON_BN_CLICKED(IDC_BUTTON8, &RoughCalibrationDlg::OnBnClickedButton8)
	ON_BN_CLICKED(IDC_BUTTON1, &RoughCalibrationDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &RoughCalibrationDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &RoughCalibrationDlg::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_BUTTON4, &RoughCalibrationDlg::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_BUTTON5, &RoughCalibrationDlg::OnBnClickedButton5)
	ON_BN_CLICKED(IDC_BUTTON6, &RoughCalibrationDlg::OnBnClickedButton6)
	ON_MESSAGE(WM_SHOW_ENCODER, &RoughCalibrationDlg::ShowValue)
	ON_MESSAGE(WM_REPLACE_XML, &RopeEncoderCalibrationDlg::OnReplaceXmlValue)
	ON_MESSAGE(WM_WRITE_XML, &RopeEncoderCalibrationDlg::OnWriteXml)
	ON_EN_CHANGE(IDC_EDIT1, &RoughCalibrationDlg::OnEnChangeEdit1)
	ON_EN_CHANGE(IDC_EDIT2, &RoughCalibrationDlg::OnEnChangeEdit2)
	ON_EN_CHANGE(IDC_EDIT3, &RoughCalibrationDlg::OnEnChangeEdit3)
	ON_EN_CHANGE(IDC_EDIT4, &RoughCalibrationDlg::OnEnChangeEdit4)
	ON_EN_CHANGE(IDC_EDIT5, &RoughCalibrationDlg::OnEnChangeEdit5)
	ON_EN_CHANGE(IDC_EDIT6, &RoughCalibrationDlg::OnEnChangeEdit6)

	ON_CONTROL_RANGE(EN_CHANGE,EDIT_ID, EDIT_ID + 10, &RoughCalibrationDlg::OnEnChangeEdit)
	ON_COMMAND_RANGE(BUTTON_ID, BUTTON_ID + 10, &RoughCalibrationDlg::OnButtonClick)
	
END_MESSAGE_MAP()

//初始化
BOOL RoughCalibrationDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  在此添加额外的初始化
	
	thisDlg = this;
	
	EnableToolTips(TRUE);
	m_ToolTip.Create(this);
	m_ToolTip.SetDelayTime(150);

	// TODO: 在此添加控件通知处理程序代码
	CString m_StaticCaption;
	CRect rect, rect2;
	GetClientRect(&rect);
	GetDlgItem(IDC_BUTTON1)->GetWindowRect(&rect2);
	GetDlgItem(IDC_EDIT1)->GetWindowRect(&rect2);

	CFont* catchtarget = GetDlgItem(IDC_BUTTON7)->GetFont();


	ScreenToClient(&rect2);

	int perWidth = rect.Width() / 4;
	int perHeight = (rect.Height() - 40) / m_RobotAxis;
	CStatic *m_MyStatic;
	CEdit *m_MyEdit;
	CButton *m_MyButton;

	CString ToolTipString = _T("请输入整数");
	CString m_BottonString = _T("获取编码器值");

	for (int i = 0; i< m_RobotAxis; i++)
	{
		m_MyStatic = new CStatic();
		m_MyEdit = new CEdit();
		m_MyButton = new CButton();

		m_StaticCaption.Format(_T("%d轴编码器值:"), i + 1);

		m_MyStatic->Create(m_StaticCaption, WS_CHILD | WS_VISIBLE | SS_LEFT, CRect(23, i*perHeight + 22, 93, i*perHeight + 42), this);
		m_MyStatic->SetFont(catchtarget, TRUE);

		m_MyEdit->Create(WS_CHILD | WS_VISIBLE | SS_LEFT | ES_AUTOHSCROLL, CRect(103, i*perHeight + 20, 253, i*perHeight + 42), this, EDIT_ID + i);
		m_MyEdit->SetFont(catchtarget, TRUE);
		m_ToolTip.AddTool(GetDlgItem(EDIT_ID + i), ToolTipString);

		m_MyButton->Create(m_BottonString, WM_SETFONT | WS_CHILD | WS_VISIBLE | SS_CENTER, CRect(263, i*perHeight + 20, 363, i*perHeight + 42), this, BUTTON_ID + i);
		m_MyButton->SetFont(catchtarget, TRUE);

		if (m_MyStatic != NULL)
		{
			m_MyStatics.Add((void*)m_MyStatic);
		}
		if (m_MyEdit != NULL)
		{
			m_MyEdits.Add((void*)m_MyEdit);
		}
		if (m_MyButton != NULL)
		{
			m_MyButtons.Add((void*)m_MyButton);
		}

	}


	return TRUE;  // return TRUE unless you set the focus to a control
				  // 异常: OCX 属性页应返回 FALSE
}

//设置 Packetid
bool RoughCalibrationDlg::SetTargetPacketID(int target) {
	return QKMLinkSetTargetPacketID(target);
}

//替换零点值,保存配置文件
void RoughCalibrationDlg::OnBnClickedButton7()
{
	// TODO: 在此添加控件通知处理程序代码
	/*CString value;
	m_EncoderValue1.GetWindowTextW(value);
	SendMessageA(this->m_hWnd, WM_REPLACE_XML, (WPARAM)&m_IDNAxis1, (LPARAM)&value);

	m_EncoderValue2.GetWindowTextW(value);
	SendMessageA(this->m_hWnd, WM_REPLACE_XML, (WPARAM)&m_IDNAxis2, (LPARAM)&value);

	m_EncoderValue3.GetWindowTextW(value);
	SendMessageA(this->m_hWnd, WM_REPLACE_XML, (WPARAM)&m_IDNAxis3, (LPARAM)&value);

	m_EncoderValue4.GetWindowTextW(value);
	SendMessageA(this->m_hWnd, WM_REPLACE_XML, (WPARAM)&m_IDNAxis4, (LPARAM)&value);

	m_EncoderValue5.GetWindowTextW(value);
	SendMessageA(this->m_hWnd, WM_REPLACE_XML, (WPARAM)&m_IDNAxis5, (LPARAM)&value);

	m_EncoderValue6.GetWindowTextW(value);
	SendMessageA(this->m_hWnd, WM_REPLACE_XML, (WPARAM)&m_IDNAxis6, (LPARAM)&value);

	SendMessageA(this->m_hWnd, WM_WRITE_XML, (WPARAM)NULL, (LPARAM)NULL);*/

	CString* value = new CString[m_RobotAxis];

	for (int ii = 0; ii < m_RobotAxis; ii++) 
	{
		/*switch (ii)
		{
		case 0:
			m_EncoderValue1.GetWindowTextW(value[ii]);
			break;
		case 1:
			m_EncoderValue2.GetWindowTextW(value[ii]);
			break;
		case 2:
			m_EncoderValue3.GetWindowTextW(value[ii]);
			break;
		case 3:
			m_EncoderValue4.GetWindowTextW(value[ii]);
			break;
		case 4:
			m_EncoderValue5.GetWindowTextW(value[ii]);
			break;
		case 5:
			m_EncoderValue6.GetWindowTextW(value[ii]);
			break;
		default:
			break;
		}*/
		GetDlgItem(EDIT_ID + ii)->GetWindowTextW(value[ii]);
	}

	for (int ii = 0; ii < m_RobotAxis; ii++) {
		if (value[ii] == _T("")) 
		{
			CString showmsg = _T("");
			showmsg.Format(_T("%d轴的值为空"), ii + 1);
			MessageBox(showmsg, _T("错误"), MB_OK);
			return;
		}
	}

	CString sSend = m_CMDIDNString;
	CString sTemp;
	CString cSvalue = _T("");
	if (m_RobotType == Scara) 
	{
		CString* IDNArray = new CString[2];

		IDNArray[0] = "P-0-0003.0.8";
		IDNArray[1] = "P-0-0003.1.8";

		for (int ii = 0; ii < m_RobotAxis; ii++) {
			sSend = _T("");
			if (ii % 2 == 0) {
				sSend += IDNArray[0] + _T(",1,");
			}
			else
			{
				sSend += IDNArray[1] + _T(",1,");
			}
			sTemp.Format(_T("%d,%s"), ii+1, value[ii]);
			sSend += sTemp;
			//QKMLinkSend(sSend);
			OutputDebugString(sSend + '\n');

			sSend = "";

		}
		//m_EncoderValue1.GetWindowTextW(value);
		//SendMessageA(this->m_hWnd, WM_REPLACE_XML, (WPARAM)&m_IDNAxis1, (LPARAM)&value);

		//m_EncoderValue2.GetWindowTextW(value);
		//SendMessageA(this->m_hWnd, WM_REPLACE_XML, (WPARAM)&m_IDNAxis2, (LPARAM)&value);

		//m_EncoderValue3.GetWindowTextW(value);
		//SendMessageA(this->m_hWnd, WM_REPLACE_XML, (WPARAM)&m_IDNAxis3, (LPARAM)&value);

		//m_EncoderValue4.GetWindowTextW(value);
		//SendMessageA(this->m_hWnd, WM_REPLACE_XML, (WPARAM)&m_IDNAxis4, (LPARAM)&value);

	}
	else if(m_RobotType = Cobot)
	{
		// TODO: 在此添加控件通知处理程序代码
		//CString value;
		m_EncoderValue1.GetWindowTextW(cSvalue);
		SendMessageA(this->m_hWnd, WM_REPLACE_XML, (WPARAM)&m_IDNAxis1, (LPARAM)&value);

		m_EncoderValue2.GetWindowTextW(cSvalue);
		SendMessageA(this->m_hWnd, WM_REPLACE_XML, (WPARAM)&m_IDNAxis2, (LPARAM)&value);

		m_EncoderValue3.GetWindowTextW(cSvalue);
		SendMessageA(this->m_hWnd, WM_REPLACE_XML, (WPARAM)&m_IDNAxis3, (LPARAM)&value);

		m_EncoderValue4.GetWindowTextW(cSvalue);
		SendMessageA(this->m_hWnd, WM_REPLACE_XML, (WPARAM)&m_IDNAxis4, (LPARAM)&value);

		m_EncoderValue5.GetWindowTextW(cSvalue);
		SendMessageA(this->m_hWnd, WM_REPLACE_XML, (WPARAM)&m_IDNAxis5, (LPARAM)&value);

		m_EncoderValue6.GetWindowTextW(cSvalue);
		SendMessageA(this->m_hWnd, WM_REPLACE_XML, (WPARAM)&m_IDNAxis6, (LPARAM)&value);

		SendMessageA(this->m_hWnd, WM_WRITE_XML, (WPARAM)NULL, (LPARAM)NULL);
	}
	else
	{

	}

	//QKMLinkSend(msg);
	MessageBox(_T("保存成功"), _T("提示"), MB_OK);
}

//返回
void RoughCalibrationDlg::OnBnClickedButton8()
{
	m_MyStatics.RemoveAll();
	m_MyEdits.RemoveAll();
	m_MyButtons.RemoveAll();
	// TODO: 在此添加控件通知处理程序代码
	PostMessageA(m_hWnd, WM_CLOSE, 0, 0);
}

//获取轴一编码器值
void RoughCalibrationDlg::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码
	CString sSend;
	m_GetEncoder = true;
	m_AxisIdx = 1;
	sSend.Format(_T("%d"), m_AxisIdx);
	sSend = m_CMDGetEncoder + sSend;
	tTargetPacketID = QKMLinkSend(sSend);
	SetTargetPacketID(tTargetPacketID);
}

//获取轴二编码器值
void RoughCalibrationDlg::OnBnClickedButton2()
{
	// TODO: 在此添加控件通知处理程序代码
	CString sSend;
	m_GetEncoder = true;
	m_AxisIdx = 2;
	sSend.Format(_T("%d"), m_AxisIdx);
	sSend = m_CMDGetEncoder + sSend;
	tTargetPacketID = QKMLinkSend(sSend);
	SetTargetPacketID(tTargetPacketID);
}

//获取轴三编码器值
void RoughCalibrationDlg::OnBnClickedButton3()
{
	// TODO: 在此添加控件通知处理程序代码
	CString sSend;
	m_GetEncoder = true;
	m_AxisIdx = 3;
	sSend.Format(_T("%d"), m_AxisIdx);
	sSend = m_CMDGetEncoder + sSend;
	tTargetPacketID = QKMLinkSend(sSend);
	SetTargetPacketID(tTargetPacketID);
}

//获取轴四编码器值
void RoughCalibrationDlg::OnBnClickedButton4()
{
	// TODO: 在此添加控件通知处理程序代码
	CString sSend;
	m_GetEncoder = true;
	m_AxisIdx = 4;
	sSend.Format(_T("%d"), m_AxisIdx);
	sSend = m_CMDGetEncoder + sSend;
	tTargetPacketID = QKMLinkSend(sSend);
	SetTargetPacketID(tTargetPacketID);
}

//获取轴五编码器值
void RoughCalibrationDlg::OnBnClickedButton5()
{
	// TODO: 在此添加控件通知处理程序代码
	CString sSend;
	m_GetEncoder = true;
	m_AxisIdx = 5;
	sSend.Format(_T("%d"), m_AxisIdx);
	sSend = m_CMDGetEncoder + sSend;
	tTargetPacketID = QKMLinkSend(sSend);
	SetTargetPacketID(tTargetPacketID);
}

//获取轴六编码器值
void RoughCalibrationDlg::OnBnClickedButton6()
{
	// TODO: 在此添加控件通知处理程序代码
	CString sSend;
	m_GetEncoder = true;
	m_AxisIdx = 6;
	sSend.Format(_T("%d"), m_AxisIdx);
	sSend = m_CMDGetEncoder + sSend;
	tTargetPacketID = QKMLinkSend(sSend);
	SetTargetPacketID(tTargetPacketID);
}

//界面刷新事件
afx_msg LRESULT RoughCalibrationDlg::OnEncoderValueShow(WPARAM wParam, LPARAM lParam)
{
	CString* pTry = (CString*)lParam;
	CString EncoderValue = *pTry;
	SendMessageA(thisDlg->m_hWnd, WM_SHOW_ENCODER, wParam, lParam);

	return 0;
}

//刷新函数
afx_msg LRESULT RoughCalibrationDlg::ShowValue(WPARAM wParam, LPARAM lParam)
{
	CString* pTry = (CString*)lParam;

	CString EncoderValue = *pTry;
	/*switch (wParam)
	{
	case 1:
		m_EncoderValue1.SetWindowTextW(EncoderValue);
		break;
	case 2:
		m_EncoderValue2.SetWindowTextW(EncoderValue);
		break;
	case 3:
		m_EncoderValue3.SetWindowTextW(EncoderValue);
		break;
	case 4:
		m_EncoderValue4.SetWindowTextW(EncoderValue);
		break;
	case 5:
		m_EncoderValue5.SetWindowTextW(EncoderValue);
		break;
	case 6:
		m_EncoderValue6.SetWindowTextW(EncoderValue);
		break;
	default:
		break;
	}*/

	GetDlgItem(EDIT_ID + wParam)->SetWindowTextW(EncoderValue);

	return 0;
}



bool RoughCalibrationDlg::IsNumber(CString& EncoderValue)
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialogEx::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
	bool isNum = false;

	if (EncoderValue.SpanIncluding(_T("-0123456789")) == EncoderValue)
	{
		return true;
	}
	else
	{
		MessageBox(_T("请输入整数"), _T("错误"), MB_OK);
	}

	while (!isNum)
	{
		EncoderValue = EncoderValue.Left(EncoderValue.GetLength() - 1);
		if (EncoderValue.SpanIncluding(_T("-0123456789")) == EncoderValue)
		{
			isNum = true;
		}
	}

	return false;
}


void RoughCalibrationDlg::OnEnChangeEdit1()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialogEx::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here

	//EditChange(GetDlgItem(IDC_EDIT1));
	CString EncoderValue;
	m_EncoderValue1.GetWindowTextW(EncoderValue);

	bool isNum = IsNumber(EncoderValue);

	m_EncoderValue1.SetSel(EncoderValue.GetLength(),EncoderValue.GetLength(), false);
	if (isNum) return;

	m_EncoderValue1.SetWindowTextW(EncoderValue);

	
}


void RoughCalibrationDlg::OnEnChangeEdit2()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialogEx::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
	CString EncoderValue;
	m_EncoderValue2.GetWindowTextW(EncoderValue);

	bool isNum = IsNumber(EncoderValue);

	m_EncoderValue2.SetSel(EncoderValue.GetLength(), EncoderValue.GetLength(), false);
	if (isNum) return;

	m_EncoderValue2.SetWindowTextW(EncoderValue);
}


void RoughCalibrationDlg::OnEnChangeEdit3()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialogEx::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
	CString EncoderValue;
	m_EncoderValue3.GetWindowTextW(EncoderValue);

	bool isNum = IsNumber(EncoderValue);

	m_EncoderValue3.SetSel(EncoderValue.GetLength(), EncoderValue.GetLength(), false);
	if (isNum) return;

	m_EncoderValue3.SetWindowTextW(EncoderValue);
}


void RoughCalibrationDlg::OnEnChangeEdit4()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialogEx::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
	CString EncoderValue;
	m_EncoderValue4.GetWindowTextW(EncoderValue);

	bool isNum = IsNumber(EncoderValue);

	m_EncoderValue4.SetSel(EncoderValue.GetLength(), EncoderValue.GetLength(), false);
	if (isNum) return;

	m_EncoderValue4.SetWindowTextW(EncoderValue);
}


void RoughCalibrationDlg::OnEnChangeEdit5()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialogEx::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
	CString EncoderValue;
	m_EncoderValue5.GetWindowTextW(EncoderValue);

	bool isNum = IsNumber(EncoderValue);

	m_EncoderValue5.SetSel(EncoderValue.GetLength(), EncoderValue.GetLength(), false);
	if (isNum) return;

	m_EncoderValue5.SetWindowTextW(EncoderValue);
}


void RoughCalibrationDlg::OnEnChangeEdit6()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialogEx::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
	CString EncoderValue;
	m_EncoderValue6.GetWindowTextW(EncoderValue);

	bool isNum = IsNumber(EncoderValue);

	m_EncoderValue6.SetSel(EncoderValue.GetLength(), EncoderValue.GetLength(), false);
	if (isNum) return;

	m_EncoderValue6.SetWindowTextW(EncoderValue);
}


afx_msg void RoughCalibrationDlg::OnButtonClick(UINT uID)
{
	m_AxisIdx = uID - BUTTON_ID + 1;

	CString sSend;
	m_GetEncoder = true;
	sSend.Format(_T("%d"), m_AxisIdx);
	sSend = m_CMDGetEncoder + sSend;
	tTargetPacketID = QKMLinkSend(sSend);
	SetTargetPacketID(tTargetPacketID);
}


afx_msg void RoughCalibrationDlg::OnEnChangeEdit(UINT uID)
{
	CString EncoderValue;
	CString ToolTipString = _T("请输入整数");

	GetDlgItem(uID)->GetWindowTextW(EncoderValue);

	bool isNum = false;

	if (EncoderValue.SpanIncluding(_T("-0123456789")) == EncoderValue)
	{
		return;
	}
	else
	{
		//m_ToolTip.AddTool(GetDlgItem(uID), ToolTipString);
		//m_ToolTip.ShowWindow(true);
		MessageBox(_T("请输入整数"), _T("错误"), MB_OK);
		//GetDlgItem(uID)->EnableToolTips(true);
		//m_ToolTip.UpdateTipText(ToolTipString, GetDlgItem(uID));
	}

	while (!isNum)
	{
		EncoderValue = EncoderValue.Left(EncoderValue.GetLength() - 1);
		if (EncoderValue.SpanIncluding(_T("-0123456789")) == EncoderValue)
		{
			isNum = true;
		}
	}


	GetDlgItem(uID)->SetWindowTextW(EncoderValue);
	((CEdit*)GetDlgItem(uID))->SetSel(EncoderValue.GetLength(), EncoderValue.GetLength(), false);
}

BOOL RoughCalibrationDlg::PreTranslateMessage(MSG* pMsg)
{
	// TODO: Add your specialized code here and/or call the base class

	if (NULL != m_ToolTip.GetSafeHwnd())
	{
		m_ToolTip.RelayEvent(pMsg);
	}

	return CDialog::PreTranslateMessage(pMsg);
}