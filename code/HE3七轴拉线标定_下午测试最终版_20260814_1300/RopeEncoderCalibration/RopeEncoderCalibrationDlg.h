#pragma once
#include "afxwin.h"
#include <string>

#define WM_DISPLAY_CHANGE (WM_USER + 1)
#define WM_ROBOT_MOVE (WM_USER + 2)
#define WM_ENABLE_BTN (WM_USER + 3)
#define WM_SAVE_JOINT (WM_USER + 4)
#define WM_GET_ROBOT_JOINT (WM_USER + 5)
#define WM_TIME_OUT (WM_USER + 6)
#define WM_BUTTON_ENABLE (WM_USER + 7)
#define WM_WRITE_LOG (WM_USER + 8)
#define WM_REPLACE_XML (WM_USER + 11)
#define WM_WRITE_XML (WM_USER + 12)
#define WM_MOVE_ZERO (WM_USER + 13)
// RopeEncoderCalibrationDlg 对话框

class RoughCalibrationDlg;


class RopeEncoderCalibrationDlg : public CDialogEx
{
	DECLARE_DYNAMIC(RopeEncoderCalibrationDlg)

public:
	RopeEncoderCalibrationDlg(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~RopeEncoderCalibrationDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_DIALOG1 };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	std::string ReadIDNValue(CString IDNModule, CString IDNIndex, CString arrIndex);
	int WriteIDNValue(CString IDNModule, CString IDNIndex, CString value, CString arrIndex = _T(""));
	std::string GeneralWaitValue(CString sSend, int iWaitTime);
	int PowerOff();
	afx_msg
		BOOL OnInitDialog();
	
	static UINT KeepConnecting(LPVOID lParam);
	static UINT MyThreadFunction(LPVOID pParam);
	static UINT ZeroThread(LPVOID pParam);

	afx_msg LRESULT OnClosing(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT GetRobotWhereAngle(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnRobotMove(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnEnablebtn(WPARAM wParam, LPARAM lParam);
	afx_msg	LRESULT StartButtonEnable(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnDisplayChange(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnReplaceXmlValue(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT ReceiveLog(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT SaveRobotJoint(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT MessageBoxShow(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnMoveZero(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnWriteXml(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnEncoderValueShow(WPARAM wParam, LPARAM lParam);

	void WriteLog(long PacketID, char *logMsg);
	void SendInit();
	void MoveLocation(int target);
	void MoveZero(CString csValue);

	void OnEnChangeEdit1();

	int SendCmd(CString msg);

	bool SetTargetPacketID(int target);
	int DecodeRobotSystemVersion(CString tMessage);

	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedButton3();
	afx_msg void OnBnClickedButton4();
	afx_msg void OnBnClickedButton5();
	afx_msg void OnBnClickedButton6();
	afx_msg void OnBnClickedButton7();
	afx_msg void OnBnClickedButton8();
	afx_msg void OnBnClickedButton9();
	afx_msg void OnBnClickedButton10();
	afx_msg void OnBnClickedButton11();
	afx_msg void OnCbnSelchangeCombo1();

	CButton m_ConnectBtn;
	CButton m_StartBtn;
	CButton m_ContinueBtn;
	CButton m_PauseBtn;
	CButton m_StopBtn;
	CButton m_CalibBtn;
	CButton m_SaveBtn;
	CButton m_EncoderBtn;
	CButton m_CalibTypeBtn;
	CButton m_LocationZero;
	CButton m_DisconnectBtn;

	CIPAddressCtrl m_RbtIPAdr;
	CIPAddressCtrl m_CellIPAdr;

	CListCtrl m_List;

	CComboBox m_RbtTypeCombo;
	CComboBox m_CalibTypeCombo;
	CComboBox m_RbtSpeed;
	
	CEdit m_A1;
	CEdit m_A2;
	CEdit m_D1;
	CEdit m_D2;
	CEdit m_D3;
	CEdit m_D4;
	CEdit m_JointShow;
	CEdit m_EncoderHomeStr;
	CEdit m_ErrorBefore;
	CEdit m_ErrorAfter;
	RoughCalibrationDlg* m_ActiveRoughCalibrationDlg = nullptr;
	afx_msg void OnSelchangeCombo2();

	
};
