#pragma once
#include "afxwin.h"
#define WM_GET_ENCODER (WM_USER + 9)
#define WM_SHOW_ENCODER (WM_USER + 10)

// RoughCalibrationDlg 对话框

class RoughCalibrationDlg : public CDialogEx
{
	DECLARE_DYNAMIC(RoughCalibrationDlg)
	CString m_IDNAxis1 = _T("P-0-0513.0.168");
	CString m_IDNAxis2 = _T("P-0-0513.0.169");
	CString m_IDNAxis3 = _T("P-0-0513.0.170");
	CString m_IDNAxis4 = _T("P-0-0513.0.171");
	CString m_IDNAxis5 = _T("P-0-0513.0.172");
	CString m_IDNAxis6 = _T("P-0-0513.0.173");
public:
	RoughCalibrationDlg(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~RoughCalibrationDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_DIALOG2 };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL OnInitDialog();
	bool SetTargetPacketID(int target);
	CEdit m_EncoderValue1;
	CEdit m_EncoderValue2;
	CEdit m_EncoderValue3;
	CEdit m_EncoderValue4;
	CEdit m_EncoderValue5;
	CEdit m_EncoderValue6;
	CButton m_EncoderBtn1;
	CButton m_EncoderBtn2;
	CButton m_EncoderBtn3;
	CButton m_EncoderBtn4;
	CButton m_EncoderBtn5;
	CButton m_EncoderBtn6;
	afx_msg void OnBnClickedButton7();
	afx_msg void OnBnClickedButton8();
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedButton3();
	afx_msg void OnBnClickedButton4();
	afx_msg void OnBnClickedButton5();
	afx_msg void OnBnClickedButton6();
	LRESULT OnEncoderValueShow(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT RoughCalibrationDlg::ShowValue(WPARAM wParam, LPARAM lParam);
	afx_msg
		bool IsNumber(CString& EncoderValue1);
	void OnEnChangeEdit1();
	afx_msg void OnEnChangeEdit2();
	afx_msg void OnEnChangeEdit3();
	afx_msg void OnEnChangeEdit4();
	afx_msg void OnEnChangeEdit5();
	afx_msg void OnEnChangeEdit6();
	afx_msg void OnButtonClick(UINT uID);
	afx_msg void OnEnChangeEdit(UINT uID);
	BOOL PreTranslateMessage(MSG * pMsg);
};
