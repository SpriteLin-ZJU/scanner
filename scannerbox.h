#pragma once
#include "InterfaceLLT_2.h"
#include <QWidget>

class QLabel;
class CInterfaceLLT;
class QComboBox;
class QGroupBox;
class QPushButton;

extern void __stdcall NewProfile(const unsigned char* pucData, unsigned int uiSize, void* pUserData);
extern 	std::vector<unsigned char> vucProfileBuffer;

class ScannerBox : public QWidget
{
	Q_OBJECT
public:
	ScannerBox(QWidget *parent = Q_NULLPTR);
	~ScannerBox();
	
	void startProfileTrans();
	void stopProfileTrans();
signals:
	void updateStatus(QString& );
private:
	void ipSearch();
	void scanConnect();
	void advancedSettings();
	void writeScannerSettings();
	void changeProfileTrans();

	void OnError(QString errorText);
	void OnError(QString errorText, int errorValue);

	QGroupBox* m_scanGroupBox;
	QLabel* m_ipLabel;
	QComboBox* m_ipComboBox;
	QLabel* m_scanType;

	QPushButton* m_ipSearch;
	QPushButton* m_scanConnect;
	QPushButton* m_advancedSettings;
	QPushButton* m_transButton;
	
	//传感器接口
	CInterfaceLLT* m_scanner;
	unsigned int m_uiResolution = 0;
	TScannerType m_tscanCONTROLType;
	
	std::vector<unsigned int> m_vuiInterfaces;
	unsigned int m_uiInterfaceCount = 0;
	std::vector<DWORD> m_vdwResolutions;

	bool m_bLoadError;
	int m_iRetValue;

};