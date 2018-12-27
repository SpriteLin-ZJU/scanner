#pragma once
#include <QWidget>
//pcl包含文件必须在InterfaceLLT_2前(windows.h前)或者见onenote！！！
#include "InterfaceLLT_2.h"
#include "scandatamanager.h"

class QLabel;
class CInterfaceLLT;
class QComboBox;
class QGroupBox;
class QPushButton;
//class ScandataManager;

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

	void setScandataManager(ScandataManager* manager);
	void setScanFeedrate(int feedrate);
signals:
	void updateStatus(const QString& );
	void drawPointClouds();
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
	
	//manager
	ScandataManager* m_scandataManager;

	//传感器接口
	CInterfaceLLT* m_scanner;
	unsigned int m_uiResolution = 640;
	unsigned int m_uiShutterTime = 100; //1000us
	unsigned int m_uiIdleTime = 3900;
	unsigned int m_uiscanRate = 30;	//25 1/s
	unsigned int m_scanFeedrate = 145;
	TScannerType m_tscanCONTROLType;
	
	std::vector<unsigned int> m_vuiInterfaces;
	unsigned int m_uiInterfaceCount = 0;
	std::vector<DWORD> m_vdwResolutions;

	bool m_bLoadError;
	int m_iRetValue;

};