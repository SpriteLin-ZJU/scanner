#pragma once
#include "InterfaceLLT_2.h"
#include <QtGui>
#include <QWidget>

class QLabel;
class CInterfaceLLT;
class QComboBox;
class QGroupBox;
class QPushButton;

class ScannerBox : public QWidget
{
	Q_OBJECT
public:
	ScannerBox(QWidget *parent = Q_NULLPTR);

private:
	void ipSearch();
	void scanConnect();
	void advancedSettings();
	void OnError(QString errorText, int errorValue);

	
	QGroupBox* m_scanGroupBox;
	QLabel* m_ipLabel;
	QComboBox* m_ipComboBox;
	QLabel* m_scanType;

	QPushButton* m_ipSearch;
	QPushButton* m_scanConnect;
	QPushButton* m_advancedSettings;
	
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