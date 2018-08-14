#pragma once
#include <QWidget>

class QGroupBox;
class QLabel;
class QComboBox;
class QPushButton;
class QLineEdit;
class QSerialPort;
class QTimer;
class GcodeManager;
class STLManager;

class PrinterBox : public QWidget
{
	Q_OBJECT
public:
	PrinterBox(QWidget *parent = Q_NULLPTR);
	~PrinterBox();

	void emergencyStop();
	void stopPrinting();
	void openFile();
	void setGcodeManager(GcodeManager* manager);
	void setSTLManager(STLManager* manager);
signals:
	void updateStatus(QString&);
	void drawSingleGcode();
	void drawSTLFile();
	void setScanFeedrate(int);
	void startProfileTrans();
	void stopProfileTrans();
private:
	const int BUFFER_SIZE = 127;
	//iterator
	QList<QString>::iterator m_gcodeIt;
	//flag
	bool isPrinting = false;

	GcodeManager* m_gcodeManager;
	STLManager* m_stlManager;

	QGroupBox* m_printGroupBox;
	//端口设置
	QLabel* m_portLabel;
	QLabel* m_baudrateLabel;
	QComboBox* m_portComboBox;
	QComboBox* m_baudComboBox;
	QPushButton* m_refreshButton;
	QPushButton* m_printConnectButton;
	QSerialPort* m_serialPort;

	//打印设置
	QLabel* m_printFileLabel;
	QPushButton* m_openFileButton;
	QLabel* m_manuCodeLabel;
	QLineEdit* m_manuCodeEdit;
	QPushButton* m_sendCodeButton;
	QPushButton* m_printButton;
	
	void initPorts();
	void connectPort();
	void changeConnectState();
	void sendManuGcode();
	void printGcode();

	void onSerialReadyRead();
	void OnError(QString errorText);
};