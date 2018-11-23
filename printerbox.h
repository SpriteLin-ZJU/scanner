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
class ScandataManager;

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
	void setScandataManager(ScandataManager* manager);
	void emitSliceSignal() { emit sliceSignal(); }
signals:
	void updateStatus(QString&);
	void drawSingleGcode();
	void drawSTLFile();
	void sliceSignal();
	void setScanFeedrate(int);
	void startProfileTrans();
	void stopProfileTrans();
	void updateColor();
	void openPcdFile(QString);
private:
	const int BUFFER_SIZE = 127;
	//iterator
	QList<QString>::iterator m_gcodeIt;
	//flag
	bool isPrinting = false;

	GcodeManager* m_gcodeManager;
	STLManager* m_stlManager;
	ScandataManager* m_scandataManager;

	QGroupBox* m_printGroupBox;
	//�˿�����
	QLabel* m_portLabel;
	QLabel* m_baudrateLabel;
	QComboBox* m_portComboBox;
	QComboBox* m_baudComboBox;
	QPushButton* m_refreshButton;
	QPushButton* m_printConnectButton;
	QSerialPort* m_serialPort;

	//��ӡ����
	QLabel* m_printFileLabel;
	QPushButton* m_openFileButton;
	QLabel* m_manuCodeLabel;
	QLineEdit* m_manuCodeEdit;
	QPushButton* m_sendCodeButton;
	QPushButton* m_printButton;
	QPushButton* m_sliceButton;
	QPushButton* m_settingsButton;
	
	void initPorts();
	void connectPort();
	void changeConnectState();
	void sendManuGcode();
	void printGcode();
	void printSettings();

	void onSerialReadyRead();
	void OnError(QString errorText);
};