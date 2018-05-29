#pragma once
#include <QWidget>

class QGroupBox;
class QLabel;
class QComboBox;
class QPushButton;
class QLineEdit;
class QSerialPort;

class PrinterBox : public QWidget
{
	Q_OBJECT
public:
	PrinterBox(QWidget *parent = Q_NULLPTR);
	~PrinterBox();
signals:
	void updateStatus(QString&);
private:
	const int BUFFER_SIZE = 127;
	QList<QString> m_fileGcode;

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
	
	void initPorts();
	void connectPort();
	void changeConnectState();
	void openFile();
	void sendManuGcode();
	void OnError(QString errorText);
};