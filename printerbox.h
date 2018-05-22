#pragma once
#include <QWidget>

class QGroupBox;
class QLabel;
class QComboBox;
class QPushButton;
class QLineEdit;

class PrinterBox : public QWidget
{
	Q_OBJECT
public:
	PrinterBox(QWidget *parent = Q_NULLPTR);
	~PrinterBox();
private:
	QGroupBox* m_printGroupBox;
	
	//�˿�����
	QLabel* m_portLabel;
	QLabel* m_baudrateLabel;
	QComboBox* m_portComboBox;
	QComboBox* m_baudComboBox;
	QPushButton* m_refreshButton;
	QPushButton* m_printConnectButton;

	//��ӡ����
	QLabel* m_printFileLabel;
	QPushButton* m_openFileButton;
	QLabel* m_manuCodeLabel;
	QLineEdit* m_manuCodeEdit;
	QPushButton* m_sendCodeButton;
	QPushButton* m_printButton;
	
};