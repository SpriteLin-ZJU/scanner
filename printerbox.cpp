#include "printerbox.h"
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QLayout>
#include <QLineEdit>
#include <QGroupBox>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>

PrinterBox::PrinterBox(QWidget *parent) :
	QWidget(parent)
{
	m_printGroupBox = new QGroupBox(tr("Printer"), this);
	m_printGroupBox->setMaximumWidth(300);

	m_portLabel = new QLabel(tr("Port:"), this);
	m_portComboBox = new QComboBox(this);
	m_refreshButton = new QPushButton(tr("Refresh"), this);
	m_baudrateLabel = new QLabel(tr("Baudrate:"), this);
	m_baudComboBox = new QComboBox(this);
	m_printConnectButton = new QPushButton(tr("Connect"), this);

	m_printFileLabel = new QLabel(tr("File name:"), this);
	m_openFileButton = new QPushButton(tr("Open File"), this);
	m_manuCodeLabel = new QLabel(tr("Gcode:"), this);
	m_manuCodeEdit = new QLineEdit(this);
	m_sendCodeButton = new QPushButton(tr("Send"), this);
	m_sendCodeButton->setDisabled(true);
	m_printButton = new QPushButton(tr("Print"), this);
	m_printButton->setDisabled(true);

	//布局
	QGridLayout* printerBoxLayout = new QGridLayout;
	printerBoxLayout->addWidget(m_portLabel, 0, 0, 1, 1);
	printerBoxLayout->addWidget(m_portComboBox, 0, 1, 1, 1);
	printerBoxLayout->addWidget(m_refreshButton, 0, 2, 1, 1);
	printerBoxLayout->addWidget(m_baudrateLabel, 1, 0, 1, 1);
	printerBoxLayout->addWidget(m_baudComboBox, 1, 1, 1, 1);
	printerBoxLayout->addWidget(m_printConnectButton, 1, 2, 1, 1);
	printerBoxLayout->addWidget(m_printFileLabel, 2, 0, 1, 2);
	printerBoxLayout->addWidget(m_openFileButton, 2, 2, 1, 1);
	printerBoxLayout->addWidget(m_manuCodeLabel, 3, 0, 1, 1);
	printerBoxLayout->addWidget(m_manuCodeEdit, 3, 1, 1, 1);
	printerBoxLayout->addWidget(m_sendCodeButton, 3, 2, 1, 1);
	printerBoxLayout->addWidget(m_printButton, 4, 0, 1, 3);

	m_printGroupBox->setLayout(printerBoxLayout);
	QHBoxLayout* mainLayout = new QHBoxLayout;
	mainLayout->addWidget(m_printGroupBox);

	setLayout(mainLayout);

	m_serialPort = new QSerialPort(this);
	initPorts();

	//信号槽
	connect(m_refreshButton, &QPushButton::clicked, this, &PrinterBox::initPorts);
	connect(m_printConnectButton, &QPushButton::clicked, this, &PrinterBox::changeConnectState);
	connect(m_openFileButton, &QPushButton::clicked, this, &PrinterBox::openFile);
	connect(m_sendCodeButton, &QPushButton::clicked, this, &PrinterBox::sendManuGcode);
}


PrinterBox::~PrinterBox()
{
	
}

void PrinterBox::initPorts()
{
	//清空当前端口列表
	m_portComboBox->clear();
	//查询已连接可供使用的端口
	foreach(const QSerialPortInfo &portInfo, QSerialPortInfo::availablePorts())
	{
		QSerialPort serial;
		serial.setPort(portInfo);
		if (serial.open(QIODevice::ReadWrite)) {
			m_portComboBox->addItem(portInfo.portName());
			serial.close();
		}
	}

	//初始化波特率
	if (m_baudComboBox->currentIndex() == -1) {
		m_baudComboBox->clear();
		QStringList baudList;
		baudList << "4800" << "9600" << "14400" << "19200" << "115200"<<"250000";
		m_baudComboBox->addItems(baudList);
	}
}

void PrinterBox::connectPort()
{
	if (m_portComboBox->currentIndex() != -1) {
		m_serialPort->setPortName(m_portComboBox->currentText());
		if (m_serialPort->open(QIODevice::ReadWrite)) {
			m_serialPort->setBaudRate(m_baudComboBox->currentText().toInt());
			m_serialPort->setDataBits(QSerialPort::Data8);
			m_serialPort->setParity(QSerialPort::NoParity);
			m_serialPort->setFlowControl(QSerialPort::NoFlowControl);
			m_serialPort->setStopBits(QSerialPort::OneStop);

			m_printConnectButton->setText("Disconnect");
			m_sendCodeButton->setEnabled(true);
		}
	}
}

void PrinterBox::changeConnectState()
{
	if (m_printConnectButton->text() == "Connect")
		connectPort();
	else {
		m_serialPort->close();
		m_printConnectButton->setText("Connect");
		m_sendCodeButton->setDisabled(true);
	}
}

void PrinterBox::openFile()
{
	QString path = QFileDialog::getOpenFileName(this, tr("Open File"), ".", "*.gcode");
	if (!path.isEmpty()) {
		QFile file(path);
		if (!file.open(QIODevice::ReadOnly)) {
			QMessageBox::warning(this, tr("Read File"),
				tr("Cannot open file:\n%1").arg(path));
			return;
		}

		QTextStream textStream(&file);
		m_fileGcode.clear();
		while (!textStream.atEnd())
			m_fileGcode.append(textStream.readLine());

		QString s = "File name:" + path;
		m_printFileLabel->setText(s);
	}
	else {
		QMessageBox::warning(this, tr("Path"),
			tr("You did not select any file."));
	}
}

void PrinterBox::sendManuGcode()
{
	m_serialPort->write((m_manuCodeEdit->text()+"\r").toLatin1());
}

void PrinterBox::OnError(QString errorText)
{
	emit updateStatus(errorText);
}






