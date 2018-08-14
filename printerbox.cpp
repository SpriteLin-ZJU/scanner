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
#include <QRegexp>
#include <QTimer>
#include <QDeBug>
#include <QSettings>

#include "gcodemanager.h"
#include "stlmanager.h"

PrinterBox::PrinterBox(QWidget *parent) :
	QWidget(parent), 
	m_gcodeManager(NULL),
	m_stlManager(NULL)
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

	//����
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

	//�źŲ�
	connect(m_refreshButton, &QPushButton::clicked, this, &PrinterBox::initPorts);
	connect(m_printConnectButton, &QPushButton::clicked, this, &PrinterBox::changeConnectState);
	connect(m_openFileButton, &QPushButton::clicked, this, &PrinterBox::openFile);
	connect(m_sendCodeButton, &QPushButton::clicked, this, &PrinterBox::sendManuGcode);
	connect(m_printButton, &QPushButton::clicked, this, &PrinterBox::printGcode);
	connect(m_serialPort, &QSerialPort::readyRead, this, &PrinterBox::onSerialReadyRead);
}


PrinterBox::~PrinterBox()
{
	m_gcodeManager = NULL;
	m_stlManager = NULL;
}

void PrinterBox::emergencyStop()
{
	m_serialPort->write(QString("M112\r").toLatin1());
	m_serialPort->flush();
	isPrinting = false;
}

void PrinterBox::stopPrinting()
{
	m_gcodeManager->clear();
	m_serialPort->flush();
	isPrinting = false;
}

void PrinterBox::initPorts()
{
	//��յ�ǰ�˿��б�
	m_portComboBox->clear();
	//��ѯ�����ӿɹ�ʹ�õĶ˿�
	foreach(const QSerialPortInfo &portInfo, QSerialPortInfo::availablePorts())
	{
		QSerialPort serial;
		serial.setPort(portInfo);
		if (serial.open(QIODevice::ReadWrite)) {
			m_portComboBox->addItem(portInfo.portName());
			serial.close();
		}
	}

	//��ʼ��������
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
		m_printButton->setDisabled(true);
	}
}

void PrinterBox::openFile()
{
	QSettings settings("ZJU", "scanner");
	QString lastPath = settings.value("lastFilePath").toString();

	QString path = QFileDialog::getOpenFileName(this, tr("Open File"), lastPath, "*.gcode;;*.stl");
	if (!path.isEmpty()) {
		
		//保存上次打开目录
		int i = path.lastIndexOf('/');
		QString rootPath = path.left(i);
		settings.setValue("lastFilePath", rootPath);

		//打开文件
		QFile file(path);
		if (!file.open(QIODevice::ReadOnly)) {
			QMessageBox::warning(this, tr("Read File"),
				tr("Cannot open file:\n%1").arg(path));
			return;
		}
		//清空之前stl及gcode
		m_stlManager->clear();
		m_gcodeManager->clear();
		//判断文件格式
		//如果是STL文件
		if (QFileInfo(file).suffix() == "STL") {
			QTextStream textStream(&file);
			while (!textStream.atEnd())
				m_stlManager->addLine(textStream.readLine());
			m_stlManager->fileToPoint();
			emit drawSTLFile();
		}
		//如果打开的为Gcode文件
		else if (QFileInfo(file).suffix() == "gcode") {
			QTextStream textStream(&file);
			while (!textStream.atEnd())
				m_gcodeManager->addCommand(textStream.readLine());
			if (m_printConnectButton->text() == "Disconnect")
				m_printButton->setEnabled(true);
		}

		QString s = "File name:" + path;
		m_printFileLabel->setText(s);
		
	}
	else {
		QMessageBox::warning(this, tr("Path"),
			tr("You did not select any file."));
	}
}

void PrinterBox::setGcodeManager(GcodeManager * manager)
{
	m_gcodeManager = manager;
}

void PrinterBox::setSTLManager(STLManager * manager)
{
	m_stlManager = manager;
}

void PrinterBox::sendManuGcode()
{
	m_serialPort->write((m_manuCodeEdit->text()+"\r").toLatin1());
	m_manuCodeEdit->clear();
}

void PrinterBox::printGcode()
{
	QRegExp rx("^[GMT]\\d{1,3}\\s*([XYZTSFE]\\s*-?\\d+\\.?\\d*\\s*){0,4}");
	rx.setMinimal(false);

	isPrinting = true;

	//先发送三条指令
	if (m_printButton->isEnabled()) {
		m_serialPort->flush();
		for (int i = 0; i < qMin(2, m_gcodeManager->fileSize()); i++) {
			int pos = 0;
			if ((pos = rx.indexIn(m_gcodeManager->takeFirstGcode(), pos)) != -1) {
				m_serialPort->write((rx.cap(0) + "\r").toLatin1());
				qDebug() << rx.cap(0);
				//并存入drawerbuffer
				m_gcodeManager->addDrawerBuffer(rx.cap(0));
			}
			else {
				i--;
			}
		}
		m_printButton->setDisabled(true);
	}
	
	if (!m_gcodeManager->fileIsEmpty()) {
		//send gcode
		int pos = 0;
		if ((pos = rx.indexIn(m_gcodeManager->takeFirstGcode(), pos)) != -1) {
			m_serialPort->write((rx.cap(0) + "\r").toLatin1());
			qDebug() << rx.cap(0);
			//add sended gcode to drawerbuffer
			m_gcodeManager->addDrawerBuffer(rx.cap(0));
		}
		else {
			printGcode();
		}
	}
	else
		isPrinting = false;
}

void PrinterBox::onSerialReadyRead()
{
	while (m_serialPort->canReadLine()) {
		QString data = m_serialPort->readLine().trimmed();
		qDebug() << data;
		if (data == "ok"&&isPrinting) {
			printGcode();
			emit drawSingleGcode();
		}
		continue;
		//begin scan, set feedrate
		//e.g:"feedrate:1500"
		QRegExp rx("feedrate:(\\d+)");
		int pos = 0;
		if ((pos = rx.indexIn(data,pos)) != -1) {
			emit setScanFeedrate(rx.cap(1).toInt());
		}
		else if (data == "scanstart") {
			emit startProfileTrans();
		}

		else if (data == "scanstop") {
			emit stopProfileTrans();
		}

	}
}


void PrinterBox::OnError(QString errorText)
{
	emit updateStatus(errorText);
}





