#include "printerbox.h"
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QLayout>
#include <QLineEdit>
#include <QGroupBox>

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
	m_printButton = new QPushButton(tr("Print"), this);
	m_printButton->setDisabled(true);

	//²¼¾Ö
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

}



PrinterBox::~PrinterBox()
{
}
