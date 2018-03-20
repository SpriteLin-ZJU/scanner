#include "scannerbox.h"
#include "InterfaceLLT_2.h"
#include "settingsdialog.h"

#include <QLayout>
#include <QLabel>
#include <QGroupBox>
#include <QPushButton>
#include <QComboBox>

ScannerBox::ScannerBox(QWidget *parent) :
	QWidget(parent)
{
	m_ipLabel = new QLabel(tr("IP:"),this);
	m_scanType = new QLabel(tr("Type:"),this);
	m_scanType->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);

	m_ipComboBox = new QComboBox(this);
	m_scanGroupBox = new QGroupBox(tr("Scanner"), this);
	m_ipSearch = new QPushButton(tr("&Search"),this);
	m_ipSearch->setDisabled(true);
	m_scanConnect = new QPushButton(tr("Connect"),this);
	m_advancedSettings = new QPushButton(tr("Advanced"),this);
	m_advancedSettings->setDisabled(false);
	
	//布局
	QVBoxLayout *typeLayout = new QVBoxLayout;
	typeLayout->addWidget(m_scanType);
	typeLayout->addStretch();

	QGridLayout *leftLayout = new QGridLayout;
	leftLayout->addWidget(m_ipLabel, 0, 0);
	leftLayout->addWidget(m_ipComboBox, 0, 1);
	leftLayout->addLayout(typeLayout, 1, 0,1,2);
	
	QVBoxLayout *rightLayout = new QVBoxLayout;
	rightLayout->addWidget(m_ipSearch);
	rightLayout->addWidget(m_scanConnect);
	rightLayout->addWidget(m_advancedSettings);

	QHBoxLayout *mainLayout = new QHBoxLayout;
	mainLayout->addLayout(leftLayout);
	mainLayout->addLayout(rightLayout);
	
	m_scanGroupBox->setLayout(mainLayout);

	//创建传感器接口
	m_scanner = new CInterfaceLLT("LLT.dll", &m_bLoadError);
	if (!m_bLoadError) {
		m_iRetValue = m_scanner->CreateLLTDevice(INTF_TYPE_ETHERNET);
		if(m_iRetValue==GENERAL_FUNCTION_OK)
			m_ipSearch->setEnabled(true);
	}


	//信号槽连接
	connect(m_ipSearch, &QPushButton::clicked, this, &ScannerBox::ipSearch);
	connect(m_scanConnect, &QPushButton::clicked, this, &ScannerBox::scanConnect);
	connect(m_advancedSettings, &QPushButton::clicked, this, &ScannerBox::advancedSettings);
}

void ScannerBox::ipSearch()
{
	//搜索连接至电脑的IP地址

	m_vuiInterfaces.resize(5);
	m_iRetValue = m_scanner->search(m_vuiInterfaces);
	
	if (iRetValue < 0) {
		OnError("Error during SetDeviceInterface", iRetValue);
		return;
	}
	else
		m_uiInterfaceCount = iRetValue;

	for (int i = 0; i < m_uiInterfaceCount; i++) {
		QString s = QString::number(m_vuiInterfaces[i], 16).toUpper();
		m_ipComboBox->addItem(s);
	}
}

void ScannerBox::scanConnect()
{
	int index = m_ipComboBox->currentIndex();
	if (index < 0) return;			//ComBox为空集

	iRetValue = m_scanner->connect(m_vuiInterfaces[index],m_sScanType);
	if (iRetValue < GENERAL_FUNCTION_OK) {
		OnError("Error during connect", iRetValue);
		return;
	}
	QString s = "Type:" + QString::fromStdString(m_sScanType);
	m_scanType->setText(s);
	m_advancedSettings->setEnabled(true);
}

void ScannerBox::advancedSettings()
{
	SettingsDialog settingDialog(this);
	if (settingDialog.exec()) {

	}
}

void ScannerBox::OnError(QString errorText, int errorValue)
{

}
