#include "scannerbox.h"
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

ScannerBox::~ScannerBox()
{
	delete m_scanner;
}

void ScannerBox::ipSearch()
{
	//搜索连接至电脑的IP地址
	//清空之前所储存的IP
	m_ipComboBox->clear();		//注意会发送currentIndexChanged(int)信号
	std::vector<unsigned int>().swap(m_vuiInterfaces);
	m_vuiInterfaces.resize(5);

	m_iRetValue = m_scanner->GetDeviceInterfacesFast(m_vuiInterfaces.data(), (unsigned int)m_vuiInterfaces.size());
		
	if (m_iRetValue < 0) {
		OnError("Error during Searching DeviceInterface", m_iRetValue);
		return;
	}
	else if (m_iRetValue > 5)
		m_uiInterfaceCount = m_vuiInterfaces.size();
	else
		m_uiInterfaceCount = m_iRetValue;

	for (int i = 0; i < m_uiInterfaceCount; i++) {
		QString s = QString::number(m_vuiInterfaces[i], 16).toUpper();
		m_ipComboBox->addItem(s);
	}
}

void ScannerBox::scanConnect()
{
	int index = m_ipComboBox->currentIndex();
	if (index < 0) return;			//ComBox为空集或未选择IP地址

	//连接至指定的IP地址
	m_iRetValue = m_scanner->SetDeviceInterface(m_vuiInterfaces[index], 0);
	m_iRetValue = m_scanner->Connect();

	if (m_iRetValue != GENERAL_FUNCTION_OK) {
		OnError("Error during connect", m_iRetValue);
		return;
	}

	//获取设备型号
	m_iRetValue = m_scanner->GetLLTType(&m_tscanCONTROLType);
	if (m_iRetValue!=GENERAL_FUNCTION_OK)
	{
		OnError("Error during GetLLTType", m_iRetValue);
		return;
	}
	QString scanCRONTROLType;
	if (m_tscanCONTROLType >= scanCONTROL28xx_25 && m_tscanCONTROLType <= scanCONTROL28xx_xxx)
		scanCRONTROLType = "scanCONTROL28xx";
	else if (m_tscanCONTROLType >= scanCONTROL27xx_25 && m_tscanCONTROLType <= scanCONTROL27xx_xxx)
		scanCRONTROLType = "scanCONTROL27xx";
	else if (m_tscanCONTROLType >= scanCONTROL26xx_25 && m_tscanCONTROLType <= scanCONTROL26xx_xxx)
		scanCRONTROLType = "scanCONTROL26xx";
	else if (m_tscanCONTROLType >= scanCONTROL29xx_25 && m_tscanCONTROLType <= scanCONTROL29xx_xxx)
		scanCRONTROLType = "scanCONTROL29xx";
	else
		scanCRONTROLType = "undefined";
	QString s = "Type:	" + scanCRONTROLType;
	
	m_scanType->setText(s);
	m_advancedSettings->setEnabled(true);
}

void ScannerBox::advancedSettings()
{
	SettingsDialog settingDialog(this);
	if (settingDialog.exec()) {
		settingDialog.writeSettings();
	}
}

void ScannerBox::OnError(QString errorText, int errorValue)
{

}
