#include "scannerbox.h"
#include "settingsdialog.h"

#include <QLayout>
#include <QLabel>
#include <QGroupBox>
#include <QPushButton>
#include <QComboBox>
#include <QSettings>

std::vector<unsigned char> vucProfileBuffer;
std::vector<double>vdValueX;
std::vector<double>vdValueZ;

QString convertIP(unsigned int i)
{
	QString s;
	s += QString::number((i >> 24) & 255,10);
	s += ".";
	s += QString::number((i >> 16) & 255, 10);
	s += ".";
	s += QString::number((i >> 8) & 255, 10);
	s += ".";
	s += QString::number(i & 255, 10);
	return s;
}

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
	m_transButton = new QPushButton(tr("Start"), this);
	m_transButton->setDisabled(true);

	//布局
	QHBoxLayout* hBoxLayout1 = new QHBoxLayout;
	hBoxLayout1->addWidget(m_ipLabel);
	hBoxLayout1->addWidget(m_ipComboBox,1);
	hBoxLayout1->addWidget(m_ipSearch);
	QHBoxLayout* hBoxLayout2 = new QHBoxLayout;
	hBoxLayout2->addWidget(m_scanType);
	hBoxLayout2->addStretch();
	hBoxLayout2->addWidget(m_scanConnect);
	QHBoxLayout* hBoxLayout3 = new QHBoxLayout;
	hBoxLayout3->addWidget(m_transButton);
	hBoxLayout3->addWidget(m_advancedSettings);
	QVBoxLayout* groupVLayout = new QVBoxLayout;
	groupVLayout->addLayout(hBoxLayout1);
	groupVLayout->addLayout(hBoxLayout2);
	groupVLayout->addLayout(hBoxLayout3);
	m_scanGroupBox->setLayout(groupVLayout);

	QHBoxLayout* mainLayout = new QHBoxLayout;
	mainLayout->addWidget(m_scanGroupBox);

	setLayout(mainLayout);
	

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
	connect(m_transButton, &QPushButton::clicked, this, &ScannerBox::changeProfileTrans);
}

ScannerBox::~ScannerBox()
{
	m_scanner->Disconnect();
	delete m_scanner;
}

void ScannerBox::ipSearch()
{
	emit updateGraph(m_uiResolution);

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

	OnError("Search OK");
	//转换为IP地址
	for (int i = 0; i < m_uiInterfaceCount; i++) {
		QString s = convertIP(m_vuiInterfaces[i]);
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

	if (m_iRetValue < GENERAL_FUNCTION_OK) {
		OnError("Error during connect", m_iRetValue);
		return;
	}
	else
		OnError("Connected");

	//获取设备型号
	m_iRetValue = m_scanner->GetLLTType(&m_tscanCONTROLType);
	if (m_iRetValue < GENERAL_FUNCTION_OK)
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

	//写入传感器设置
	writeScannerSettings();
	m_advancedSettings->setEnabled(true);
	m_transButton->setEnabled(true);
}

void ScannerBox::advancedSettings()
{
	SettingsDialog settingDialog(this);
	if (settingDialog.exec()) {
		settingDialog.writeSettings();
		writeScannerSettings();
	}
}

void ScannerBox::writeScannerSettings()
{
	QSettings settings("ZJU", "scanner");
	//设置分辨率
	m_iRetValue = m_scanner->SetResolution(settings.value("resolution").toUInt());
	if (m_iRetValue < GENERAL_FUNCTION_OK) {
		OnError("Error during SetFeature(FEATURE_FUNCTION_TRIGGER)", m_iRetValue);
		return;
	}
	m_uiResolution = settings.value("resolution").toUInt();

	//设置trigger模式
	m_iRetValue = m_scanner->SetFeature(FEATURE_FUNCTION_TRIGGER, 0x00000000);
	if (m_iRetValue < GENERAL_FUNCTION_OK) {
		OnError("Error duiring SetFeature(FEATURE_FUNCTION_TRIGGER)", m_iRetValue);
		return;
	}
	//设置Profile格式
	m_iRetValue = m_scanner->SetProfileConfig(PROFILE);
	if (m_iRetValue < GENERAL_FUNCTION_OK) {
		OnError("Error during SetProfileConfig", m_iRetValue);
		return;
	}
	//设置shutter time
	m_iRetValue = m_scanner->SetFeature(FEATURE_FUNCTION_SHUTTERTIME, 100U);
	if (m_iRetValue < GENERAL_FUNCTION_OK) {
		OnError("Error during SetFeature(FEATURE_FUNCTION_SHUTTERTIME)", m_iRetValue);
		return;
	}
	//设置idle time
	m_iRetValue = m_scanner->SetFeature(FEATURE_FUNCTION_IDLETIME, 900U);
	if (m_iRetValue < GENERAL_FUNCTION_OK) {
		OnError("Error during SetFeature(FEATURE_FUNCTION_IDLETIME)", m_iRetValue);
		return;
	}
	//登记回调函数
	m_iRetValue = m_scanner->RegisterCallback(STD_CALL, (void*)NewProfile, 0);
	if (m_iRetValue < GENERAL_FUNCTION_OK) {
		OnError("Error during RegisterCallback", m_iRetValue);
		return;
	}
}

//改变传输状态
void ScannerBox::changeProfileTrans()
{
	if (m_transButton->text() == "Start")
		startProfileTrans();
	else if (m_transButton->text() == "Stop")
		stopProfileTrans();
}


void ScannerBox::startProfileTrans()
{
	//首先清空之前数据
	vucProfileBuffer.clear();

	m_iRetValue = m_scanner->TransferProfiles(NORMAL_TRANSFER, true);
	if (m_iRetValue < GENERAL_FUNCTION_OK) {
		OnError("Error during TransferProfiles", m_iRetValue);
		return;
	}
	m_transButton->setText(tr("Stop"));
	Sleep(100);		//warm up
}

void ScannerBox::stopProfileTrans()
{
	m_iRetValue = m_scanner->TransferProfiles(NORMAL_TRANSFER, false);
	if (m_iRetValue < GENERAL_FUNCTION_OK) {
		OnError("Error during TransferProfiles", m_iRetValue);
		return;
	}

	m_transButton->setText(tr("Start"));

	//处理接收到的数据,分配向量所需大小
	vdValueX.clear();
	vdValueX.resize(vucProfileBuffer.size() / 64);
	vdValueZ.clear();
	vdValueZ.resize(vucProfileBuffer.size() / 64);

	for (int i = 0; i < vucProfileBuffer.size() / (64*m_uiResolution); i++) {
		
		m_iRetValue = m_scanner->ConvertProfile2Values(&vucProfileBuffer[i*m_uiResolution*64], m_uiResolution, PROFILE, m_tscanCONTROLType, 0, true, NULL,
			NULL, NULL, &vdValueX[i*m_uiResolution], &vdValueZ[i*m_uiResolution], NULL, NULL);
	}
	
	if (((m_iRetValue & CONVERT_X) == 0) || ((m_iRetValue & CONVERT_Z) == 0))
	{
		OnError("Error during Converting of profile data", m_iRetValue);
		return;
	}
	//更新图像
	emit updateGraph(m_uiResolution);
}


void ScannerBox::OnError(QString errorText, int errorValue)
{
	char acErrorString[200];
	if (m_scanner->TranslateErrorValue(errorValue, acErrorString, sizeof(acErrorString)) >= GENERAL_FUNCTION_OK) {
		QString s = errorText + ":" + acErrorString;
		emit updateStatus(s);
		return;
	}
}

void ScannerBox::OnError(QString errorText)
{
	emit updateStatus(errorText);
}

//回调函数
void __stdcall NewProfile(const unsigned char* pucData, unsigned int uiSize, void* pUserData)
{
	vucProfileBuffer.insert(vucProfileBuffer.end(), pucData, pucData + uiSize);	
}
