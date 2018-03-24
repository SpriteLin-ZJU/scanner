#include "scannerbox.h"
#include "settingsdialog.h"

#include <QLayout>
#include <QLabel>
#include <QGroupBox>
#include <QPushButton>
#include <QComboBox>
#include <QSettings>

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
	
	//����
	QHBoxLayout* hBoxLayout1 = new QHBoxLayout;
	hBoxLayout1->addWidget(m_ipLabel);
	hBoxLayout1->addWidget(m_ipComboBox);
	hBoxLayout1->addWidget(m_ipSearch);
	QHBoxLayout* hBoxLayout2 = new QHBoxLayout;
	hBoxLayout2->addWidget(m_scanType);
	hBoxLayout2->addStretch();
	hBoxLayout2->addWidget(m_scanConnect);
	QHBoxLayout* hBoxLayout3 = new QHBoxLayout;
	hBoxLayout3->addStretch();
	hBoxLayout3->addWidget(m_advancedSettings);
	QVBoxLayout* groupVLayout = new QVBoxLayout;
	groupVLayout->addLayout(hBoxLayout1);
	groupVLayout->addLayout(hBoxLayout2);
	groupVLayout->addLayout(hBoxLayout3);
	m_scanGroupBox->setLayout(groupVLayout);

	QHBoxLayout* mainLayout = new QHBoxLayout;
	mainLayout->addWidget(m_scanGroupBox);

	setLayout(mainLayout);
	

	//�����������ӿ�
	m_scanner = new CInterfaceLLT("LLT.dll", &m_bLoadError);
	if (!m_bLoadError) {
		m_iRetValue = m_scanner->CreateLLTDevice(INTF_TYPE_ETHERNET);
		if(m_iRetValue==GENERAL_FUNCTION_OK)
			m_ipSearch->setEnabled(true);
	}


	//�źŲ�����
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
	//�������������Ե�IP��ַ
	//���֮ǰ�������IP
	m_ipComboBox->clear();		//ע��ᷢ��currentIndexChanged(int)�ź�
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
	if (index < 0) return;			//ComBoxΪ�ռ���δѡ��IP��ַ

	//������ָ����IP��ַ
	m_iRetValue = m_scanner->SetDeviceInterface(m_vuiInterfaces[index], 0);
	m_iRetValue = m_scanner->Connect();

	if (m_iRetValue != GENERAL_FUNCTION_OK) {
		OnError("Error during connect", m_iRetValue);
		return;
	}

	//��ȡ�豸�ͺ�
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
		writeScannerSettings();
	}
}

void ScannerBox::writeScannerSettings()
{
	QSettings settings("ZJU", "scanner");
	//���÷ֱ���
	m_iRetValue = m_scanner->SetResolution(settings.value("resolution").toInt());
	if (m_iRetValue != GENERAL_FUNCTION_OK) {
		OnError("Error during SetFeature(FEATURE_FUNCTION_TRIGGER)", m_iRetValue);
		return;
	}
	//����triggerģʽ

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
