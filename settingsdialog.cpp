#include "settingsdialog.h"
#include <QLabel>
#include <QLineEdit>
#include <QGroupBox>
#include <QLayout>
#include <QCheckBox>
#include <QComboBox>
#include <QTabWidget>
#include <QWidget>
SettingsDialog::SettingsDialog(QWidget* parent)
	: QDialog(parent)
{
	m_settingTabWidget = new QTabWidget(this);
	m_settingTabWidget->addTab(createGeneralTabWidget(), tr("General"));
	
	QVBoxLayout* mainLayout = new QVBoxLayout;
	mainLayout->addWidget(m_settingTabWidget);
	setLayout(mainLayout);
}

void SettingsDialog::readSettings()
{
}

QWidget * SettingsDialog::createGeneralTabWidget()
{
	//创建快门时间设置框
	m_shutterTimeLabel = new QLabel(tr("Shutter time:"), this);
	m_shutterTimeLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	m_idleTimeLabel = new QLabel(tr("Idle time:"), this);
	m_idleTimeLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	m_shutterTime = new QLineEdit(this);
	m_idleTime = new QLineEdit(this);
	m_shutterGroupBox = new QGroupBox(tr("ShutterTime"), this);
	//创建快门时间设置框布局
	QHBoxLayout *shutterGroupLayout = new QHBoxLayout;
	shutterGroupLayout->addWidget(m_shutterTimeLabel);
	shutterGroupLayout->addWidget(m_shutterTime);
	shutterGroupLayout->addWidget(m_idleTimeLabel);
	shutterGroupLayout->addWidget(m_idleTime);
	m_shutterGroupBox->setLayout(shutterGroupLayout);

	//创建Profile测量设置框
	m_measuringFieldLabel = new QLabel(tr("Measuring field:"), this);
	m_measuringFieldComBox = new QComboBox(this);
	QStringList list;
	list << "small" << "standard" << "large" << "huge";
	m_measuringFieldComBox->addItems(list);
	m_resolutionLabel = new QLabel(tr("Resolution:"), this);
	m_resolutionComBox = new QComboBox(this);
	list.clear();
	list << "80" << "160" << "320" << "640";
	m_resolutionComBox->addItems(list);
	m_invertX = new QCheckBox(tr("Invert X"), this);
	m_invertZ = new QCheckBox(tr("Invert Z"), this);
	m_profileGroupBox = new QGroupBox(tr("Profile"), this);
	//创建Profile测量设置框布局
	QHBoxLayout* profileGroupHLayout1 = new QHBoxLayout;
	profileGroupHLayout1->addWidget(m_measuringFieldLabel);
	profileGroupHLayout1->addWidget(m_measuringFieldComBox);
	profileGroupHLayout1->addWidget(m_resolutionLabel);
	profileGroupHLayout1->addWidget(m_resolutionComBox);
	QHBoxLayout* profileGroupHLayout2 = new QHBoxLayout;
	profileGroupHLayout2->addWidget(m_invertX);
	profileGroupHLayout2->addStretch();
	profileGroupHLayout2->addWidget(m_invertZ);
	profileGroupHLayout2->addStretch();
	QVBoxLayout* profileGroupVLayout = new QVBoxLayout;
	profileGroupVLayout->addLayout(profileGroupHLayout1);
	profileGroupVLayout->addLayout(profileGroupHLayout2);
	m_profileGroupBox->setLayout(profileGroupVLayout);

	//创建Sensor设置框
	m_sensorGroupBox = new QGroupBox(tr("Sensor"), this);
	m_thresholdLabel = new QLabel(tr("Threshold"), this);
	m_thresholdCombBox = new QComboBox(this);
	m_thresholdCombBox->addItem(tr("absolute"));
	m_thresholdCombBox->addItem(tr("dynamic"));
	m_thresholdLineEdit = new QLineEdit(this);
	m_reflectionsLabel = new QLabel(tr("Reflections"), this);
	m_reflectionsCombBox = new QComboBox(this);
	list.clear();
	list << "first" << "last" << "largest area" << "highest intensity" << "only single";
	m_reflectionsCombBox->addItems(list);
	m_laserPowerLabel = new QLabel(tr("Laser power"), this);
	m_laserPowerCombBox = new QComboBox(this);
	list.clear();
	list << "off" << "reduced" << "standard" << "reduced(pulsed)" << "standard(pulsed)";
	m_laserPowerCombBox->addItems(list);
	//设置Sensor设置框布局
	QHBoxLayout* thresholdHlayout = new QHBoxLayout;
	thresholdHlayout->addWidget(m_thresholdCombBox);
	thresholdHlayout->addWidget(m_thresholdLineEdit);
	QGridLayout* sensorGridLayout = new QGridLayout;
	sensorGridLayout->addWidget(m_thresholdLabel, 0, 0, 1, 1);
	sensorGridLayout->addLayout(thresholdHlayout, 0, 1);
	sensorGridLayout->addWidget(m_reflectionsLabel, 0, 2);
	sensorGridLayout->addWidget(m_reflectionsCombBox, 0, 3);
	sensorGridLayout->addWidget(m_laserPowerLabel, 1, 0);
	sensorGridLayout->addWidget(m_laserPowerCombBox, 1, 1, 1, 1);
	m_sensorGroupBox->setLayout(sensorGridLayout);
	//创建General选项卡布局
	QVBoxLayout* layoutGeneralTabWidget = new QVBoxLayout;
	layoutGeneralTabWidget->addWidget(m_shutterGroupBox);
	layoutGeneralTabWidget->addWidget(m_profileGroupBox);
	layoutGeneralTabWidget->addWidget(m_sensorGroupBox);

	QWidget* generalTab = new QWidget(this);
	generalTab->setLayout(layoutGeneralTabWidget);
	
	return generalTab;
}
