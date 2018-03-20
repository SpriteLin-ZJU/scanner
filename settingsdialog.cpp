#include "settingsdialog.h"
#include <QLabel>
#include <QLineEdit>
#include <QGroupBox>
#include <QLayout>
#include <QCheckBox>
#include <QComboBox>

SettingsDialog::SettingsDialog(QWidget* parent)
	: QDialog(parent)
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
	QVBoxLayout* profileGroupVLayout = new QVBoxLayout;
	profileGroupVLayout->addLayout(profileGroupHLayout1);
	profileGroupVLayout->addLayout(profileGroupHLayout2);
	m_profileGroupBox->setLayout(profileGroupVLayout);

	QVBoxLayout* layout1 = new QVBoxLayout;
	layout1->addWidget(m_shutterGroupBox);
	layout1->addWidget(m_profileGroupBox);
	setLayout(layout1);
}

void SettingsDialog::readSettings()
{
}
