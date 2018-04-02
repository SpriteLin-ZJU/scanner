#include "settingsdialog.h"
#include <QLabel>
#include <QLineEdit>
#include <QGroupBox>
#include <QLayout>
#include <QCheckBox>
#include <QComboBox>
#include <QTabWidget>
#include <QWidget>
#include <QSpinBox>
#include <QPushButton>
#include <QSettings>

SettingsDialog::SettingsDialog(QWidget* parent)
	: QDialog(parent)
{
	//�����Ի���TAB��ǩҳ
	m_settingTabWidget = new QTabWidget(this);
	m_settingTabWidget->addTab(createGeneralTabWidget(), tr("General"));
	m_settingTabWidget->addTab(createInterfaceTabWidget(), tr("Interface"));
	//������ť
	m_okButton = new QPushButton(tr("ok"), this);
	m_cancelButton = new QPushButton(tr("cancel"), this);
	//��������
	QHBoxLayout* buttonLayout = new QHBoxLayout;
	buttonLayout->addStretch();
	buttonLayout->addWidget(m_okButton);
	buttonLayout->addWidget(m_cancelButton);
	QVBoxLayout* mainLayout = new QVBoxLayout;
	mainLayout->addWidget(m_settingTabWidget);
	mainLayout->addLayout(buttonLayout);
	setLayout(mainLayout);

	//��ȡ֮ǰ���������
	readSettings();
	//�����źŲ�
	connect(m_okButton, &QPushButton::clicked, this, &QDialog::accept);
	connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);

}

void SettingsDialog::readSettings()
{
	QSettings settings("ZJU", "scanner");
	//��һ�δ���ע���
	if (!settings.contains("shutterTime"))
		return;
	//��ȡ����
	m_shutterTime->setValue(settings.value("shutterTime").toUInt());
	m_idleTime->setValue(settings.value("idleTime").toUInt());
	m_resolutionComBox->setCurrentIndex(m_resolutionComBox->findText(settings.value("resolution").toString()));
}

void SettingsDialog::writeSettings()
{
	QSettings settings("ZJU", "scanner");
	settings.setValue("shutterTime", m_shutterTime->value());
	settings.setValue("idleTime", m_idleTime->value());
	settings.setValue("resolution", m_resolutionComBox->currentText().toUInt());
	
	settings.sync();
}

QWidget * SettingsDialog::createGeneralTabWidget()
{
	//��������ʱ�����ÿ�
	m_shutterTimeLabel = new QLabel(tr("Shutter time: "), this);
	m_shutterTimeLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	m_idleTimeLabel = new QLabel(tr("Idle time: "), this);
	m_idleTimeLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	m_shutterTime = new QSpinBox(this);
	m_shutterTime->setSingleStep(10);
	m_shutterTime->setRange(1, 2000);
	m_shutterTime->setSuffix(" us");
	m_shutterTime->setAlignment(Qt::AlignRight);
	m_idleTime = new QSpinBox(this);
	m_idleTime->setSingleStep(10);
	m_idleTime->setRange(1, 2000);
	m_idleTime->setSuffix(" us");
	m_idleTime->setAlignment(Qt::AlignRight);
	m_shutterGroupBox = new QGroupBox(tr("ShutterTime"), this);
	//��������ʱ�����ÿ򲼾�
	QHBoxLayout *shutterGroupLayout = new QHBoxLayout;
	shutterGroupLayout->addWidget(m_shutterTimeLabel);
	shutterGroupLayout->addStretch();
	shutterGroupLayout->addWidget(m_shutterTime);
	shutterGroupLayout->addWidget(m_idleTimeLabel);
	shutterGroupLayout->addStretch();
	shutterGroupLayout->addWidget(m_idleTime);
	m_shutterGroupBox->setLayout(shutterGroupLayout);

	//����Profile�������ÿ�
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
	//����Profile�������ÿ򲼾�
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

	//����Sensor���ÿ�
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
	//����Sensor���ÿ򲼾�
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
	//����Generalѡ�����
	QVBoxLayout* layoutGeneralTabWidget = new QVBoxLayout;
	layoutGeneralTabWidget->addWidget(m_shutterGroupBox);
	layoutGeneralTabWidget->addWidget(m_profileGroupBox);
	layoutGeneralTabWidget->addWidget(m_sensorGroupBox);

	QWidget* generalTab = new QWidget(this);
	generalTab->setLayout(layoutGeneralTabWidget);
	
	return generalTab;
}

QWidget * SettingsDialog::createInterfaceTabWidget()
{
	//����RS422�ӿ�������
	m_RS422GroupBox = new QGroupBox(tr("RS422"), this);
	m_RS422ModeLabel = new QLabel(tr("RS422 mode: "), this);
	m_RS422ModeComboBox = new QComboBox(this);
	QStringList list;
	list << "serial" << "ext. trigger input" << "ext. trigger output";
	m_RS422ModeComboBox->addItems(list);
	m_serialBaudrateLabel = new QLabel(tr("Serial baudrate: "), this);
	m_serialBaudrateComboBox = new QComboBox(this);
	list.clear();
	list << "9600" << "19200" << "38400" << "57600" << "115200";
	m_serialBaudrateComboBox->addItems(list);
	m_RS422Termination = new QCheckBox(tr("No RS422 termination"),this);
	//����RS422�ӿ������鲼��
	QGridLayout* RS422GridLayout = new QGridLayout;
	RS422GridLayout->addWidget(m_RS422ModeLabel, 0, 0, 1, 1);
	RS422GridLayout->addWidget(m_RS422ModeComboBox, 0, 1, 1, 1);
	RS422GridLayout->addWidget(m_serialBaudrateLabel, 0, 2, 1, 1);
	RS422GridLayout->addWidget(m_serialBaudrateComboBox, 0, 3, 1, 1);
	RS422GridLayout->addWidget(m_RS422Termination);
	m_RS422GroupBox->setLayout(RS422GridLayout);

	//����digital inputs������
	m_digitalInputsGroupBox = new QGroupBox(tr("Digital inputs"), this);
	m_inputsModeLabel = new QLabel(tr("Digital inputs mode: "), this);
	m_inputsModeComboBox = new QComboBox(this);
	list.clear();
	list << "encoder + reset" << "encode + trigger" << "trigger" << "user modes + trigger" << "user modes" << "timestamp";
	m_inputsModeComboBox->addItems(list);
	m_inputsLogicLabel = new QLabel(tr("Digital inputs logic: "), this);
	m_inputsLogicComboBox = new QComboBox(this);
	list.clear();
	list << "low level logic(5V)" << "high level logic(24V)";
	m_inputsLogicComboBox->addItems(list);
	//����digital inputs����
	QHBoxLayout* digitalInputsLayout = new QHBoxLayout;
	digitalInputsLayout->addWidget(m_inputsModeLabel);
	digitalInputsLayout->addWidget(m_inputsModeComboBox);
	digitalInputsLayout->addWidget(m_inputsLogicLabel);
	digitalInputsLayout->addWidget(m_inputsLogicComboBox);
	m_digitalInputsGroupBox->setLayout(digitalInputsLayout);

	//����trigger
	m_triggerGroupBox = new QGroupBox(tr("Trigger"), this);
	m_triggerModeLabel = new QLabel(tr("Trigger Mode"), this);
	m_triggerModeComboBox = new QComboBox(this);
	list.clear();
	list << "Encoder" << "internal" << "pos. edge" << "neg. edge" << "pos. pulse" << "neg. pulse" << "pos. gate" << "neg. gate";
	m_triggerModeComboBox->addItems(list);
	m_triggerSourceLabel = new QLabel(tr("Trigger source: "), this);
	m_triggerSourceComboBox = new QComboBox(this);
	m_triggerSourceComboBox->addItem("RS422");
	m_triggerSourceComboBox->addItem("digital inputs");
	m_encodeStepLabel = new QLabel(tr("Encode step: "), this);
	m_encodeStepSpinBox = new QSpinBox(this);
	m_encodeStepSpinBox->setRange(1, 100);
	m_encodeStepSpinBox->setSingleStep(1);
	m_encodeStepSpinBox->setAlignment(Qt::AlignRight);
	m_encodeActiveCBox = new QCheckBox(tr("Encode active"), this);
	QGridLayout* triggerGridLayout = new QGridLayout;
	triggerGridLayout->addWidget(m_triggerModeLabel,0,0,1,1);
	triggerGridLayout->addWidget(m_triggerModeComboBox, 0, 1, 1, 1);
	triggerGridLayout->addWidget(m_triggerSourceLabel, 0, 2, 1, 1);
	triggerGridLayout->addWidget(m_triggerSourceComboBox, 0, 3, 1, 1);
	triggerGridLayout->addWidget(m_encodeStepLabel, 1, 0, 1, 1);
	triggerGridLayout->addWidget(m_encodeStepSpinBox, 1, 1, 1, 1);
	triggerGridLayout->addWidget(m_encodeActiveCBox, 1, 3, 1, 1);
	m_triggerGroupBox->setLayout(triggerGridLayout);

	QVBoxLayout* layoutInterfaceTabWidget = new QVBoxLayout;
	layoutInterfaceTabWidget->addWidget(m_RS422GroupBox);
	layoutInterfaceTabWidget->addWidget(m_digitalInputsGroupBox);
	layoutInterfaceTabWidget->addWidget(m_triggerGroupBox);

	QWidget* interfaceTab = new QWidget(this);
	interfaceTab->setLayout(layoutInterfaceTabWidget);

	return interfaceTab;
}
