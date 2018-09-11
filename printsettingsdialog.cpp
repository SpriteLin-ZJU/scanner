#include "printsettingsdialog.h"
#include <QLabel>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QSettings>
#include <QLayout>

PrintSettingsDialog::PrintSettingsDialog(QWidget* parent)
	:QDialog(parent)
{
	//创建布局
	m_layerHeightLabel = new QLabel(tr("Layer height: "), this);
	m_layerHeightLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	m_layerHeightSpinBox = new QDoubleSpinBox(this);
	m_layerHeightSpinBox->setSingleStep(0.1);
	
	m_stlModelColorLabel = new QLabel(tr("STL color: "), this);
	m_stlRLabel = new QLabel(tr("R:"), this);
	m_stlRSpinBox = new QDoubleSpinBox(this);
	m_stlRSpinBox->setSingleStep(0.1);
	m_stlRSpinBox->setRange(0.0, 1.0);
	m_stlGLabel = new QLabel(tr("G:"), this);
	m_stlGSpinBox = new QDoubleSpinBox(this);
	m_stlGSpinBox->setSingleStep(0.1);
	m_stlGSpinBox->setRange(0.0, 1.0);
	m_stlBLabel = new QLabel(tr("B:"), this);
	m_stlBSpinBox = new QDoubleSpinBox(this);
	m_stlBSpinBox->setSingleStep(0.1);
	m_stlBSpinBox->setRange(0.0, 1.0);

	m_sliceColorLabel = new QLabel(tr("Slice color: "), this);
	m_sliceRLabel = new QLabel(tr("R:"), this);
	m_sliceRSpinBox = new QDoubleSpinBox(this);
	m_sliceRSpinBox->setSingleStep(0.1);
	m_sliceRSpinBox->setRange(0.0, 1.0);
	m_sliceGLabel = new QLabel(tr("G:"), this);
	m_sliceGSpinBox = new QDoubleSpinBox(this);
	m_sliceGSpinBox->setSingleStep(0.1);
	m_sliceGSpinBox->setRange(0.0, 1.0);
	m_sliceBLabel = new QLabel(tr("B:"), this);
	m_sliceBSpinBox = new QDoubleSpinBox(this);
	m_sliceBSpinBox->setSingleStep(0.1);
	m_sliceBSpinBox->setRange(0.0, 1.0);

	m_gcodeColorLabel = new QLabel(tr("Gcode color: "), this);
	m_gcodeRLabel = new QLabel(tr("R:"), this);
	m_gcodeRSpinBox = new QDoubleSpinBox(this);
	m_gcodeRSpinBox->setSingleStep(0.1);
	m_gcodeRSpinBox->setRange(0.0, 1.0);
	m_gcodeGLabel = new QLabel(tr("G:"), this);
	m_gcodeGSpinBox = new QDoubleSpinBox(this);
	m_gcodeGSpinBox->setSingleStep(0.1);
	m_gcodeGSpinBox->setRange(0.0, 1.0);
	m_gcodeBLabel = new QLabel(tr("B:"), this);
	m_gcodeBSpinBox = new QDoubleSpinBox(this);
	m_gcodeBSpinBox->setSingleStep(0.1);
	m_gcodeBSpinBox->setRange(0.0, 1.0);

	QGridLayout* gridLayout = new QGridLayout;
	gridLayout->addWidget(m_layerHeightLabel, 0, 0, 1, 2);
	gridLayout->addWidget(m_layerHeightSpinBox, 0, 6, 1, 2);

	gridLayout->addWidget(m_stlModelColorLabel, 1, 0, 1, 2);
	gridLayout->addWidget(m_stlRLabel, 1, 2, 1, 1);
	gridLayout->addWidget(m_stlRSpinBox, 1, 3, 1, 1);
	gridLayout->addWidget(m_stlGLabel, 1, 4, 1, 1);
	gridLayout->addWidget(m_stlGSpinBox, 1, 5, 1, 1);
	gridLayout->addWidget(m_stlBLabel, 1, 6, 1, 1);
	gridLayout->addWidget(m_stlBSpinBox, 1, 7, 1, 1);

	gridLayout->addWidget(m_sliceColorLabel, 2, 0, 1, 2);
	gridLayout->addWidget(m_sliceRLabel, 2, 2, 1, 1);
	gridLayout->addWidget(m_sliceRSpinBox, 2, 3, 1, 1);
	gridLayout->addWidget(m_sliceGLabel, 2, 4, 1, 1);
	gridLayout->addWidget(m_sliceGSpinBox, 2, 5, 1, 1);
	gridLayout->addWidget(m_sliceBLabel, 2, 6, 1, 1);
	gridLayout->addWidget(m_sliceBSpinBox, 2, 7, 1, 1);

	gridLayout->addWidget(m_gcodeColorLabel, 3, 0, 1, 2);
	gridLayout->addWidget(m_gcodeRLabel, 3, 2, 1, 1);
	gridLayout->addWidget(m_gcodeRSpinBox, 3, 3, 1, 1);
	gridLayout->addWidget(m_gcodeGLabel, 3, 4, 1, 1);
	gridLayout->addWidget(m_gcodeGSpinBox, 3, 5, 1, 1);
	gridLayout->addWidget(m_gcodeBLabel, 3, 6, 1, 1);
	gridLayout->addWidget(m_gcodeBSpinBox, 3, 7, 1, 1);

	QHBoxLayout* hboxLayout = new QHBoxLayout;
	m_okButton = new QPushButton(tr("ok"), this);
	m_cancelButton = new QPushButton(tr("cancel"), this);
	hboxLayout->addStretch();
	hboxLayout->addWidget(m_okButton);
	hboxLayout->addWidget(m_cancelButton);

	QVBoxLayout* vboxLayout = new QVBoxLayout;
	vboxLayout->addLayout(gridLayout);
	vboxLayout->addLayout(hboxLayout);

	setLayout(vboxLayout);

	//读取之前保存的设置
	readSettings();
	//创建信号槽
	connect(m_okButton, &QPushButton::clicked, this, &QDialog::accept);
	connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);
}

void PrintSettingsDialog::readSettings()
{
	QSettings settings("ZJU", "scanner");
	//第一次创建注册表
	if (!settings.contains("layerHeight"))
		return;
	m_layerHeightSpinBox->setValue(settings.value("layerHeight").toDouble());
	m_stlRSpinBox->setValue(settings.value("stlR").toDouble());
	m_stlGSpinBox->setValue(settings.value("stlG").toDouble());
	m_stlBSpinBox->setValue(settings.value("stlB").toDouble());
	m_sliceRSpinBox->setValue(settings.value("sliceR").toDouble());
	m_sliceGSpinBox->setValue(settings.value("sliceG").toDouble());
	m_sliceBSpinBox->setValue(settings.value("sliceB").toDouble());
	m_gcodeRSpinBox->setValue(settings.value("gcodeR").toDouble());
	m_gcodeGSpinBox->setValue(settings.value("gcodeG").toDouble());
	m_gcodeBSpinBox->setValue(settings.value("gcodeB").toDouble());
}

void PrintSettingsDialog::writeSetting()
{
	QSettings settings("ZJU", "scanner");
	settings.setValue("layerHeight", m_layerHeightSpinBox->value());
	settings.setValue("stlR", m_stlRSpinBox->value());
	settings.setValue("stlG", m_stlGSpinBox->value());
	settings.setValue("stlB", m_stlBSpinBox->value());
	settings.setValue("sliceR", m_sliceRSpinBox->value());
	settings.setValue("sliceG", m_sliceGSpinBox->value());
	settings.setValue("sliceB", m_sliceBSpinBox->value());
	settings.setValue("gcodeR", m_gcodeRSpinBox->value());
	settings.setValue("gcodeG", m_gcodeGSpinBox->value());
	settings.setValue("gcodeB", m_gcodeBSpinBox->value());

	settings.sync();
}
