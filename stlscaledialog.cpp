#include "stlscaledialog.h"
#include <QLabel>
#include <QDoubleSpinBox>
#include <QLayout>

STLScaleDialog::STLScaleDialog(QWidget* parent)
	:QDialog(parent)
{
	m_xScaleLabel = new QLabel(tr("X "), this);
	m_xScaleLabel->setStyleSheet("color:red;");
	m_yScaleLabel = new QLabel(tr("Y "), this);
	m_yScaleLabel->setStyleSheet("color:green;");
	m_zScaleLabel = new QLabel(tr("Z "), this);
	m_zScaleLabel->setStyleSheet("color:blue;");

	m_xScaleSpinBox = new QDoubleSpinBox(this);
	m_xScaleSpinBox->setSingleStep(10);
	m_xScaleSpinBox->setRange(1.0, 1000.0);
	m_xScaleSpinBox->setSuffix(" %");
	m_yScaleSpinBox = new QDoubleSpinBox(this);
	m_yScaleSpinBox->setSingleStep(10);
	m_yScaleSpinBox->setRange(1.0, 1000.0);
	m_yScaleSpinBox->setSuffix(" %");
	m_zScaleSpinBox = new QDoubleSpinBox(this);
	m_zScaleSpinBox->setSingleStep(10);
	m_zScaleSpinBox->setRange(1.0, 1000.0);
	m_zScaleSpinBox->setSuffix(" %");

	QVBoxLayout* vLayoutLeft = new QVBoxLayout;
	vLayoutLeft->addWidget(m_xScaleLabel, 1, Qt::AlignLeft);
	vLayoutLeft->addWidget(m_yScaleLabel, 1, Qt::AlignLeft);
	vLayoutLeft->addWidget(m_zScaleLabel, 1, Qt::AlignLeft);
	QVBoxLayout* vLayoutRight = new QVBoxLayout;
	vLayoutRight->addWidget(m_xScaleSpinBox, 1, Qt::AlignRight);
	vLayoutRight->addWidget(m_yScaleSpinBox, 1, Qt::AlignRight);
	vLayoutRight->addWidget(m_zScaleSpinBox, 1, Qt::AlignRight);
	QHBoxLayout* hLayout = new QHBoxLayout;
	hLayout->addLayout(vLayoutLeft);
	hLayout->addLayout(vLayoutRight);
	setLayout(hLayout);

	Qt::WindowFlags flags = Qt::Dialog;
	flags |= Qt::WindowCloseButtonHint;
	setWindowFlags(flags);

	void (QDoubleSpinBox::*fun)(double) = &QDoubleSpinBox::valueChanged;	//取重载函数中参数为double类型的函数指针
	connect(m_xScaleSpinBox, fun, this, &STLScaleDialog::emitScaleSig);
	connect(m_yScaleSpinBox, fun, this, &STLScaleDialog::emitScaleSig);
	connect(m_zScaleSpinBox, fun, this, &STLScaleDialog::emitScaleSig);
}
STLScaleDialog::~STLScaleDialog()
{
}

void STLScaleDialog::setScaleValue(QVector3D scale)
{
	m_xScaleSpinBox->setValue(scale.x());
	m_yScaleSpinBox->setValue(scale.y());
	m_zScaleSpinBox->setValue(scale.z());
}

void STLScaleDialog::emitScaleSig()
{
	QVector3D scale(m_xScaleSpinBox->value(), m_yScaleSpinBox->value(), m_zScaleSpinBox->value());
	emit scaleSTL(scale);
}
