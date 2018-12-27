#include "stlrotatedialog.h"
#include <QLabel>
#include <QDoubleSpinBox>
#include <QLayout>

STLRotateDialog::STLRotateDialog(QWidget* parent)
	:QDialog(parent)
{
	m_xRotLabel = new QLabel(tr("X "), this);
	m_xRotLabel->setStyleSheet("color:red;");
	m_yRotLabel = new QLabel(tr("Y "), this);
	m_yRotLabel->setStyleSheet("color:green;");
	m_zRotLabel = new QLabel(tr("Z "), this);
	m_zRotLabel->setStyleSheet("color:blue;");

	m_xRotSpinBox = new QDoubleSpinBox(this);
	m_xRotSpinBox->setSingleStep(45);
	m_xRotSpinBox->setRange(-180.0, 180.0);
	m_xRotSpinBox->setSuffix(" degree");
	m_yRotSpinBox = new QDoubleSpinBox(this);
	m_yRotSpinBox->setSingleStep(45);
	m_yRotSpinBox->setRange(-180.0, 180.0);
	m_yRotSpinBox->setSuffix(" degree");
	m_zRotSpinBox = new QDoubleSpinBox(this);
	m_zRotSpinBox->setSingleStep(45);
	m_zRotSpinBox->setRange(-180.0, 180.0);
	m_zRotSpinBox->setSuffix(" degree");

	QVBoxLayout* vLayoutLeft = new QVBoxLayout;
	vLayoutLeft->addWidget(m_xRotLabel, 1, Qt::AlignLeft);
	vLayoutLeft->addWidget(m_yRotLabel, 1, Qt::AlignLeft);
	vLayoutLeft->addWidget(m_zRotLabel, 1, Qt::AlignLeft);
	QVBoxLayout* vLayoutRight = new QVBoxLayout;
	vLayoutRight->addWidget(m_xRotSpinBox, 1, Qt::AlignRight);
	vLayoutRight->addWidget(m_yRotSpinBox, 1, Qt::AlignRight);
	vLayoutRight->addWidget(m_zRotSpinBox, 1, Qt::AlignRight);
	QHBoxLayout* hLayout = new QHBoxLayout;
	hLayout->addLayout(vLayoutLeft);
	hLayout->addLayout(vLayoutRight);
	setLayout(hLayout);

	Qt::WindowFlags flags = Qt::Dialog;
	flags |= Qt::WindowCloseButtonHint;
	setWindowFlags(flags);

	void (QDoubleSpinBox::*fun)(double) = &QDoubleSpinBox::valueChanged;	//取重载函数中参数为double类型的函数指针
	connect(m_xRotSpinBox, fun, this, &STLRotateDialog::emitRotateSig);
	connect(m_yRotSpinBox, fun, this, &STLRotateDialog::emitRotateSig);
	connect(m_zRotSpinBox, fun, this, &STLRotateDialog::emitRotateSig);
}
STLRotateDialog::~STLRotateDialog()
{
}

void STLRotateDialog::setRotateValue(QVector3D rotate)
{
	m_xRotSpinBox->setValue(rotate.x());
	m_yRotSpinBox->setValue(rotate.y());
	m_zRotSpinBox->setValue(rotate.z());
}

void STLRotateDialog::emitRotateSig()
{
	QVector3D position(m_xRotSpinBox->value(), m_yRotSpinBox->value(), m_zRotSpinBox->value());
	emit rotateSTL(position);
}
