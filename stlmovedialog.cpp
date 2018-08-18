#include "stlmovedialog.h"
#include <QLabel>
#include <QDoubleSpinBox>
#include <QLayout>

STLMoveDialog::STLMoveDialog(QWidget* parent)
	:QDialog(parent)
{
	m_xMoveLabel = new QLabel(tr("X "), this);
	m_xMoveLabel->setStyleSheet("color:red;");
	m_yMoveLabel = new QLabel(tr("Y "), this);
	m_yMoveLabel->setStyleSheet("color:green;");
	m_zMoveLabel = new QLabel(tr("Z "), this);
	m_zMoveLabel->setStyleSheet("color:blue;");

	m_xMoveSpinBox = new QDoubleSpinBox(this);
	m_xMoveSpinBox->setSingleStep(10);
	m_xMoveSpinBox->setRange(-500.0, 500.0);
	m_xMoveSpinBox->setSuffix(" mm");
	m_yMoveSpinBox = new QDoubleSpinBox(this);
	m_yMoveSpinBox->setSingleStep(10);
	m_yMoveSpinBox->setRange(-500.0, 500.0);
	m_yMoveSpinBox->setSuffix(" mm");
	m_zMoveSpinBox = new QDoubleSpinBox(this);
	m_zMoveSpinBox->setSingleStep(10);
	m_zMoveSpinBox->setRange(-500.0, 500.0);
	m_zMoveSpinBox->setSuffix(" mm");

	QVBoxLayout* vLayoutLeft = new QVBoxLayout;
	vLayoutLeft->addWidget(m_xMoveLabel, 1, Qt::AlignLeft);
	vLayoutLeft->addWidget(m_yMoveLabel, 1, Qt::AlignLeft);
	vLayoutLeft->addWidget(m_zMoveLabel, 1, Qt::AlignLeft);
	QVBoxLayout* vLayoutRight = new QVBoxLayout;
	vLayoutRight->addWidget(m_xMoveSpinBox, 1, Qt::AlignRight);
	vLayoutRight->addWidget(m_yMoveSpinBox, 1, Qt::AlignRight);
	vLayoutRight->addWidget(m_zMoveSpinBox, 1, Qt::AlignRight);
	QHBoxLayout* hLayout = new QHBoxLayout;
	hLayout->addLayout(vLayoutLeft);
	hLayout->addLayout(vLayoutRight);
	setLayout(hLayout);

	Qt::WindowFlags flags = Qt::Dialog;
	flags |= Qt::WindowCloseButtonHint;
	setWindowFlags(flags);

	void (QDoubleSpinBox::*fun)(double) = &QDoubleSpinBox::valueChanged;	//取重载函数中参数为double类型的函数指针
	connect(m_xMoveSpinBox, fun, this, &STLMoveDialog::emitMoveSig);
	connect(m_yMoveSpinBox, fun, this, &STLMoveDialog::emitMoveSig);
	connect(m_zMoveSpinBox, fun, this, &STLMoveDialog::emitMoveSig);

}

STLMoveDialog::~STLMoveDialog()
{
}

void STLMoveDialog::setMoveValue(QVector3D position)
{
	m_xMoveSpinBox->setValue(position.x());
	m_yMoveSpinBox->setValue(position.y());
	m_zMoveSpinBox->setValue(position.z());
}

void STLMoveDialog::emitMoveSig()
{
	QVector3D position(m_xMoveSpinBox->value(), m_yMoveSpinBox->value(), m_zMoveSpinBox->value());
	emit moveSTL(position);
}

