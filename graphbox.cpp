#include "graphbox.h"
#include <QGroupBox>
#include <QRadioButton>
#include <QLayout>
#include <QSlider>
#include <QLabel>

GraphBox::GraphBox( QWidget *parent)
	:QWidget(parent)
{

	//创建界面
	m_selectionGroupBox = new QGroupBox(tr("Selection Mode"), this);

	m_modeNoneRB = new QRadioButton(this);
	m_modeNoneRB->setText(tr("No selection"));
	m_modeNoneRB->setChecked(false);

	m_modeItemRB = new QRadioButton(this);
	m_modeItemRB->setText(tr("Item"));
	m_modeItemRB->setChecked(false);

	m_modeSliceRowRB = new QRadioButton(this);
	m_modeSliceRowRB->setText(tr("Row Slice"));
	m_modeSliceRowRB->setChecked(false);

	m_modeSliceColumRB = new QRadioButton(this);
	m_modeSliceColumRB->setText(tr("Column Slice"));
	m_modeSliceColumRB->setChecked(false);

	QVBoxLayout* selectionVBox = new QVBoxLayout;
	selectionVBox->addWidget(m_modeNoneRB);
	selectionVBox->addWidget(m_modeItemRB);
	selectionVBox->addWidget(m_modeSliceRowRB);
	selectionVBox->addWidget(m_modeSliceColumRB);
	m_selectionGroupBox->setLayout(selectionVBox);

	m_axisMinSliderX = new QSlider(Qt::Horizontal, this);
	m_axisMinSliderX->setMinimum(0);
	m_axisMinSliderX->setTickInterval(1);
	m_axisMinSliderX->setEnabled(true);

	m_axisMaxSliderX = new QSlider(Qt::Horizontal, this);
	m_axisMaxSliderX->setMinimum(1);
	m_axisMaxSliderX->setTickInterval(1);
	m_axisMaxSliderX->setEnabled(true);

	m_axisMinSliderY = new QSlider(Qt::Horizontal, this);
	m_axisMinSliderY->setMinimum(0);
	m_axisMinSliderY->setTickInterval(1);
	m_axisMinSliderY->setEnabled(true);

	m_axisMaxSliderY = new QSlider(Qt::Horizontal, this);
	m_axisMaxSliderY->setMinimum(1);
	m_axisMaxSliderY->setTickInterval(1);
	m_axisMaxSliderY->setEnabled(true);

	//创建布局
	QVBoxLayout* vLayout = new QVBoxLayout;
	vLayout->addWidget(m_selectionGroupBox);
	vLayout->addWidget(new QLabel(tr("Colume range"), this));
	vLayout->addWidget(m_axisMinSliderX);
	vLayout->addWidget(m_axisMaxSliderX);
	vLayout->addWidget(new QLabel(tr("Row range"), this));
	vLayout->addWidget(m_axisMinSliderY);
	vLayout->addWidget(m_axisMaxSliderY);

	setLayout(vLayout);
	
	m_modeItemRB->setChecked(true);

	//建立信号槽
	//connect(m_modeNoneRB, &QRadioButton::toggled, m_modifier, &GraphModifier::toggleModeNone);
	//connect(m_modeItemRB, &QRadioButton::toggled, m_modifier, &GraphModifier::toggleModeItem);
	//connect(m_modeSliceRowRB, &QRadioButton::toggled, m_modifier, &GraphModifier::toggleModeSliceRow);
	//connect(m_modeSliceColumRB, &QRadioButton::toggled, m_modifier, &GraphModifier::toggleModeSliceColumn);

}
