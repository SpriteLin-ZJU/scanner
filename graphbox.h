#pragma once
#include "graphmodifier.h"
#include <QWidget>

class QGroupBox;
class QRadioButton;
class QSlider;

class GraphBox : public QWidget
{
	Q_OBJECT
public:
	GraphBox(Q3DSurface *graph, QWidget *parent = Q_NULLPTR);

	void updateGraph(unsigned int resolution) {
		m_modifier->updateGraph(resolution);
	}
private:
	GraphModifier * m_modifier;
	//切面选择对话框
	QGroupBox* m_selectionGroupBox;
	QRadioButton* m_modeNoneRB;
	QRadioButton* m_modeItemRB;
	QRadioButton* m_modeSliceRowRB;
	QRadioButton* m_modeSliceColumRB;
	//坐标范围
	QSlider* m_axisMinSliderX;
	QSlider* m_axisMaxSliderX;
	QSlider* m_axisMinSliderY;
	QSlider* m_axisMaxSliderY;

};