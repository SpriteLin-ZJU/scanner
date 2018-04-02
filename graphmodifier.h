#pragma once
#include <QtDataVisualization/Q3DSurface>
#include <QtDataVisualization/QSurfaceDataProxy>
#include <QtDataVisualization/QHeightMapSurfaceDataProxy>
#include <QtDataVisualization/QSurface3DSeries>
#include <QWidget>

class QSlider;
extern std::vector<double>vdValueX;
extern std::vector<double>vdValueZ;
using namespace QtDataVisualization;

class GraphModifier :public QObject
{
	Q_OBJECT
public:
	explicit GraphModifier(Q3DSurface* graph);
	~GraphModifier();

	//设置图形显示模式
	void toggleModeNone() { m_graph->setSelectionMode(QAbstract3DGraph::SelectionNone); }
	void toggleModeItem() { m_graph->setSelectionMode(QAbstract3DGraph::SelectionItem); }
	void toggleModeSliceRow() {
		m_graph->setSelectionMode(QAbstract3DGraph::SelectionItemAndRow
			| QAbstract3DGraph::SelectionSlice);
	}
	void toggleModeSliceColumn() {
		m_graph->setSelectionMode(QAbstract3DGraph::SelectionItemAndColumn
			| QAbstract3DGraph::SelectionSlice);
	}

	//设置高度颜色渐变
	void setBlackToYellowGradient();

	//设置坐标范围
	void setAxisMinSliderX(QSlider *slider) { m_axisMinSliderX = slider; }
	void setAxisMaxSliderX(QSlider *slider) { m_axisMaxSliderX = slider; }
	void setAxisMinSliderZ(QSlider *slider) { m_axisMinSliderZ = slider; }
	void setAxisMaxSliderZ(QSlider *slider) { m_axisMaxSliderZ = slider; }

	//设置坐标系参数
	void createAxisSettings();

	//绘制
	void updateGraph(unsigned int resolution);

	void adjustXMin(int min);
	void adjustXMax(int max);
	void adjustZMin(int min);
	void adjustZMax(int max);


private:
	Q3DSurface * m_graph;
	QSurfaceDataProxy* m_scanDataProxy;
	QSurface3DSeries* m_scanDataSeries;
	QSurfaceDataArray* m_dataArray;
	QSurfaceDataRow *m_newRow;

	QSlider *m_axisMinSliderX;
	QSlider *m_axisMaxSliderX;
	QSlider *m_axisMinSliderZ;
	QSlider *m_axisMaxSliderZ;
};