#include "graphmodifier.h"

GraphModifier::GraphModifier(Q3DSurface* graph)
	:m_graph(graph)
{
	createAxisSettings();
	m_dataArray=new QSurfaceDataArray;
	m_newRow = new QSurfaceDataRow;
}

GraphModifier::~GraphModifier()
{
	delete m_graph;
}

void GraphModifier::setBlackToYellowGradient()
{
	QLinearGradient gr;
	gr.setColorAt(0.0, Qt::black);
	gr.setColorAt(0.33, Qt::blue);
	gr.setColorAt(0.67, Qt::red);
	gr.setColorAt(1.0, Qt::yellow);

	m_graph->seriesList().at(0)->setBaseGradient(gr);
	m_graph->seriesList().at(0)->setColorStyle(Q3DTheme::ColorStyleRangeGradient);
}

void GraphModifier::createAxisSettings()
{
	m_graph->setAxisX(new QValue3DAxis);
	m_graph->setAxisY(new QValue3DAxis);
	m_graph->setAxisZ(new QValue3DAxis);

	//设置数据格式
	m_scanDataProxy = new QSurfaceDataProxy();
	m_scanDataSeries = new QSurface3DSeries(m_scanDataProxy);
	m_scanDataSeries->setDrawMode(QSurface3DSeries::DrawSurfaceAndWireframe);
	m_scanDataSeries->setItemLabelFormat(QStringLiteral("x: @xLabel ,y: @zLabel ,z: @yLabel"));	//Qt默认Y轴朝上
	
	m_graph->addSeries(m_scanDataSeries);

	//设置坐标参数
	m_graph->axisX()->setLabelFormat("%.2f mm");
	m_graph->axisY()->setLabelFormat("%.2f mm");
	m_graph->axisZ()->setLabelFormat("%.2f mm");
	
	//m_graph->axisX()->setRange(-30.0f, 30.0f);
	//m_graph->axisY()->setRange(0.0f, 50.0f);
	//m_graph->axisZ()->setRange(0.0f, 100.0f);

	m_graph->axisX()->setLabelAutoRotation(30);
	m_graph->axisY()->setLabelAutoRotation(90);
	m_graph->axisZ()->setLabelAutoRotation(30);


}

void GraphModifier::updateGraph(unsigned int resolution)
{
	m_dataArray->clear();
	m_newRow->resize(resolution);

	//存入数据
	int profileCount = vdValueX.size() / resolution;
	for (int i = 0; i < profileCount; i++) {
		for (int j = 0; j < resolution; j++) {
			float x = (float)i;
			float y = (float)vdValueZ[i*resolution + j];
			float z = (float)vdValueX[i*resolution + j];
			(*m_newRow)[j].setPosition(QVector3D(x, y, z));
		}
		*m_dataArray << m_newRow;
	}

	//m_scanDataProxy->resetArray(NULL);
	m_scanDataProxy->resetArray(m_dataArray);
}
