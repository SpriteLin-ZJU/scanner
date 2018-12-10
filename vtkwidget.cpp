#include "vtkwidget.h"
#include <QDeBug>

void pointPickingOccured(const pcl::visualization::PointPickingEvent & event, void * viewer_void)
{
	float x, y, z;
	event.getPoint(x, y, z);
	qDebug() << x << y << z << endl;
}

VTKWidget::VTKWidget(QWidget* parent)
	:QVTKWidget(parent),
	viewPort1(0),
	viewPort2(0)
{
	m_viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	m_viewer->registerPointPickingCallback(pointPickingOccured, (void*)&m_viewer);
	m_viewer->initCameraParameters();

	m_viewer->createViewPort(0.0, 0.0, 0.5, 1.0, viewPort1);
	m_viewer->setBackgroundColor(0.7, 0.7, 0.7, viewPort1);
	m_viewer->addText("Clouds before processing", 10, 10, "v1 text", viewPort1);

	m_viewer->createViewPort(0.5, 0.0, 1.0, 1.0, viewPort2);
	m_viewer->setBackgroundColor(0.3, 0.3, 0.3, viewPort2);
	m_viewer->addText("Clouds after processing", 10, 10, "v2 text", viewPort2);
	m_viewer->addCoordinateSystem(1.0);

	SetRenderWindow(m_viewer->getRenderWindow());
	m_viewer->setupInteractor(GetInteractor(), GetRenderWindow());
	m_viewer->resetCamera();
	this->update();
}

VTKWidget::~VTKWidget()
{
}

void VTKWidget::updateViewer(int viewPort, QVector<PointCloudT::Ptr> clouds)
{
	m_viewer->removeAllPointClouds(viewPort);
	for (int i = 0; i < clouds.size(); i++) {
		std::stringstream ss;
		ss << "port" << viewPort << i;
		m_viewer->addPointCloud<PointT> (clouds[i], ss.str(), viewPort);
	}
	this->update();
}

void VTKWidget::updateViewer(int viewPort, PointCloudT::Ptr cloud)
{
	m_viewer->removeAllPointClouds(viewPort);
	std::stringstream ss;
	ss << "port" << viewPort;
	m_viewer->addPointCloud<PointT> (cloud, ss.str(), viewPort);
	this->update();
}

void VTKWidget::addViewer(int viewPort, PointCloudT::Ptr cloud, QString id)
{
	std::string idString = id.toStdString();
	m_viewer->removePointCloud(idString, viewPort);
	m_viewer->addPointCloud<PointT>(cloud, idString, viewPort);
	this->update();
}

void VTKWidget::removeViewPortPointClouds(int viewPort)
{
	m_viewer->removeAllPointClouds(viewPort);
	this->update();
}

