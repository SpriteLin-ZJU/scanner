#include "scannerdrawer.h"
#include "scandatamanager.h"
#include <QSettings>

ScannerDrawer::ScannerDrawer()
{
	m_pointSize = 2.0;
}

ScannerDrawer::~ScannerDrawer()
{
}

void ScannerDrawer::setScandataManager(ScandataManager * manager)
{
	m_scandataManager = manager;
}

void ScannerDrawer::drawScandataGL()
{
	ShaderDrawable::update();
	emit updateGraph();
}

bool ScannerDrawer::updateData()
{
	m_points.clear();
	QVector3D color = { 1.0f,0.3f,0.2f };
	for (auto srcName : m_scandataManager->m_currentSelectedPointClouds.keys()) {
		PointCloudT::Ptr pPointCloud = m_scandataManager->m_currentSelectedPointClouds.value(srcName);
		for (int i = 0; i < pPointCloud->size(); i++) {
			QVector3D position = { pPointCloud->points[i].x,pPointCloud->points[i].y,pPointCloud->points[i].z };
			m_points.append({ position,color,m_vectorNaN });
		}
	}

	return true;
}
