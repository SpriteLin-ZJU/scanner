#include "scandatamanager.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <QDebug>

ScandataManager::ScandataManager()
	:viewPort1(0),viewPort2(0)
{
	m_viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	m_viewer->initCameraParameters();
	
	m_viewer->createViewPort(0.0, 0.0, 0.5, 1.0, viewPort1);
	m_viewer->setBackgroundColor(0.8, 0.8, 0.8, viewPort1);
	m_viewer->addText("Clouds before processing", 10, 10, "v1 text", viewPort1);
	
	m_viewer->createViewPort(0.5, 0.0, 1.0, 1.0, viewPort2);
	m_viewer->setBackgroundColor(0.3, 0.3, 0.3, viewPort2);
	m_viewer->addText("Clouds after processing", 10, 10, "v2 text", viewPort2);
	m_viewer->addCoordinateSystem(1.0);
}

ScandataManager::~ScandataManager()
{
}

bool ScandataManager::isEmpty()
{
	return m_clouds.isEmpty();
}

void ScandataManager::scandataToPoint(QVector<double> valueX, QVector<double> valueY, QVector<double> valueZ, unsigned int resolution)
{
	//将扫描得到的数据存入pcl点云
	PointCloudT::Ptr tempCloud(new PointCloudT);
	tempCloud->width = resolution;
	tempCloud->height = valueX.size() / resolution;
	tempCloud->is_dense = true;
	tempCloud->points.resize(valueX.size());
	for (int i = 0; i < tempCloud->points.size(); i++) {
		tempCloud->points[i].x = valueX[i];
		tempCloud->points[i].y = valueY[i];
		tempCloud->points[i].z = valueZ[i];
	}
	m_clouds.push_back(tempCloud);
	qDebug() << "Saved " << tempCloud->points.size() << " data points \n"
		<< "now we have " << m_clouds.size() << " point clouds" << endl;
}

void ScandataManager::openPcdFile(QString path)
{
	PointCloudT::Ptr tempCloud(new PointCloudT);
	if (pcl::io::loadPCDFile<PointT>(path.toStdString(), *tempCloud) == -1) {
		qDebug() << "Couldn't read file \n" << endl;
		return;
	}
	qDebug() << "Loaded"
		<< tempCloud->width*tempCloud->height
		<< " data points from pcd file." << endl;
	//存入点云向量
	m_clouds.push_back(tempCloud);
	updateViewer(viewPort1,m_clouds);

	emit drawPointClouds();
}

void ScandataManager::savePcdFile(QString path)
{
	for (int i = 0; i < m_clouds.size(); i++) {
		std::string savePath = path.toStdString() + '-' + std::to_string(i);
		pcl::io::savePCDFileASCII(savePath, *(m_clouds[i]));
		qDebug() << "Saved " << m_clouds[i]->points.size() 
				<< " data points to " << QString::fromStdString(savePath) << endl;
	}
}

void ScandataManager::clear()
{
	m_clouds.clear();
	m_filteredClouds.clear();
	m_viewer->removeAllPointClouds();
	
	emit updateVTK();
	emit drawPointClouds();
}

void ScandataManager::filterPointCloud(int meanK, double thresh)
{
	if (m_clouds.isEmpty()) {
		qDebug() << "No clouds in vector!" << endl;
		return;
	}
	//如果m_filteredClouds为空，将原点云滤波后存入filteredClouds
	if (m_filteredClouds.isEmpty()) {
		for (int i = 0; i < m_clouds.size(); i++) {
			PointCloudT::Ptr tempCloud(new PointCloudT);
			pcl::StatisticalOutlierRemoval<PointT> sor;
			sor.setInputCloud(m_clouds[i]);
			qDebug() << "Cloud before filtering: " << (m_clouds[i])->points.size() << endl;
			sor.setMeanK(meanK);
			sor.setStddevMulThresh(thresh);
			sor.filter(*tempCloud);
			qDebug() << "Cloud after filtering: " << tempCloud->points.size() << endl;
			m_filteredClouds.push_back(tempCloud);
		}
	}
	//若已有已滤波点云
	else {
		for (int i = 0; i < m_filteredClouds.size(); i++) {
			PointCloudT::Ptr tempCloud(new PointCloudT);
			pcl::StatisticalOutlierRemoval<PointT> sor;
			sor.setInputCloud(m_filteredClouds[i]);
			qDebug() << "Cloud before filtering: " << (m_filteredClouds[i])->points.size() << endl;
			sor.setMeanK(meanK);
			sor.setStddevMulThresh(thresh);
			sor.filter(*tempCloud);
			m_filteredClouds[i].swap(tempCloud);
			qDebug() << "Cloud after filtering: " << m_filteredClouds[i]->points.size() << endl;
		}
	}
	//更新点云视图
	updateViewer(viewPort2,m_filteredClouds);
}

void ScandataManager::sacPointCloud(int maxIterations, double thresh)
{
	if (m_clouds.isEmpty()) {
		qDebug() << "No clouds in vector!" << endl;
		return;
	}
	//如果filteredClouds为空，则将m_clouds数据拷入,便于后续程序统一处理
	if (m_filteredClouds.isEmpty()) {
		for (int i = 0; i < m_clouds.size(); i++) {
			PointCloudT::Ptr tempCloud(new PointCloudT);
			pcl::copyPointCloud(*m_clouds[i], *tempCloud);
			m_filteredClouds.push_back(tempCloud);
			qDebug() << "Copy " << tempCloud->points.size() << " data to filteredClouds." << endl;
		}
	}
	else
		updateViewer(viewPort1, m_filteredClouds);
	//SAC分割
	for (int i = 0; i < m_filteredClouds.size(); i++) {
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(maxIterations);
		seg.setDistanceThreshold(thresh);

		seg.setInputCloud(m_filteredClouds[i]);
		seg.segment(*inliers, *coefficients); 
		if (inliers->indices.size() == 0)
			qDebug() << "Could not estimate a planar model for the given dataset: m_filteredClouds["<<i<<"]" <<endl;

		//Creat the filtering object
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(m_filteredClouds[i]);
		extract.setIndices(inliers);
		extract.setNegative(true);
		PointCloudT::Ptr tempCloud(new PointCloudT);
		extract.filter(*tempCloud);
		m_filteredClouds[i].swap(tempCloud);
		qDebug() << "SAC successful." << endl;
	}
	//更新点云视图
	updateViewer(viewPort2,m_filteredClouds);
}

void ScandataManager::resetPointCloud()
{
	m_filteredClouds.clear();
	for (int i = 0; i < m_clouds.size(); i++) {
		PointCloudT::Ptr tempCloud(new PointCloudT);
		pcl::copyPointCloud(*m_clouds[i], *tempCloud);
		m_filteredClouds.push_back(tempCloud);
		qDebug() << "Reset " << tempCloud->points.size() << " data to filteredClouds." << endl;
	}
	m_viewer->removeAllPointClouds();
	updateViewer(viewPort1,m_clouds);
}

void ScandataManager::updateViewer(int viewPort, QVector<PointCloudT::Ptr> clouds)
{
	m_viewer->removeAllPointClouds(viewPort);
	for (int i = 0; i < clouds.size(); i++) {
		std::stringstream ss;
		ss << "port"<<viewPort << i;
		m_viewer->addPointCloud(clouds[i], ss.str(), viewPort);
	}
	emit updateVTK();
}
