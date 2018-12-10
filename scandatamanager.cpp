#include "scandatamanager.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/registration/icp.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/boundary.h>
#include <QDebug>
#include <QMetaType>

ScandataManager::ScandataManager()
	:m_finalCloud(NULL),m_stlLayerCloud(NULL)
{
	//��������Ϊblocking����ô����ע�ᡣ
	qRegisterMetaType<QVector<PointCloudT::Ptr>>("QVector<PointCloudT::Ptr>");
	qRegisterMetaType<PointCloudT::Ptr>("PointCloudT::Ptr");
}

ScandataManager::~ScandataManager()
{
}

bool ScandataManager::isEmpty()
{
	return(m_clouds.isEmpty() && m_filteredClouds.isEmpty() && (m_finalCloud == NULL) && (m_stlLayerCloud == NULL));
}

void ScandataManager::scandataToPoint(QVector<double> valueX, QVector<double> valueY, QVector<double> valueZ, QVector<unsigned short> intensity,unsigned int resolution)
{
	//��ɨ��õ������ݴ���pcl����
	PointCloudT::Ptr tempCloud(new PointCloudT);
	PointT tempPoint;
	for (int i = 0; i < valueX.size(); i++) {
		if (valueZ[i] == 0)	//ɾ����Ч��
			continue;
		tempPoint.x = valueX[i]-80.0;
		tempPoint.y = valueY[i];
		tempPoint.z = valueZ[i]-69.7;
		tempPoint.intensity = intensity[i];
		tempCloud->push_back(tempPoint);
	}
	tempCloud->is_dense = true;
	m_clouds.push_back(tempCloud);
	qDebug() << "Saved " << tempCloud->points.size() << " data points \n"
		<< "now we have " << m_clouds.size() << " point clouds" << endl;
	emit updateViewer(1, m_clouds);
}

void ScandataManager::stlLayerToPoint(QVector<double> valueX, QVector<double> valueY, double z)
{
	PointCloudT::Ptr tempCloud(new PointCloudT);
	PointT tempPoint;
	for (int i = 0; i < valueX.size(); i++) {
		tempPoint.x = valueX[i];
		tempPoint.y = valueY[i];
		tempPoint.z = z;
		tempCloud->push_back(tempPoint);
	}
	tempCloud->is_dense = true;
	m_stlLayerCloud.swap(tempCloud);
	emit addViewer(1, m_stlLayerCloud,"LayerCloud");
}

void ScandataManager::openPcdFile(QString path)
{
	emit updateStatus(tr("Opening PCD file."));
	PointCloudT::Ptr tempCloud(new PointCloudT);
	if (pcl::io::loadPCDFile<PointT>(path.toStdString(), *tempCloud) == -1) {
		qDebug() << "Couldn't read file \n" << endl;
		return;
	}
	qDebug() << "Loaded "
		<< tempCloud->width*tempCloud->height
		<< " data points from pcd file." << endl;
	//ȥ����Ч��
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*tempCloud, *tempCloud, indices);
	qDebug() << "Remove " << indices.size() << "invalid Points." << endl;
	//�����������
	m_clouds.push_back(tempCloud);

	emit updateViewer(1, m_clouds);
	emit updateStatus(tr("Open PCD file sucessful."));
	emit drawPointClouds();
}

void ScandataManager::savePcdFile(QString path)
{
	path=path.left(path.length() - 4);
	for (int i = 0; i < m_clouds.size(); i++) {
		std::string savePath = path.toStdString() + '_' + std::to_string(i)+".pcd";
		pcl::io::savePCDFileASCII<PointT> (savePath, *(m_clouds[i]));
		qDebug() << "Saved " << m_clouds[i]->points.size() 
				<< " data points to " << QString::fromStdString(savePath) << endl;
	}
	for (int i = 0; i < m_filteredClouds.size(); i++) {
		std::string savePath = path.toStdString() + '_' + std::to_string(i)+"filtered" + ".pcd";
		pcl::io::savePCDFileASCII<PointT>(savePath, *(m_filteredClouds[i]));
		qDebug() << "Saved " << m_filteredClouds[i]->points.size()
			<< " data points to " << QString::fromStdString(savePath) << endl;
	}
	if (m_finalCloud != NULL) {
		std::string savePath = path.toStdString() + '_' + "finalCloud" + ".pcd";
		pcl::io::savePCDFileASCII<PointT>(savePath, *m_finalCloud);
	}
	if (m_stlLayerCloud != NULL) {
		std::string savePath = path.toStdString() + '_' + "stlLayerCloud" + ".pcd";
		pcl::io::savePCDFileASCII<PointT>(savePath, *m_stlLayerCloud);
	}
}

void ScandataManager::clear()
{
	m_clouds.clear();
	m_filteredClouds.clear();
	m_finalCloud = NULL;
	emit removeViewPortPointCloud(0);
	emit drawPointClouds();
}

void ScandataManager::filterPointCloud(int meanK, double thresh)
{
	if (m_clouds.isEmpty()) {
		qDebug() << "No clouds in vector!" << endl;
		return;
	}
	emit updateStatus(tr("Filtering Point Cloud."));
	//���m_filteredCloudsΪ�գ���ԭ�����˲������filteredClouds
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
	//���������˲�����
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
	//���µ�����ͼ
	emit updateStatus(tr("Filter complete."));
	emit updateViewer(2, m_filteredClouds);
}

void ScandataManager::sacPointCloud(int maxIterations, double thresh)
{
	if (m_clouds.isEmpty()) {
		qDebug() << "No clouds in vector!" << endl;
		return;
	}
	emit updateStatus(tr("SAC Point Cloud."));
	//���filteredCloudsΪ�գ���m_clouds���ݿ���,���ں�������ͳһ����
	if (m_filteredClouds.isEmpty()) {
		for (int i = 0; i < m_clouds.size(); i++) {
			PointCloudT::Ptr tempCloud(new PointCloudT);
			pcl::copyPointCloud(*m_clouds[i], *tempCloud);
			m_filteredClouds.push_back(tempCloud);
			qDebug() << "Copy " << tempCloud->points.size() << " data to filteredClouds." << endl;
		}
	}
	else
		emit updateViewer(1, m_filteredClouds);
	//SAC�ָ�
	for (int i = 0; i < m_filteredClouds.size(); i++) {
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<PointT> seg;
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
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(m_filteredClouds[i]);
		extract.setIndices(inliers);
		extract.setNegative(true);
		PointCloudT::Ptr tempCloud(new PointCloudT);
		extract.filter(*tempCloud);
		m_filteredClouds[i].swap(tempCloud);
		qDebug() << "SAC successful." << endl;
	}
	emit updateStatus(tr("SAC complete."));
	//���µ�����ͼ
	emit updateViewer(2, m_filteredClouds);
}

void ScandataManager::icpPointCloud(int maxIterations, double eucliEpsilon)
{
	if (m_clouds.isEmpty()) {
		qDebug() << "No clouds in vector!" << endl;
		return;
	}
	//���filteredCloudsΪ�գ���m_clouds���ݿ���,���ں�������ͳһ����
	if (m_filteredClouds.size()<m_clouds.size()) {
		for (int i = m_filteredClouds.size(); i < m_clouds.size(); i++) {
			PointCloudT::Ptr tempCloud(new PointCloudT);
			pcl::copyPointCloud(*m_clouds[i], *tempCloud);
			m_filteredClouds.push_back(tempCloud);
			qDebug() << "Copy " << tempCloud->points.size() << " data to filteredClouds." << endl;
		}
	}
	//����С��2�����޷���׼����
	if (m_filteredClouds.size() < 2) {
		qDebug() << "Less than 2 clouds in vector." << endl;
		return;
	}
	//icp��׼
	emit updateStatus(tr("ICP begin."));
	PointCloudT::Ptr finalCloud(new PointCloudT);
	pcl::copyPointCloud(*m_clouds[0], *finalCloud);
	for (int i = 0; i < m_filteredClouds.size() - 1; i++) {
		emit updateViewer(1, finalCloud);
		emit updateViewer(2, m_filteredClouds[i + 1]);

		pcl::IterativeClosestPoint<PointT, PointT> icp;
		icp.setMaxCorrespondenceDistance(0.1);
		icp.setTransformationEpsilon(1e-10);
		icp.setEuclideanFitnessEpsilon(eucliEpsilon);
		icp.setMaximumIterations(maxIterations);

		PointCloudT::Ptr outCloud(new PointCloudT);
		icp.setInputSource(m_filteredClouds[i+1]);
		icp.setInputTarget(finalCloud);
		icp.align(*outCloud);	//source�����仯��ĵ���

		*finalCloud += *outCloud;
	}
	//�²���
	emit updateStatus(tr("ICP complete."));
	//
	m_finalCloud = finalCloud;
	emit removeViewPortPointCloud(2);
	emit updateViewer(1, m_finalCloud);
}

void ScandataManager::findPointCloudBoundary(int maxSacIterations, double sacThresh, int normKSearch, int boundKSearch)
{
	if (m_clouds.isEmpty()) {
		qDebug() << "No clouds in vector!" << endl;
		return;
	}
	//finalCloud������
	if (m_finalCloud == NULL || m_finalCloud->size() == 0) {
		qDebug() << "No point in m_finalCloud" << endl;
		
		PointCloudT::Ptr tempCloud(new PointCloudT);
		if (m_filteredClouds.isEmpty())
			pcl::copyPointCloud(*m_clouds[0], *tempCloud);
		else
			pcl::copyPointCloud(*m_filteredClouds[0], *tempCloud);
		m_finalCloud.swap(tempCloud);
	}
	/*
	//RANSAC
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(maxSacIterations);
	seg.setDistanceThreshold(sacThresh);

	seg.setInputCloud(m_finalCloud);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0)
		qDebug() << "Could not estimate a planar model for the given dataset: m_finalCloud" << endl;

	//Creat the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(m_finalCloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	PointCloudT::Ptr tempCloud(new PointCloudT);
	extract.filter(*tempCloud);
	m_finalCloud.swap(tempCloud);
	*/


	//NormalEstimation
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	emit updateStatus(tr("NormalEstimation begin."));
	pcl::NormalEstimationOMP<PointT, pcl::Normal> normEst;
	normEst.setInputCloud(m_finalCloud);
	normEst.setNumberOfThreads(6);
	normEst.setSearchMethod(tree);
	normEst.setKSearch(normKSearch);
	normEst.compute(*normals);
	emit updateStatus(tr("NormalEstimation done."));
	//normEst.setViewPoint(0, 0, 0);	//�����ʹ����һ�£�

	//BoundaryEstimation
	emit updateStatus(tr("BoundaryEstimation begin."));
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> est;
	est.setInputCloud(m_finalCloud);
	est.setInputNormals(normals);
	est.setSearchMethod(tree);
	est.setKSearch(boundKSearch);
	//est.setAngleThreshold();
	//est.setRadiusSearch();
	est.compute(boundaries);

	pcl::PointCloud<PointT>::Ptr boundPoints(new pcl::PointCloud<PointT>);
	int countBoundaries = 0;
	for (int i = 0; i < m_finalCloud->size(); i++) {
		int a = static_cast<int>(boundaries.points[i].boundary_point);
		if (a == 1) {
			//�Ǳ߽��
			boundPoints->push_back(m_finalCloud->points[i]);
			countBoundaries++;
		}
	}
	qDebug() << "Boundary point size: " << countBoundaries << endl;
	emit updateStatus(tr("BoundaryEstimation done."));
	m_finalCloud.swap(boundPoints);
	emit updateViewer(1, m_finalCloud);
}

void ScandataManager::resetPointCloud()
{
	m_filteredClouds.clear();
	m_finalCloud = NULL;
	for (int i = 0; i < m_clouds.size(); i++) {
		PointCloudT::Ptr tempCloud(new PointCloudT);
		pcl::copyPointCloud(*m_clouds[i], *tempCloud);
		m_filteredClouds.push_back(tempCloud);
		qDebug() << "Reset " << tempCloud->points.size() << " data to filteredClouds." << endl;
	}

	emit removeViewPortPointCloud(0);
	emit updateViewer(1, m_clouds);
}

