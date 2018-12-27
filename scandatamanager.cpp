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
#include <QFileInfo>

ScandataManager::ScandataManager()
{
	//��������Ϊblocking����ô����ע�ᡣ
	qRegisterMetaType<QMap<QString, PointCloudT::Ptr>>("QMap<QString, PointCloudT::Ptr>");
	qRegisterMetaType<PointCloudT::Ptr>("PointCloudT::Ptr");
	qRegisterMetaType<QString>("QString&");
}

ScandataManager::~ScandataManager()
{
}

bool ScandataManager::isEmpty()
{
	return(m_mapSrcToPointCloud.isEmpty());
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

	//ӳ��������Ƶ�����
	QString srcName = m_mapSrcToPointCloud.mapSrcToPointCloud("Scandata", tempCloud);
	emit signalAddPointCloudList(srcName);
	qDebug() << "Saved " << tempCloud->points.size() << " data points \n" << endl;
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

	//ӳ��������Ƶ�����
	QString srcName = m_mapSrcToPointCloud.mapSrcToPointCloud("StlLayerCloud", tempCloud);
	emit signalAddPointCloudList(srcName);
}

void ScandataManager::openPcdFile(const QString& path)
{
	QFileInfo fileInfo(path);
	if (fileInfo.suffix() != "pcd")
		return;

	emit updateStatus(tr("Opening PCD file."));
	//���ļ�
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
	
	//ӳ���ļ��������ָ��
	QString srcName = m_mapSrcToPointCloud.mapSrcToPointCloud(fileInfo.fileName(), tempCloud);
	emit signalAddPointCloudList(srcName);

	//�����������
	emit updateStatus(tr("Open PCD file sucessful."));
	//��opengl�������
	emit drawPointClouds();
}

//���浱ǰѡ�еĵ���
void ScandataManager::savePcdFile(QString path)
{
	path=path.left(path.length() - 4);

	if (m_currentSelectedPointClouds.size() == 0)
		return;
	int i = 0;
	for (auto srcName : m_currentSelectedPointClouds.keys()) {
		QString suffix = (m_currentSelectedPointClouds.size() == 1) ? ".pcd" : QString::number(i) + ".pcd";
		std::string savePath = (path + suffix).toStdString();
		pcl::io::savePCDFileASCII<PointT>(savePath, *m_currentSelectedPointClouds.value(srcName));
		qDebug() << "Saved " << m_currentSelectedPointClouds[srcName]->points.size()
			<< " data points to " << QString::fromStdString(savePath) << endl;
	}
}

void ScandataManager::clear()
{
	m_mapSrcToPointCloud.clear();
	m_currentSelectedPointClouds.clear();
	emit signalClearPointCloudList();
	emit removeViewPortPointCloud(0);
	emit drawPointClouds();
}

/*���µ�ǰѡ�е����б�*/
void ScandataManager::onUpdateCurrentPointCloud(const QStringList & currentPointCloud)
{
	m_currentSelectedPointClouds.clear();
	for (auto srcPointCloud : currentPointCloud) {
		if (m_mapSrcToPointCloud.getPointCloudFromSrc(srcPointCloud) != NULL)
			m_currentSelectedPointClouds.insert(srcPointCloud, m_mapSrcToPointCloud.getPointCloudFromSrc(srcPointCloud));
	}
	emit repaintPointCloudViewer(1, m_currentSelectedPointClouds);
}

void ScandataManager::filterPointCloud(int meanK, double thresh)
{
	if (m_currentSelectedPointClouds.isEmpty() || m_currentSelectedPointClouds.size()>1) {
		updateStatus("Please select a PointCloud.");
		return;
	}
	emit updateStatus(tr("Filtering Point Cloud."));

	//�Ե��ƽ���ͳ���˲�
	QString srcName = m_currentSelectedPointClouds.keys().at(0);
	PointCloudT::Ptr inputCloud(m_currentSelectedPointClouds.value(srcName));
	PointCloudT::Ptr tempCloud(new PointCloudT);
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(inputCloud);
	qDebug() << "Cloud before filtering: " << inputCloud->points.size() << endl;
	sor.setMeanK(meanK);
	sor.setStddevMulThresh(thresh);
	sor.filter(*tempCloud);
	qDebug() << "Cloud after filtering: " << tempCloud->points.size() << endl;
	m_mapSrcToPointCloud.updateSrcToPointCloud(srcName, tempCloud);

	//���µ�����ͼ
	emit updateStatus(tr("Filter complete."));
	emit updatePointCloudViewer(2, tempCloud, srcName);
}

void ScandataManager::sacPointCloud(int maxIterations, double thresh)
{
	if (m_currentSelectedPointClouds.isEmpty() || m_currentSelectedPointClouds.size() > 1) {
		updateStatus("Please select a PointCloud.");
		return;
	}
	emit updateStatus(tr("SAC Point Cloud."));
	
	//���󴰿���ʾ��SAC����
	emit removeViewPortPointCloud(0);
	emit repaintPointCloudViewer(1, m_currentSelectedPointClouds);
	QString srcName = m_currentSelectedPointClouds.keys().at(0);
	//SAC�ָ�
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

	seg.setInputCloud(m_currentSelectedPointClouds.value(srcName));
	seg.segment(*inliers, *coefficients); 
	if (inliers->indices.size() == 0)
		qDebug() << "Could not estimate a planar model for the given dataset: "<< srcName <<endl;

	//Creat the filtering object
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(m_currentSelectedPointClouds.value(srcName));
	extract.setIndices(inliers);
	extract.setNegative(true);
	PointCloudT::Ptr tempCloud(new PointCloudT);
	extract.filter(*tempCloud);
	m_mapSrcToPointCloud.updateSrcToPointCloud(srcName, tempCloud);
	qDebug() << "SAC successful." << endl;

	emit updateStatus(tr("SAC complete."));
	//���µ�����ͼ
	emit updatePointCloudViewer(2, tempCloud, srcName);
}

void ScandataManager::icpPointCloud(int maxIterations, double eucliEpsilon)
{
	if (m_currentSelectedPointClouds.size() != 2) {
		updateStatus("Please select two PointClouds.");
		return;
	}
	//��ȡ�������
	QString alignedCloudName = m_currentSelectedPointClouds.keys().at(0);
	QString referenceCloudName = m_currentSelectedPointClouds.keys().at(1);
	//icp��׼
	emit updateStatus(tr("ICP begin."));
	//��δ��׼������������ʾ���󴰿�
	emit repaintPointCloudViewer(1, m_currentSelectedPointClouds);
	//icp
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(eucliEpsilon);
	icp.setMaximumIterations(maxIterations);

	PointCloudT::Ptr outCloud(new PointCloudT);
	icp.setInputSource(m_currentSelectedPointClouds.value(alignedCloudName));
	icp.setInputTarget(m_currentSelectedPointClouds.value(referenceCloudName));
	icp.align(*outCloud);	//source�����仯��ĵ���

	//�²���
	//
	emit updateStatus(tr("ICP complete."));
	//������׼��ĵ���
	m_mapSrcToPointCloud.updateSrcToPointCloud(alignedCloudName, outCloud);
	emit repaintPointCloudViewer(2, m_currentSelectedPointClouds);
}

void ScandataManager::findPointCloudBoundary(int maxSacIterations, double sacThresh, int normKSearch, int boundKSearch)
{
	if (m_currentSelectedPointClouds.isEmpty() || m_currentSelectedPointClouds.size() > 1) {
		updateStatus("Please select a PointCloud.");
		return;
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

	//��������
	QString srcName = m_currentSelectedPointClouds.keys().at(0);
	//������ȡ�߽�ĵ�����ʾ�����
	emit removeViewPortPointCloud(0);
	emit repaintPointCloudViewer(1, m_currentSelectedPointClouds);
	//NormalEstimation
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	emit updateStatus(tr("NormalEstimation begin."));
	pcl::NormalEstimationOMP<PointT, pcl::Normal> normEst;
	normEst.setInputCloud(m_currentSelectedPointClouds.value(srcName));
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
	est.setInputCloud(m_currentSelectedPointClouds.value(srcName));
	est.setInputNormals(normals);
	est.setSearchMethod(tree);
	est.setKSearch(boundKSearch);
	//est.setAngleThreshold();
	//est.setRadiusSearch();
	est.compute(boundaries);

	pcl::PointCloud<PointT>::Ptr boundPoints(new pcl::PointCloud<PointT>);
	int countBoundaries = 0;
	for (int i = 0; i < m_currentSelectedPointClouds.value(srcName)->size(); i++) {
		int a = static_cast<int>(boundaries.points[i].boundary_point);
		if (a == 1) {
			//�Ǳ߽��
			boundPoints->push_back(m_currentSelectedPointClouds.value(srcName)->points[i]);
			countBoundaries++;
		}
	}
	qDebug() << "Boundary point size: " << countBoundaries << endl;
	emit updateStatus(tr("BoundaryEstimation done."));
	
	//�б������ӱ߽����
	QString boundaryName = srcName + "_boundary";
	boundaryName = m_mapSrcToPointCloud.mapSrcToPointCloud(boundaryName, boundPoints);
	
	emit updatePointCloudViewer(2, boundPoints, boundaryName);
	emit signalAddPointCloudList(boundaryName);
}


MapSrcToPointCloud::MapSrcToPointCloud()
{
}

QString MapSrcToPointCloud::mapSrcToPointCloud(const QString & srcName, PointCloudT::Ptr pointCloud)
{
	QString src = srcName;
	int i = 1;
	while (m_mapSrcToPointCloud.contains(src)) {
		src = src.left(srcName.size()) + "(" + QString::number(i) + ")";
		i++;
	}
	m_mapSrcToPointCloud.insert(src, pointCloud);
	return src;
}

void MapSrcToPointCloud::updateSrcToPointCloud(const QString & srcName, PointCloudT::Ptr pointCloud)
{
	m_mapSrcToPointCloud.insert(srcName, pointCloud);
}

PointCloudT::Ptr MapSrcToPointCloud::getPointCloudFromSrc(const QString & srcName)
{
	if (m_mapSrcToPointCloud.contains(srcName))
		return m_mapSrcToPointCloud.value(srcName);
	return PointCloudT::Ptr();
}
