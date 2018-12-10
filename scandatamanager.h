#pragma once
#include <iostream>

//point cloud library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <QObject>
#include <QString>
#include <QVector>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class ScandataManager : public QObject
{
	Q_OBJECT
public:
	ScandataManager();
	~ScandataManager();

	bool isEmpty();

	void scandataToPoint(QVector<double> valueX, QVector<double> valueY, QVector<double> valueZ, QVector<unsigned short> intensity, unsigned int resolution);
	void stlLayerToPoint(QVector<double> valueX, QVector<double> valueY, double z);
	//槽
	void openPcdFile(QString path);
	void savePcdFile(QString path);
	void clear();
	//槽：点云处理
	void filterPointCloud(int meanK, double thresh);
	void sacPointCloud(int maxIterations, double thresh);
	void icpPointCloud(int maxIterations, double eucliEpsilon);
	void findPointCloudBoundary(int maxSacIterations, double sacThresh, int normKSearch, int boundKSearch);
	void resetPointCloud();

	//点云
	QVector<PointCloudT::Ptr> m_clouds;
	QVector<PointCloudT::Ptr> m_filteredClouds;
	PointCloudT::Ptr m_finalCloud;
	PointCloudT::Ptr m_stlLayerCloud;
signals:
	void drawPointClouds();
	void updateStatus(QString&);
	void updateViewer(int viewPort, QVector<PointCloudT::Ptr> clouds);
	void updateViewer(int viewPort, PointCloudT::Ptr cloud);
	void addViewer(int viewPort, PointCloudT::Ptr cloud, QString id);
	void removeViewPortPointCloud(int viewPort);	//0为ALL，1为左半视图，2为右半视图。
};