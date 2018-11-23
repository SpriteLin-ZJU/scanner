#pragma once
#include <iostream>

//point cloud library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

#include <QObject>
#include <QString>
#include <QVector>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class ScandataManager : public QObject
{
	Q_OBJECT
public:
	ScandataManager();
	~ScandataManager();

	bool isEmpty();

	void scandataToPoint(QVector<double> valueX, QVector<double> valueY, QVector<double> valueZ, unsigned int resolution);
	//槽
	void openPcdFile(QString path);
	void savePcdFile(QString path);
	void clear();
	//槽：点云处理
	void filterPointCloud(int meanK, double thresh);
	void sacPointCloud(int maxIterations, double thresh);
	void resetPointCloud();
	void updateViewer(int viewPort, QVector<PointCloudT::Ptr> clouds);

	//点云
	QVector<PointCloudT::Ptr> m_clouds;
	QVector<PointCloudT::Ptr> m_filteredClouds;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
	
signals:
	void drawPointClouds();
	void updateVTK();
private:
	int viewPort1;
	int viewPort2;
};