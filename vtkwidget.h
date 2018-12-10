#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

#include "QVTKWidget.h"
#include <vtkRenderWindow.h>

//�Ƿ��ظ����壿
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
class VTKWidget : public QVTKWidget
{
	Q_OBJECT
public:
	VTKWidget(QWidget* parent = NULL);
	~VTKWidget();

	//���ź�
	void updateViewer(int viewPort, QVector<PointCloudT::Ptr> clouds);
	void updateViewer(int viewPort, PointCloudT::Ptr cloud);
	void addViewer(int viewPort, PointCloudT::Ptr cloud, QString id);
	void removeViewPortPointClouds(int viewPort);
signals:

private:
	int viewPort1, viewPort2;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;

};