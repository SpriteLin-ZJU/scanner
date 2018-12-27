#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

#include "QVTKWidget.h"
#include <vtkRenderWindow.h>

//是否重复定义？
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
class VTKWidget : public QVTKWidget
{
	Q_OBJECT
public:
	VTKWidget(QWidget* parent = NULL);
	~VTKWidget();

	//槽信号
	void repaintPointCloudViewer(int viewPort, const QMap<QString, PointCloudT::Ptr>& clouds);
	void updatePointCloudViewer(int viewPort, PointCloudT::Ptr cloud, const QString& id);
	void removeViewPortPointClouds(int viewPort);
signals:

private:
	int viewPort1, viewPort2;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;

};