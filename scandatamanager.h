#pragma once
#include <iostream>

//point cloud library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <QObject>
#include <QString>
#include <QVector>
#include <QMap>
#include <QList>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//将点云名称映射到点云
class MapSrcToPointCloud
{
public:
	MapSrcToPointCloud();

	bool isEmpty() { return m_mapSrcToPointCloud.isEmpty(); }
	void clear() { m_mapSrcToPointCloud.clear(); }

	/*映射点云名称到点云，返回处理后的文件名*/
	QString mapSrcToPointCloud(const QString& srcName, PointCloudT::Ptr pointCloud);
	/*更新当前key对应的点云*/
	void updateSrcToPointCloud(const QString& srcName, PointCloudT::Ptr pointCloud);
	/*获取当前key对应的点云*/
	PointCloudT::Ptr getPointCloudFromSrc(const QString& srcName);
private:
	QMap<QString, PointCloudT::Ptr> m_mapSrcToPointCloud;
};

class ScandataManager : public QObject
{
	Q_OBJECT
public:
	ScandataManager();
	~ScandataManager();

	bool isEmpty();

	//槽
	void scandataToPoint(QVector<double> valueX, QVector<double> valueY, QVector<double> valueZ, QVector<unsigned short> intensity, unsigned int resolution);
	void stlLayerToPoint(QVector<double> valueX, QVector<double> valueY, double z);
	void openPcdFile(const QString& path);
	void savePcdFile(QString path);
	void clear();
	/*更新当前选中点云列表*/
	void onUpdateCurrentPointCloud(const QStringList& currentPointCloud);
	//槽：点云处理
	void filterPointCloud(int meanK, double thresh);
	void sacPointCloud(int maxIterations, double thresh);
	void icpPointCloud(int maxIterations, double eucliEpsilon);
	void findPointCloudBoundary(int maxSacIterations, double sacThresh, int normKSearch, int boundKSearch);

	//点云
	MapSrcToPointCloud m_mapSrcToPointCloud;
	QMap<QString,PointCloudT::Ptr> m_currentSelectedPointClouds;

signals:
	void drawPointClouds();
	void updateStatus(const QString&);
	//清空点云列表
	void signalClearPointCloudList();
	//更新点云列表
	void signalAddPointCloudList(const QString&);
	//显示当前选择的点云
	void repaintPointCloudViewer(int viewPort, const QMap<QString, PointCloudT::Ptr>& clouds);
	//更新src对应的点云
	void updatePointCloudViewer(int viewPort, PointCloudT::Ptr cloud, const QString& srcName);
	//删除viewer中显示的点云
	void removeViewPortPointCloud(int viewPort);	//0为ALL，1为左半视图，2为右半视图。
};

