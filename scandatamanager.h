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

//����������ӳ�䵽����
class MapSrcToPointCloud
{
public:
	MapSrcToPointCloud();

	bool isEmpty() { return m_mapSrcToPointCloud.isEmpty(); }
	void clear() { m_mapSrcToPointCloud.clear(); }

	/*ӳ��������Ƶ����ƣ����ش������ļ���*/
	QString mapSrcToPointCloud(const QString& srcName, PointCloudT::Ptr pointCloud);
	/*���µ�ǰkey��Ӧ�ĵ���*/
	void updateSrcToPointCloud(const QString& srcName, PointCloudT::Ptr pointCloud);
	/*��ȡ��ǰkey��Ӧ�ĵ���*/
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

	//��
	void scandataToPoint(QVector<double> valueX, QVector<double> valueY, QVector<double> valueZ, QVector<unsigned short> intensity, unsigned int resolution);
	void stlLayerToPoint(QVector<double> valueX, QVector<double> valueY, double z);
	void openPcdFile(const QString& path);
	void savePcdFile(QString path);
	void clear();
	/*���µ�ǰѡ�е����б�*/
	void onUpdateCurrentPointCloud(const QStringList& currentPointCloud);
	//�ۣ����ƴ���
	void filterPointCloud(int meanK, double thresh);
	void sacPointCloud(int maxIterations, double thresh);
	void icpPointCloud(int maxIterations, double eucliEpsilon);
	void findPointCloudBoundary(int maxSacIterations, double sacThresh, int normKSearch, int boundKSearch);

	//����
	MapSrcToPointCloud m_mapSrcToPointCloud;
	QMap<QString,PointCloudT::Ptr> m_currentSelectedPointClouds;

signals:
	void drawPointClouds();
	void updateStatus(const QString&);
	//��յ����б�
	void signalClearPointCloudList();
	//���µ����б�
	void signalAddPointCloudList(const QString&);
	//��ʾ��ǰѡ��ĵ���
	void repaintPointCloudViewer(int viewPort, const QMap<QString, PointCloudT::Ptr>& clouds);
	//����src��Ӧ�ĵ���
	void updatePointCloudViewer(int viewPort, PointCloudT::Ptr cloud, const QString& srcName);
	//ɾ��viewer����ʾ�ĵ���
	void removeViewPortPointCloud(int viewPort);	//0ΪALL��1Ϊ�����ͼ��2Ϊ�Ұ���ͼ��
};

