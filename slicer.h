#pragma once
#include "stlmanager.h"
#include "shaderdrawable.h"
#include "layer.h"
#include "geometry.h"
#include <QVector>
#include <QVector3D>
#include <QSharedPointer>
#include <QWeakPointer>

const double perturbationVal = 0.00001;
class Slicer :public ShaderDrawable
{
	Q_OBJECT
public:
	Slicer();
	~Slicer();

	void setSTLManager(STLManager* manager);
	void slice();
	void getPolyLinePoints(double z);
	void clear();

	void interSect(double z, QSharedPointer<Triangle> spSurface);		//轮廓求交算法初始化，求出第一条交线和第一个交点
	QVector3D calcuInterSectPoint(double z, QSharedPointer<Edge> spEdge);
	QVector3D calcuScanLineSectPoint(double scanLineY, QSharedPointer<LayerEdge> spLayerEdge);
	void judgeFaceType(double z, QSharedPointer<Triangle> spSurface);	//判断面片类型
	void judgeOtherLine(double z, QSharedPointer<Triangle> spSurface);	//判断另一条相交线
	double getZMin(QSharedPointer<Triangle> spSurface);					//返回面片的最小Z坐标值
	double getZMax(QSharedPointer<Triangle> spSurface);
	double getZMid(QSharedPointer<Triangle> spSurface);					//处于中间的顶点Z值

	//void linshiyongyong(double _z);

	void updateColor() override;
	//槽
	void drawPolyLine();
	void convertLayertoPointCloud(double layer);
protected:
	bool updateData() override;

signals:
	void stlLayerToPointCloud(QVector<double> valueX, QVector<double> valueY, double z);
private:
	QVector<QSharedPointer<Edge>> m_sliceEdges;	//存储相交边
	QVector<QSharedPointer<Layer>> m_layers;	//存储片层
	QVector<QSharedPointer<Triangle>> m_sliceTris;	//保存一份stlmanager中的三角面片，作为初始数据
	double m_beginLayer;
	double m_layerHeight;
	STLManager* m_stlManager;
	bool m_sliced = false;
};