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

	void interSect(double z, QSharedPointer<Triangle> spSurface);		//�������㷨��ʼ���������һ�����ߺ͵�һ������
	QVector3D calcuInterSectPoint(double z, QSharedPointer<Edge> spEdge);
	QVector3D calcuScanLineSectPoint(double scanLineY, QSharedPointer<LayerEdge> spLayerEdge);
	void judgeFaceType(double z, QSharedPointer<Triangle> spSurface);	//�ж���Ƭ����
	void judgeOtherLine(double z, QSharedPointer<Triangle> spSurface);	//�ж���һ���ཻ��
	double getZMin(QSharedPointer<Triangle> spSurface);					//������Ƭ����СZ����ֵ
	double getZMax(QSharedPointer<Triangle> spSurface);
	double getZMid(QSharedPointer<Triangle> spSurface);					//�����м�Ķ���Zֵ

	//void linshiyongyong(double _z);

	void updateColor() override;
	//��
	void drawPolyLine();
	void convertLayertoPointCloud(double layer);
protected:
	bool updateData() override;

signals:
	void stlLayerToPointCloud(QVector<double> valueX, QVector<double> valueY, double z);
private:
	QVector<QSharedPointer<Edge>> m_sliceEdges;	//�洢�ཻ��
	QVector<QSharedPointer<Layer>> m_layers;	//�洢Ƭ��
	QVector<QSharedPointer<Triangle>> m_sliceTris;	//����һ��stlmanager�е�������Ƭ����Ϊ��ʼ����
	double m_beginLayer;
	double m_layerHeight;
	STLManager* m_stlManager;
	bool m_sliced = false;
};