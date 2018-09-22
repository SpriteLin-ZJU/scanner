#pragma once
#include <QObject>
#include <QVector3D>
#include <QVector>
#include <QSharedPointer>
#include <QFile>
#include "geometry.h"

struct PointData
{
	QVector3D position;
	QVector3D normal;
};

class STLManager :public QObject
{
	Q_OBJECT
public:
	STLManager();
	~STLManager();
	void clear();
	void addLine(const QString& string);
	int pointCount();
	void readBinarySTL(QFile& binaryFile);
	void readAsciiSTL(QFile& asciiFile);
	float round(float val, int n);
	QSharedPointer<Vertex> searchPtInVertices(const QVector3D& pt);
	//返回m_STLpoint迭代器
	QVector<PointData>::iterator pointBegin();
	QVector<PointData>::iterator pointEnd();
	
	//槽信号
	void setXMove(double val);
	void setYMove(double val);
	void setZMove(double val);
	void setMoveVal(QVector3D position);
	void setRotateVal(QVector3D rotate);
	void setScaleVal(QVector3D scale);
	QVector3D getMoveValue();
	QVector3D getRotateValue();
	QVector3D getScaleValue();
	
	//更新STL数据点
	void updateSTL();
	//居中STL
	void centreSTL();

	//建立拓扑关系
	void STLTopologize();
	void findExtreme(double ext[]);
	void findOrigianlExtrem(double ext[]);
	//拓扑关系
	QVector<QSharedPointer<Vertex>> m_vertices;		//顶点:点从小到大排列，没有重复点
	QVector<QSharedPointer<Edge>> m_edges;			//边:STL顺序排列
	QVector<QSharedPointer<Triangle>> m_tris;		//面:STL顺序排列
	QVector<QSharedPointer<QVector3D>> m_normals;	//法向量:STL顺序排列

signals:
	void drawSTLPoint();
private:
	QList<QString> m_fileSTL;
	QVector<PointData> m_currentSTLPoint;
	QVector<PointData> m_STLPoint;

	double m_xMove = 0.0, m_yMove = 0.0, m_zMove = 0.0, m_xMoveLast = 0.0, m_yMoveLast = 0.0, m_zMoveLast = 0.0;
	double m_xRot = 0.0, m_yRot = 0.0, m_zRot = 0.0, m_xRotLast = 0.0, m_yRotLast = 0.0, m_zRotLast = 0.0;
	double m_xScale = 100.0, m_yScale = 100.0, m_zScale = 100.0, m_xScaleLast = 100.0, m_yScaleLast = 100.0, m_zScaleLast = 100.0;
};