#pragma once
#include <QVector>
#include <QVector3D>
#include <QSharedPointer>
#include <QWeakPointer>
#include "geometry.h"

enum PointTypes { InvalidPoint = 0, ConvexPoint, ConcavePoint, FlatPoint };
struct Line;
struct PointTri
{
	PointTri();
	PointTri(const QVector3D& _position);
	PointTri(const PointTri& _pointTri, QSharedPointer<Triangle> _tri = NULL);

	bool operator^=(const PointTri& rhs) const;

	//////////
	QVector3D vertexToLine;				//����������������
	QSharedPointer<Triangle> tri;		//�������ĸ�������Ƭ
	QSharedPointer<QVector3D> normal;
	QSharedPointer<Vertex> vertexToPointTri;	//�����ĸ�����ת������
	QSharedPointer<Edge> pointTriToEdge;		//��¼�������������
	

	bool bUse=false;
	bool isoRestrition=false;
	QVector3D position;
	PointTypes PointType = InvalidPoint;
};

struct Line
{
	Line();
	Line(QSharedPointer<PointTri> _spHead, QSharedPointer<PointTri> _spEnd);
	Line(PointTri headPoint, PointTri endPoint);
	Line(Edge e);

	double getLength();

	/////////
	QSharedPointer<PointTri> spHead;
	QSharedPointer<PointTri> spEnd;
	QSharedPointer<Triangle> tri;		//�߶����ڵ�������Ƭ
	PointTri a, b;
	double length;
};

struct Plane
{
	Plane();
	Plane(PointTri _a, PointTri _b, PointTri _c);
	Plane(QSharedPointer<Triangle> _tri);
	Plane(QSharedPointer<PointTri> _spPt, QVector3D _normal);

	///////
	PointTri a;
	PointTri b;
	PointTri c;

	PointTri pt;
	QVector3D normal;
	QSharedPointer<Triangle> tri;
};

struct PolyLine
{
	PolyLine();
	QVector<QSharedPointer<PointTri>> m_linkPoints;
};

struct Layer
{
	Layer();

	/////
	double z;
	QVector<QSharedPointer<PolyLine>> m_polyLines;
};

struct LayerEdge
{
	LayerEdge();
	LayerEdge(QSharedPointer<PointTri> _spHead, QSharedPointer<PointTri> _spEnd);
	QSharedPointer<PointTri> spLEHead;
	QSharedPointer<PointTri> spLEEnd;
	QSharedPointer<PointTri> spVMax, spVMin;
	void sortVertex();
};