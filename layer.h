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
	QVector3D vertexToLine;				//点属于哪条轮廓线
	QSharedPointer<Triangle> tri;		//点属于哪个三角面片
	QSharedPointer<QVector3D> normal;
	QSharedPointer<Vertex> vertexToPointTri;	//点由哪个顶点转化而来
	QSharedPointer<Edge> pointTriToEdge;		//记录点属于哪条半边
	

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
	QSharedPointer<Triangle> tri;		//线段属于的三角面片
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