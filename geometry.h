#pragma once
#include <QSharedPointer>
#include <QVector>
#include <QVector3D>
#include <QWeakPointer>
#include <QDeBug>

struct Vertex;
struct Edge;
struct Triangle;
struct EdgeHull;

struct Vertex
{
	Vertex();
	Vertex(const Vertex& vertex);
	Vertex(QVector3D point);

	bool operator==(const Vertex& pt);
	bool operator<(const Vertex& pt) const;
	bool operator>(const Vertex& pt) const;

	QVector3D operator-(const Vertex& pt) const;
	QVector3D position;
	QVector<QSharedPointer<Edge>> edgeContainVertex;	//记录包含该端点的所有半边（注意是半边不是边），这里是否需要使用弱引用？？？
};

struct Edge	//半边数据结构
{
	Edge();
	Edge(QSharedPointer<Vertex> _spV1, QSharedPointer<Vertex> _spV2);

	bool isOpposite(const QSharedPointer<Edge> _spEdge) const;		//判断是否为伙伴半边
	double getLength() const;
	double getLength(const QSharedPointer<Edge> _spEdge) const;

	///////////////////
	QSharedPointer<Vertex> spV1, spV2;
	QWeakPointer<Triangle> spTri;			//边所属的三角面片,为避免循环引用，故使用弱引用
	QWeakPointer<Edge> spEdgePrev, spEdgeNext, spEdgeAdja;
	int num;
	bool bUse;
};

struct Triangle
{
	Triangle();
	Triangle(QSharedPointer<Vertex> _spV1, QSharedPointer<Vertex> _spV2, QSharedPointer<Vertex> _spV3);
	QSharedPointer<Triangle> getNbTri1() const;			//返回指向与边1相邻的三角面片的指针
	QSharedPointer<Triangle> getNbTri2() const;
	QSharedPointer<Triangle> getNbTri3() const;
	QSharedPointer<QVector3D> getNormal();

	///////////////
	QSharedPointer<Vertex> spV1, spV2, spV3;		//逆时针排列
	QSharedPointer<Edge> spEdge1, spEdge2, spEdge3;	//半边
	QSharedPointer<QVector3D> spNormal;				//法向量
	QSharedPointer<Edge> spSelectIntersectLine;		//上一面片已经求交的相交线，用于寻找下一相交线
	QSharedPointer<Edge> spOtherIntersectLine;		//另一条相交线

	bool bUse;										//面片是否被使用
	int faceType;									//面片类型
};

struct EdgeHull
{
	EdgeHull();
	
	bool isOpposite(const EdgeHull& edgeHull);	//判断是否为伙伴半边
	bool isOppositeApp(const EdgeHull& edgeHull);	//处理一些裂缝，数值为0.01。合并为同一个点
	bool operator < (const EdgeHull& edgeHull) const;

	QSharedPointer<Edge> spEdge;
	bool bUse;
};
