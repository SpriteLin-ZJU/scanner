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
	QVector<QSharedPointer<Edge>> edgeContainVertex;	//��¼�����ö˵�����а�ߣ�ע���ǰ�߲��Ǳߣ��������Ƿ���Ҫʹ�������ã�����
};

struct Edge	//������ݽṹ
{
	Edge();
	Edge(QSharedPointer<Vertex> _spV1, QSharedPointer<Vertex> _spV2);

	bool isOpposite(const QSharedPointer<Edge> _spEdge) const;		//�ж��Ƿ�Ϊ�����
	double getLength() const;
	double getLength(const QSharedPointer<Edge> _spEdge) const;

	///////////////////
	QSharedPointer<Vertex> spV1, spV2;
	QWeakPointer<Triangle> spTri;			//��������������Ƭ,Ϊ����ѭ�����ã���ʹ��������
	QWeakPointer<Edge> spEdgePrev, spEdgeNext, spEdgeAdja;
	int num;
	bool bUse;
};

struct Triangle
{
	Triangle();
	Triangle(QSharedPointer<Vertex> _spV1, QSharedPointer<Vertex> _spV2, QSharedPointer<Vertex> _spV3);
	QSharedPointer<Triangle> getNbTri1() const;			//����ָ�����1���ڵ�������Ƭ��ָ��
	QSharedPointer<Triangle> getNbTri2() const;
	QSharedPointer<Triangle> getNbTri3() const;
	QSharedPointer<QVector3D> getNormal();

	///////////////
	QSharedPointer<Vertex> spV1, spV2, spV3;		//��ʱ������
	QSharedPointer<Edge> spEdge1, spEdge2, spEdge3;	//���
	QSharedPointer<QVector3D> spNormal;				//������
	QSharedPointer<Edge> spSelectIntersectLine;		//��һ��Ƭ�Ѿ��󽻵��ཻ�ߣ�����Ѱ����һ�ཻ��
	QSharedPointer<Edge> spOtherIntersectLine;		//��һ���ཻ��

	bool bUse;										//��Ƭ�Ƿ�ʹ��
	int faceType;									//��Ƭ����
};

struct EdgeHull
{
	EdgeHull();
	
	bool isOpposite(const EdgeHull& edgeHull);	//�ж��Ƿ�Ϊ�����
	bool isOppositeApp(const EdgeHull& edgeHull);	//����һЩ�ѷ죬��ֵΪ0.01���ϲ�Ϊͬһ����
	bool operator < (const EdgeHull& edgeHull) const;

	QSharedPointer<Edge> spEdge;
	bool bUse;
};
