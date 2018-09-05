#include "geometry.h"

Vertex::Vertex()
{
}

Vertex::Vertex(const Vertex & vertex)
{
	position = vertex.position;
}

Vertex::Vertex(QVector3D point)
{
	position = point;
}

bool Vertex::operator==(const Vertex & pt)
{
	if ((position.x() == pt.position.x())
		&& (position.y() == pt.position.y())
		&& (position.z() == pt.position.z()))
		return true;
	return false;
}

bool Vertex::operator<(const Vertex & pt) const
{
	if ((position.x() < pt.position.x())
		|| (position.x() == pt.position.x() && position.y() < pt.position.y())
		|| (position.x() == pt.position.x() && position.y() == pt.position.y() && position.z() < pt.position.z()))
		return true;
	return false;
}

bool Vertex::operator>(const Vertex & pt) const
{
	if (position == pt.position)
		return false;
	return !((*this) < pt);
}

QVector3D Vertex::operator-(const Vertex& pt) const
{
	return position - pt.position;
}

Edge::Edge()
{
	bUse = false;
	num = 0;
}

Edge::Edge(QSharedPointer<Vertex> _spV1, QSharedPointer<Vertex> _spV2)
{
	spV1 = _spV1;
	spV2 = _spV2;
	bUse = false;
	num = 0;
}

bool Edge::isOpposite(const QSharedPointer<Edge> _spEdge) const
{
	//两者的起点终点相反，即为伙伴半边
	if ((spV1->position == _spEdge->spV2->position) && (spV2->position == _spEdge->spV1->position))
		return true;
	return false;
}

double Edge::getLength() const
{
	return (spV1->position - spV2->position).length();
}

double Edge::getLength(const QSharedPointer<Edge> _spEdge) const
{
	return (_spEdge->spV1->position - _spEdge->spV2->position).length();
}

Triangle::Triangle()
{
	bUse = false;
	faceType = NoSectSurface;
}

Triangle::Triangle(QSharedPointer<Vertex> _spV1, QSharedPointer<Vertex> _spV2, QSharedPointer<Vertex> _spV3)
{
	spV1 = _spV1;
	spV2 = _spV2;
	spV3 = _spV3;

	bUse = false;
	faceType = NoSectSurface;
}

QSharedPointer<Triangle> Triangle::getNbTri1() const
{
	return spEdge1->spEdgeAdja.lock() ? spEdge1->spEdgeAdja.lock()->spTri.lock() : NULL;
}

QSharedPointer<Triangle> Triangle::getNbTri2() const
{
	return spEdge2->spEdgeAdja.lock() ? spEdge2->spEdgeAdja.lock()->spTri.lock() : NULL;
}

QSharedPointer<Triangle> Triangle::getNbTri3() const
{
	return spEdge3->spEdgeAdja.lock() ? spEdge3->spEdgeAdja.lock()->spTri.lock() : NULL;
}

QSharedPointer<QVector3D> Triangle::getNormal()
{
	QVector3D normal = QVector3D::normal(*spV2 - *spV1, *spV3 - *spV2);
	spNormal = QSharedPointer<QVector3D>(new QVector3D(normal));
	return spNormal;
}

void Triangle::sortVertex()
{
	spVMin = spV1;
	spVMid = spV2;
	spVMax = spV3;
	//对vmax,vmid,vmin进行排序
	if (spVMin->position.z() > spVMid->position.z())
		qSwap(spVMin, spVMid);
	if (spVMid->position.z() > spVMax->position.z())
		qSwap(spVMid, spVMax);
	if (spVMin->position.z() > spVMid->position.z())
		qSwap(spVMin, spVMid);
}

EdgeHull::EdgeHull()
{
	bUse = false;
}

bool EdgeHull::isOpposite(const EdgeHull & edgeHull)
{
	if ((*spEdge->spV1 == *edgeHull.spEdge->spV2) && (*spEdge->spV2 == *edgeHull.spEdge->spV1))
		return true;
	return false;
}

bool EdgeHull::isOppositeApp(const EdgeHull & edgeHull)
{
	const double precis = 0.01;
	double x1, y1, z1, x2, y2, z2;
	double _x1, _y1, _z1, _x2, _y2, _z2;
	double dx1, dy1, dz1, dx2, dy2, dz2;
	x1 = spEdge->spV1->position.x();
	y1 = spEdge->spV1->position.y();
	z1 = spEdge->spV1->position.z();
	x2 = spEdge->spV2->position.x();
	y2 = spEdge->spV2->position.y();
	z2 = spEdge->spV2->position.z();

	_x1 = edgeHull.spEdge->spV1->position.x();
	_y1 = edgeHull.spEdge->spV1->position.y();
	_z1 = edgeHull.spEdge->spV1->position.z();
	_x2 = edgeHull.spEdge->spV2->position.x();
	_y2 = edgeHull.spEdge->spV2->position.y();
	_z2 = edgeHull.spEdge->spV2->position.z();

	dx1 = fabs(x1 - _x2);
	dy1 = fabs(y1 - _y2);
	dz1 = fabs(z1 - _z2);
	dx2 = fabs(x2 - _x1);
	dy2 = fabs(y2 - _y1);
	dz2 = fabs(z2 - _z1);

	if (dx1 < precis && dy1 < precis && dz1 < precis && dx2 < precis && dy2 < precis && dz2 < precis)//即二者的起点终点刚好相反
		return true;
	return false;
}

bool EdgeHull::operator<(const EdgeHull & edgeHull) const
{
	Vertex v1 = *spEdge->spV1;
	Vertex v2 = *spEdge->spV2;
	if (v2 < v1)
		qSwap(v1, v2);		//本质上还是使用了std::swap，使用move来交换数据成员
	Vertex v1_ = *edgeHull.spEdge->spV1;
	Vertex v2_ = *edgeHull.spEdge->spV2;
	if (v2_ < v1_)
		qSwap(v1_, v2_);

	//判断边大小，用于排序查找相邻边
	if ((v1 < v1_) || ((v1 == v1_) && (v2 < v2_)))
		return true;
	return false;
}
