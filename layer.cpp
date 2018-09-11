#include "layer.h"

PointTri::PointTri()
{
}

PointTri::PointTri(const QVector3D & _position)
{
	position = _position;
}

PointTri::PointTri(const PointTri & _pointTri, QSharedPointer<Triangle> _tri)
{
	position = _pointTri.position;
	tri = _tri;
}

bool PointTri::operator^=(const PointTri & rhs) const
{
	QVector3D vec = position - rhs.position;
	double lengthSquar = vec.lengthSquared();
	if (lengthSquar <= 0.0000000001)
		return true;
	return false;
}

Line::Line()
{
	length = 0.0;
}

Line::Line(QSharedPointer<PointTri> _spHead, QSharedPointer<PointTri> _spEnd)
{
	spHead = _spHead;
	spEnd = _spEnd;
	length = getLength();
}

Line::Line(PointTri headPoint, PointTri endPoint)
{
	spHead = QSharedPointer<PointTri>(new PointTri(headPoint));
	spEnd = QSharedPointer<PointTri>(new PointTri(endPoint));
	length = getLength();
	a = headPoint;
	b = endPoint;
}

Line::Line(Edge e)
{
	a.position = e.spV1->position;
	b.position = e.spV2->position;
}

double Line::getLength()
{
	return (spHead->position - spEnd->position).length();
}

Plane::Plane()
{
}

Plane::Plane(PointTri _a, PointTri _b, PointTri _c)
{
	a = _a;
	b = _b;
	c = _c;
}

Plane::Plane(QSharedPointer<Triangle> _tri)
{
	a = _tri->spV1->position;
	b = _tri->spV2->position;
	c = _tri->spV3->position;
	tri = _tri;
}

Plane::Plane(QSharedPointer<PointTri> _spPt, QVector3D _normal)		//点法式求平面三点
{
	pt = *_spPt;
	normal = _normal;
	a = *_spPt;

	double v_x, v_y, v_z;
	const double Precision = 0.0001;
	v_x = normal.x();
	v_y = normal.y();
	v_z = normal.z();
	if (fabs(v_y) < Precision && fabs(v_z) < Precision)
	{
		b = QVector3D(pt.position.x(), pt.position.y() + 1, pt.position.z());
		c = QVector3D(pt.position.x(), pt.position.y(), pt.position.z() + 1);
	}
	if (fabs(v_y) < Precision && fabs(v_z) > Precision)
	{
		b = QVector3D(pt.position.x(), pt.position.y() + 1, pt.position.z());
		c = QVector3D(pt.position.x() + 1, pt.position.y(), pt.position.z() - (v_x / v_z));
	}
	if (fabs(v_y) > Precision && fabs(v_z) < Precision)
	{
		b = QVector3D(pt.position.x(), pt.position.y(), pt.position.z() + 1);
		c = QVector3D(pt.position.x() + 1, pt.position.y() - (v_x / v_y), pt.position.z());
	}
	if (fabs(v_y) > Precision && fabs(v_z) > Precision)
	{
		b = QVector3D(pt.position.x(), pt.position.y() + 1, pt.position.z() - (v_y / v_z));
		c = QVector3D(pt.position.x(), pt.position.y() - (v_z / v_y), pt.position.z() + 1);
	}
}

PolyLine::PolyLine()
{
}

Layer::Layer()
{
}
