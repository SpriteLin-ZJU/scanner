#include "platformdrawer.h"

PlatformDrawer::PlatformDrawer()
{
}

bool PlatformDrawer::updateData()
{
	QVector3D color = { 0.8f, 0.8f, 0.8f };
	m_triangles = {
		{QVector3D(-180.0,-100.0,-0.1),color},
		{QVector3D(180.0,-100.0,-0.1),color},
		{QVector3D(-180.0,100.0,-0.1),color},
		{QVector3D(180.0,100.0,-0.1),color}
	};
	return true;
}
