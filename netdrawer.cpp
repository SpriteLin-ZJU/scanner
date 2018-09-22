#include "netdrawer.h"

NetDrawer::NetDrawer()
{
	m_lineWidth = 5.0;
	m_pointSize = 1.0;
}

bool NetDrawer::updateData()
{
	QVector3D color = { 0,0,0 };
	int xMax = 125, xMin = -125;
	int yMax = 50, yMin = -50;
	int step = 10;

	for (int i = 0; i <= (xMax-xMin) / step; i++) {
		m_lines.append({ QVector3D(i*step + xMin,yMin,0), color, m_vectorNaN });
		m_lines.append({ QVector3D(i * step + xMin,yMax,0), color, m_vectorNaN });
	}
	for (int i = 0; i <= (yMax-yMin) / step; i++) {
		m_lines.append({ QVector3D(xMin,i * step + yMin,0), color, m_vectorNaN });
		m_lines.append({ QVector3D(xMax,i * step + yMin,0), color, m_vectorNaN });
	}

	QVector3D colorPlat = { 0.75f, 0.8f, 0.75f };
	m_triangles = {
		{ QVector3D(xMin,yMin,-0.1),colorPlat, m_vectorNaN },
		{ QVector3D(xMax,yMin,-0.1),colorPlat, m_vectorNaN },
		{ QVector3D(xMin,yMax,-0.1),colorPlat, m_vectorNaN },
		{ QVector3D(xMax,yMin,-0.1),colorPlat, m_vectorNaN },
		{ QVector3D(xMin,yMax,-0.1),colorPlat, m_vectorNaN },
		{ QVector3D(xMax,yMax,-0.1),colorPlat, m_vectorNaN }
	};
	return true;
}
