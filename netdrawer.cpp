#include "netdrawer.h"

NetDrawer::NetDrawer()
{
	m_lineWidth = 5.0;
	m_pointSize = 1.0;
}

bool NetDrawer::updateData()
{
	QVector3D color = { 0,0,0 };
	int xMax = 180, xMin = -180;
	int yMax = 100, yMin = -100;
	int step = 20;

	for (int i = 0; i <= (xMax-xMin) / step; i++) {
		m_lines.append({ QVector3D(i*step + xMin,yMin,0), color, m_vectorNaN });
		m_lines.append({ QVector3D(i * step + xMin,yMax,0), color, m_vectorNaN });
	}
	for (int i = 0; i <= (yMax-yMin) / step; i++) {
		m_lines.append({ QVector3D(xMin,i * step + yMin,0), color, m_vectorNaN });
		m_lines.append({ QVector3D(xMax,i * step + yMin,0), color, m_vectorNaN });
	}

	return true;
}
