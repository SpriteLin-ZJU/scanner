#include "scannerdrawer.h"

ScannerDrawer::ScannerDrawer() :
	QObject()	//实际上会自动调用父类的无参构造函数，因此不写也没事
{
	m_pointSize = 2.0;
}

void ScannerDrawer::update(unsigned int resolution)
{
	m_resolution = resolution;
	ShaderDrawable::update();
	emit updateGraph();
}

bool ScannerDrawer::updateData()
{
	m_points.clear();
	QVector3D color = { 1.0,0.0,0.0 };

	if (m_resolution == 0)
		return true;
	m_profileCount = vdValueX.size() / m_resolution;

	for (int i = 0; i < m_profileCount; i++) {
		for (int j = 0; j < m_resolution; j++) {
			m_points.append({ QVector3D(vdValueX[i*m_resolution + j],(double)i*0.5,vdValueZ[i*m_resolution + j]),color });
		}
	}

	return true;
}
