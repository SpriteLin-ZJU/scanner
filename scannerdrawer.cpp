#include "scannerdrawer.h"

#include <QSettings>

ScannerDrawer::ScannerDrawer()
{
	m_pointSize = 2.0;
}

void ScannerDrawer::update(unsigned int resolution)
{
	m_resolution = resolution;
	ShaderDrawable::update();
	emit updateGraph();
}

void ScannerDrawer::setScanFeedrate(int feedrate)
{
	m_scanFeedrate = feedrate;
}

bool ScannerDrawer::updateData()
{
	m_points.clear();
	QVector3D color = { 1.0f,0.3f,0.2f };

	if (m_resolution == 0)
		return true;
	m_profileCount = vdValueX.size() / m_resolution;

	QSettings settings("ZJU", "scanner");
	if (settings.contains("shutterTime")) {
		unsigned int scanRate = settings.value("scanRate").toUInt();
		m_stepY = m_scanFeedrate / (scanRate * 60.0);
	}
	for (int i = 0; i < m_profileCount; i++) {
		for (int j = 0; j < m_resolution; j++) {
			m_points.append({ QVector3D(vdValueX[i*m_resolution + j],m_stepY*i,vdValueZ[i*m_resolution + j]),color, m_vectorNaN });
		}
	}

	return true;
}
