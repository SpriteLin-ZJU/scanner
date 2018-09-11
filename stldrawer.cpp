#include "stldrawer.h"
#include <QSettings>

STLDrawer::STLDrawer()
	:m_stlManager(NULL)
{
	updateColor();
}
STLDrawer::~STLDrawer()
{
	m_stlManager = NULL;
}

void STLDrawer::setSTLManager(STLManager * manager)
{
	m_stlManager = manager;
}

void STLDrawer::drawSTLFile()
{
	ShaderDrawable::update();
	showGraph();
	emit updateGraph();
}

void STLDrawer::updateColor()
{
	QSettings settings("ZJU", "scanner");
	m_color = { settings.value("stlR",0.2f).toFloat(),settings.value("stlG",0.7f).toFloat(),settings.value("stlB",0.3f).toFloat() };
	update();
}

bool STLDrawer::updateData()
{
	m_triangles.clear();
	//QVector3D color = { 0.2f,0.7f,0.3f };
	for (auto it = m_stlManager->pointBegin(); it != m_stlManager->pointEnd(); it++) {
		m_triangles.append({ it->position,m_color,it->normal });
	}
	return true;
}
