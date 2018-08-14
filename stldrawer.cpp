#include "stldrawer.h"

STLDrawer::STLDrawer()
	:m_stlManager(NULL)
{
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

bool STLDrawer::updateData()
{
	m_triangles.clear();
	QVector3D color = { 0.2f,0.7f,0.3f };
	for (auto it = m_stlManager->pointBegin(); it != m_stlManager->pointEnd(); it++) {
		m_triangles.append({ it->position,color,it->normal });
	}
	return true;
}
