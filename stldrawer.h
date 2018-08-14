#pragma once
#include "shaderdrawable.h"
#include "stlmanager.h"


class STLDrawer :public QObject, public ShaderDrawable
{
	Q_OBJECT
public:
	STLDrawer();
	~STLDrawer();

	void setSTLManager(STLManager* manager);
	void drawSTLFile();
signals:
	void updateGraph();
protected:
	bool updateData();
private:
	STLManager * m_stlManager;
};