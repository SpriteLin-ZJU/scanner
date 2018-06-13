#pragma once

#include "shaderdrawable.h"

class GcodeManager;

class GcodeDrawer :public QObject, public ShaderDrawable
{
	Q_OBJECT
public:
	GcodeDrawer();
	~GcodeDrawer();

	void setGcodeManager(GcodeManager* manager);
	void drawSingleGcode();
signals:
	void updateGraph();
protected:
	bool updateData();
private:
	GcodeManager* m_gcodeManager;
	
	QVector3D m_lastPoint;
	QVector3D m_targetPoint;
};