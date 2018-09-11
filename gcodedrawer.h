#pragma once

#include "shaderdrawable.h"

class GcodeManager;

class GcodeDrawer :public ShaderDrawable
{
	Q_OBJECT
public:
	GcodeDrawer();
	~GcodeDrawer();

	void setGcodeManager(GcodeManager* manager);
	void drawSingleGcode();
	void updateColor() override;
protected:
	bool updateData() override;
private:
	GcodeManager* m_gcodeManager;
	
	QVector3D m_lastPoint;
	QVector3D m_targetPoint;
};