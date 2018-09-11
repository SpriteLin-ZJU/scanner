#pragma once
#include "shaderdrawable.h"
#include "stlmanager.h"


class STLDrawer :public ShaderDrawable
{
	Q_OBJECT
public:
	STLDrawer();
	~STLDrawer();

	void setSTLManager(STLManager* manager);
	void drawSTLFile();
	void updateColor() override;
protected:
	bool updateData();
private:
	STLManager * m_stlManager;
};