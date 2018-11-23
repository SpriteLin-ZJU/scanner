#pragma once
#include "shaderdrawable.h"

class ScandataManager;
class ScannerDrawer :public ShaderDrawable
{
	Q_OBJECT
public:
	ScannerDrawer();
	~ScannerDrawer();

	void setScandataManager(ScandataManager* manager);
	void drawScandataGL();
protected:
	bool updateData();
private:
	ScandataManager* m_scandataManager;
};