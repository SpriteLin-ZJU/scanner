#pragma once
#include "shaderdrawable.h"

class NetDrawer :public ShaderDrawable
{
public:
	NetDrawer();
protected:
	bool updateData();
};