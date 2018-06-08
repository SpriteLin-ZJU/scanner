#pragma once
#include "shaderdrawable.h"

class PlatformDrawer :public ShaderDrawable
{
public:
	PlatformDrawer();
protected:
	bool updateData();
};