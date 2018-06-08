#pragma once

#include "shaderdrawable.h"

class OriginDrawer : public ShaderDrawable
{
public:
	OriginDrawer();

protected:
	bool updateData();
};