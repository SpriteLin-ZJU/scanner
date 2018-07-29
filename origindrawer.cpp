#include "origindrawer.h"

OriginDrawer::OriginDrawer()
{
	m_lineWidth = 1.0;
	m_pointSize = 1.0;
}

bool OriginDrawer::updateData()
{
	m_lines = {
		{ QVector3D(0, 0, 0), QVector3D(1.0, 0.0, 0.0), m_vectorNaN },
		{ QVector3D(17, 0, 0), QVector3D(1.0, 0.0, 0.0), m_vectorNaN },
		{ QVector3D(20, 0, 0), QVector3D(1.0, 0.0, 0.0), m_vectorNaN },
		{ QVector3D(16, 3, 0), QVector3D(1.0, 0.0, 0.0), m_vectorNaN },
		{ QVector3D(16, 3, 0), QVector3D(1.0, 0.0, 0.0), m_vectorNaN },
		{ QVector3D(16, -3, 0), QVector3D(1.0, 0.0, 0.0), m_vectorNaN },
		{ QVector3D(16, -3, 0), QVector3D(1.0, 0.0, 0.0), m_vectorNaN },
		{ QVector3D(20, 0, 0), QVector3D(1.0, 0.0, 0.0), m_vectorNaN },

		// Y-axis
		{ QVector3D(0, 0, 0), QVector3D(0.0, 1.0, 0.0), m_vectorNaN },
		{ QVector3D(0, 17, 0), QVector3D(0.0, 1.0, 0.0), m_vectorNaN },
		{ QVector3D(0, 20, 0), QVector3D(0.0, 1.0, 0.0), m_vectorNaN },
		{ QVector3D(3, 16, 0), QVector3D(0.0, 1.0, 0.0), m_vectorNaN },
		{ QVector3D(3, 16, 0), QVector3D(0.0, 1.0, 0.0), m_vectorNaN },
		{ QVector3D(-3, 16, 0), QVector3D(0.0, 1.0, 0.0), m_vectorNaN },
		{ QVector3D(-3, 16, 0), QVector3D(0.0, 1.0, 0.0), m_vectorNaN },
		{ QVector3D(0, 20, 0), QVector3D(0.0, 1.0, 0.0), m_vectorNaN },

		// Z-axis
		{ QVector3D(0, 0, 0), QVector3D(0.0, 0.0, 1.0), m_vectorNaN },
		{ QVector3D(0, 0, 17), QVector3D(0.0, 0.0, 1.0), m_vectorNaN },
		{ QVector3D(0, 0, 20), QVector3D(0.0, 0.0, 1.0), m_vectorNaN },
		{ QVector3D(3, 0, 16), QVector3D(0.0, 0.0, 1.0), m_vectorNaN },
		{ QVector3D(3, 0, 16), QVector3D(0.0, 0.0, 1.0), m_vectorNaN },
		{ QVector3D(-3, 0, 16), QVector3D(0.0, 0.0, 1.0), m_vectorNaN },
		{ QVector3D(-3, 0, 16), QVector3D(0.0, 0.0, 1.0), m_vectorNaN },
		{ QVector3D(0, 0, 20), QVector3D(0.0, 0.0, 1.0), m_vectorNaN },

		// 2x2 rect
		{ QVector3D(1, 1, 0), QVector3D(1.0, 0.0, 0.0), m_vectorNaN },
		{ QVector3D(-1, 1, 0), QVector3D(1.0, 0.0, 0.0), m_vectorNaN },
		{ QVector3D(-1, 1, 0), QVector3D(1.0, 0.0, 0.0), m_vectorNaN },
		{ QVector3D(-1, -1, 0), QVector3D(1.0, 0.0, 0.0), m_vectorNaN },
		{ QVector3D(-1, -1, 0), QVector3D(1.0, 0.0, 0.0), m_vectorNaN },
		{ QVector3D(1, -1, 0), QVector3D(1.0, 0.0, 0.0), m_vectorNaN },
		{ QVector3D(1, -1, 0), QVector3D(1.0, 0.0, 0.0), m_vectorNaN },
		{ QVector3D(1, 1, 0), QVector3D(1.0, 0.0, 0.0), m_vectorNaN }
	};
	return true;
}
