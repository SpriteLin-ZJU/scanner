#pragma once

#include "shaderdrawable.h"

extern std::vector<double>vdValueX;
extern std::vector<double>vdValueZ;

class ScannerDrawer : public QObject, public ShaderDrawable
{
	Q_OBJECT
public:
	ScannerDrawer();
	void update(unsigned int resolution);
	void setScanFeedrate(int feedrate);
signals:
	void updateGraph();
protected:
	bool updateData();
private:
	unsigned int m_resolution=0;
	unsigned int m_profileCount=0;
	unsigned int m_scanFeedrate = 1500;	//与scanRate一起确定轮廓的y坐标。
	double m_stepY = 0.0;
};