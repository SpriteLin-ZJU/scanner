#pragma once
#include <QObject>
#include <QVector3D>
#include <QVector>

struct PointData
{
	QVector3D position;
	QVector3D normal;
};

class STLManager :public QObject
{
	Q_OBJECT
public:
	STLManager();
	~STLManager();
	void clear();
	void addLine(const QString& string);
	int pointCount();
	void fileToPoint();
	//·µ»Øm_STLpointµü´úÆ÷
	QVector<PointData>::iterator pointBegin();
	QVector<PointData>::iterator pointEnd();

private:
	QList<QString> m_fileSTL;
	QVector<PointData> m_STLPoint;
};