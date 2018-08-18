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
	//返回m_STLpoint迭代器
	QVector<PointData>::iterator pointBegin();
	QVector<PointData>::iterator pointEnd();
	
	//槽信号
	void setXMove(double val);
	void setYMove(double val);
	void setZMove(double val);
	void setMoveVal(QVector3D position);

	QVector3D getMoveValue();
	
	//更新STL数据点
	void updateSTL();

signals:
	void drawSTLPoint();

private:
	QList<QString> m_fileSTL;
	QVector<PointData> m_currentSTLPoint;
	QVector<PointData> m_STLPoint;
	double m_xMove = 0.0, m_yMove = 0.0, m_zMove = 0.0, m_xMoveLast = 0.0, m_yMoveLast = 0.0, m_zMoveLast = 0.0;
	double m_xRot = 0.0, m_yRot = 0.0, m_zRot = 0.0, m_xRotLast = 0.0, m_yRotLast = 0.0, m_zRotLast = 0.0;
};