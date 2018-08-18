#include "stlmanager.h"
#include <QRegexp>
#include <QMatrix4x4>
#include <QMatrix>
#include <QDeBug>
#include <QTime>
STLManager::STLManager()
{
}

STLManager::~STLManager()
{
}

void STLManager::clear()
{
	m_fileSTL.clear();
	m_STLPoint.clear();
}

void STLManager::addLine(const QString & string)
{
	m_fileSTL.append(string);
}

int STLManager::pointCount()
{
	return m_STLPoint.count();
}

void STLManager::fileToPoint()
{
	//点坐标
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
	//面片法向量
	float nx = 0.0f;
	float ny = 0.0f;
	float nz = 0.0f;
	//法向量正则表达式
	QRegExp normalRx("facet normal ([^ ]+) ([^ ]+) ([^ ]+)");
	normalRx.setMinimal(false);
	//顶点正则表达式
	QRegExp positionRx("vertex ([^ ]+) ([^ ]+) ([^ ]+)");
	positionRx.setMinimal(false);

	int pos = 0;
	auto it = m_STLPoint.begin();
	for (auto it = m_fileSTL.cbegin(); it != m_fileSTL.cend(); it++) {
		//若为法向量
		if ((pos = normalRx.indexIn(*it)) != -1) {							//注意indexIn(str,int offset=0)!offset不能为-1；
			nx = normalRx.cap(1).toFloat();
			ny = normalRx.cap(2).toFloat();
			nz = normalRx.cap(3).toFloat();
		}
		//若为顶点坐标
		else if ((pos = positionRx.indexIn(*it)) != -1) {
			x = positionRx.cap(1).toFloat();
			y = positionRx.cap(2).toFloat();
			z = positionRx.cap(3).toFloat();
			m_STLPoint.append({ QVector3D(x,y,z),QVector3D(nx,ny,nz) });
		}
	}
	m_currentSTLPoint = m_STLPoint;
}

QVector<PointData>::iterator STLManager::pointBegin()
{
	return m_currentSTLPoint.begin();
}

QVector<PointData>::iterator STLManager::pointEnd()
{
	return m_currentSTLPoint.end();
}

void STLManager::setXMove(double val)
{
	m_xMove = val;
}

void STLManager::setYMove(double val)
{
	m_yMove = val;
}

void STLManager::setZMove(double val)
{
	m_zMove = val;
}

void STLManager::setMoveVal(QVector3D position)
{
	m_xMove = position.x();
	m_yMove = position.y();
	m_zMove = position.z();
	updateSTL();
}

QVector3D STLManager::getMoveValue()
{
	return QVector3D(m_xMove, m_yMove, m_zMove);
}

void STLManager::updateSTL()
{
	//若STL模型未旋转也未移动
	if ((m_xMove == m_xMoveLast) && (m_yMove == m_yMoveLast) && (m_zMove == m_zMoveLast)
		&& (m_xRot == m_xRotLast) && (m_yRot == m_yRotLast) && (m_zRot == m_zRotLast))	return;
	//模型需要旋转或者移动
	QMatrix4x4 moveMatrix;
	moveMatrix.setToIdentity();
	moveMatrix.translate(m_xMove, m_yMove, m_zMove);
	QMatrix4x4 rotateMatrix;
	rotateMatrix.setToIdentity();
	rotateMatrix.rotate(m_xRot, 1.0, 0.0, 0.0);
	rotateMatrix.rotate(m_yRot, 0.0, 1.0, 0.0);
	rotateMatrix.rotate(m_zRot, 0.0, 0.0, 1.0);
	QMatrix4x4 positionMatrix;
	positionMatrix = moveMatrix * rotateMatrix;
	QMatrix4x4 normalMatrix;
	normalMatrix = QMatrix4x4((moveMatrix*rotateMatrix).normalMatrix());	//右下角为1

	m_currentSTLPoint.clear();
	for (auto it = m_STLPoint.begin(); it != m_STLPoint.end(); it++) {
		PointData transedPoint;
		transedPoint.position = positionMatrix * it->position;
		transedPoint.normal = normalMatrix * it->normal;		//因为w分量为1，所以返回计算后的QVector3D（x，y，z）
		
		m_currentSTLPoint.push_back(transedPoint);
	}
	emit drawSTLPoint();
}

