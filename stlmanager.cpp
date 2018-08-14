#include "stlmanager.h"
#include <QRegexp>
#include <QDeBug>
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
	//������
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
	//��Ƭ������
	float nx = 0.0f;
	float ny = 0.0f;
	float nz = 0.0f;
	//������������ʽ
	QRegExp normalRx("facet normal ([^ ]+) ([^ ]+) ([^ ]+)");
	normalRx.setMinimal(false);
	//����������ʽ
	QRegExp positionRx("vertex ([^ ]+) ([^ ]+) ([^ ]+)");
	positionRx.setMinimal(false);

	int pos = 0;
	auto it = m_STLPoint.begin();
	for (auto it = m_fileSTL.cbegin(); it != m_fileSTL.cend(); it++) {
		//��Ϊ������
		if ((pos = normalRx.indexIn(*it)) != -1) {							//ע��indexIn(str,int offset=0)!offset����Ϊ-1��
			nx = normalRx.cap(1).toFloat();
			ny = normalRx.cap(2).toFloat();
			nz = normalRx.cap(3).toFloat();
		}
		//��Ϊ��������
		else if ((pos = positionRx.indexIn(*it)) != -1) {
			x = positionRx.cap(1).toFloat();
			y = positionRx.cap(2).toFloat();
			z = positionRx.cap(3).toFloat();
			m_STLPoint.append({ QVector3D(x,y,z),QVector3D(nx,ny,nz) });
		}
	}

}

QVector<PointData>::iterator STLManager::pointBegin()
{
	return m_STLPoint.begin();
}

QVector<PointData>::iterator STLManager::pointEnd()
{
	return m_STLPoint.end();
}

