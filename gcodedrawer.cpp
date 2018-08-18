#include "gcodedrawer.h"
#include "gcodemanager.h"
#include <QRegexp>
#include <QtMath>

GcodeDrawer::GcodeDrawer()
	: m_gcodeManager(NULL)
{
}

GcodeDrawer::~GcodeDrawer()
{
	m_gcodeManager = NULL;
}

void GcodeDrawer::setGcodeManager(GcodeManager * manager)
{
	m_gcodeManager = manager;
}

//当收到一条ok信号后，查看gcodebuffer，若为G0\G1指令，有的话转为圆柱数据点
void GcodeDrawer::drawSingleGcode()
{
	//^(G[01])\s*(F\d{0,4})?\s*(X\s*-?\d+\.?\d*)?\s*(Y\s*-?\d+\.?\d*)?\s*(Z\s*-?\d+\.?\d*)?
	QRegExp rx("^(G[01])\\s*(F\\d{0,4})?\\s*(X\\s*-?\\d+\\.?\\d*)?\\s*(Y\\s*-?\\d+\\.?\\d*)?\\s*(Z\\s*-?\\d+\\.?\\d*)?");
	rx.setMinimal(false);
	rx.setCaseSensitivity(Qt::CaseInsensitive);

	//判断是否为G0或G1指令
	int pos = 0;
	if ((pos = rx.indexIn(m_gcodeManager->takeFirstBuffer(), pos)) != -1) {
		//为G0指令
		if(rx.cap(1)=="G0"||rx.cap(1)=="g0")
			for (int i = 3; i < 6; i++) {
				if (!rx.cap(i).isEmpty()) {
					QString s = rx.cap(i);
					s.remove(0,1);//保留坐标
					m_lastPoint[i - 3] = s.toFloat();
				}
			}
		//为G1指令,设定目标点坐标并绘制
		else {
			for (int i = 3; i < 6; i++) {
				if (!rx.cap(i).isEmpty()) {
					QString s = rx.cap(i);
					s.remove(0, 1);//保留坐标
					m_targetPoint[i - 3] = s.toFloat();
				}
				else
					m_targetPoint[i - 3] = m_lastPoint[i - 3];
			}
			//结合目标坐标及当前坐标，更新m_triangles.
			ShaderDrawable::update();
			updateData();
			emit updateGraph();	//为什么GLWidget的update()函数调用后没有立即更新图像？
		}
	}
}

bool GcodeDrawer::updateData()
{
	if (m_lastPoint == m_targetPoint)
		return true;

	//计算变换矩阵
	QVector3D difference = m_targetPoint - m_lastPoint;
	float length = difference.length();
	QVector3D axis = QVector3D(0.0f, 0.0f, length);
	QVector3D crossProduct = QVector3D::normal(axis, difference);
	QMatrix4x4 changeMatrix;
	changeMatrix.translate(m_lastPoint);
	changeMatrix.rotate(90.0f, crossProduct);	//注意，相当于translate*rotate*point，根据左乘原则，先旋转再平移。
	QMatrix4x4 normalMatrix;
	normalMatrix = QMatrix4x4(changeMatrix.normalMatrix());
	//载入圆柱各点坐标
	int pointCircleCount=6;
	double r = 0.2;
	double diffRadians = 2 * M_PI / pointCircleCount;

	QVector3D color = { 0.1f,0.3f,0.8f };
	for (int i = 0; i < pointCircleCount; i++) {
		//第一个三角面片
		QVector3D point0 = (changeMatrix * QVector4D(r*qCos(i*diffRadians), r*qSin(i*diffRadians), 0, 1)).toVector3D();
		QVector3D point1 = (changeMatrix * QVector4D(r*qCos(i*diffRadians), r*qSin(i*diffRadians), length, 1)).toVector3D();
		QVector3D point2 = (changeMatrix * QVector4D(r*qCos((i + 1)*diffRadians), r*qSin((i + 1)*diffRadians), 0, 1)).toVector3D();
		QVector3D normal = QVector3D::normal((point2 - point0), (point1 - point0));	//三角面片的法向量，朝外
		
		m_triangles.append({ point0,color, normal });
		m_triangles.append({ point1,color, normal });
		m_triangles.append({ point2,color, normal });
		//第二个三角面片
		QVector3D point3 = (changeMatrix * QVector4D(r*qCos((i + 1)*diffRadians), r*qSin((i + 1)*diffRadians), length, 1)).toVector3D();
		m_triangles.append({ point1,color, normal });
		m_triangles.append({ point2,color, normal });
		m_triangles.append({ point3,color, normal });
		//底面 注意Qt是行主序，而opengl是列主序
		normal = normalMatrix * QVector3D(0, 0, -1);
		m_triangles.append({ changeMatrix * QVector4D(0, 0, 0, 1).toVector3D(),color, normal });
		m_triangles.append({ point0,color, normal });
		m_triangles.append({ point2,color, normal });
		//顶面
		normal = normalMatrix * QVector3D(0, 0, 1);
		m_triangles.append({ changeMatrix * QVector4D(0, 0, length, 1).toVector3D(),color, normal });
		m_triangles.append({ point1,color, normal });
		m_triangles.append({ point3,color, normal });
	}

	m_lastPoint = m_targetPoint;
	return true;
}
