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

//���յ�һ��ok�źź󣬲鿴gcodebuffer����ΪG0\G1ָ��еĻ�תΪԲ�����ݵ�
void GcodeDrawer::drawSingleGcode()
{
	//^(G[01])\s*(F\d{0,4})?\s*(X\s*-?\d+\.?\d*)?\s*(Y\s*-?\d+\.?\d*)?\s*(Z\s*-?\d+\.?\d*)?
	QRegExp rx("^(G[01])\\s*(F\\d{0,4})?\\s*(X\\s*-?\\d+\\.?\\d*)?\\s*(Y\\s*-?\\d+\\.?\\d*)?\\s*(Z\\s*-?\\d+\\.?\\d*)?");
	rx.setMinimal(false);
	rx.setCaseSensitivity(Qt::CaseInsensitive);

	//�ж��Ƿ�ΪG0��G1ָ��
	int pos = 0;
	if ((pos = rx.indexIn(m_gcodeManager->takeFirstBuffer(), pos)) != -1) {
		//ΪG0ָ��
		if(rx.cap(1)=="G0"||rx.cap(1)=="g0")
			for (int i = 3; i < 6; i++) {
				if (!rx.cap(i).isEmpty()) {
					QString s = rx.cap(i);
					s.remove(0,1);//��������
					m_lastPoint[i - 3] = s.toFloat();
				}
			}
		//ΪG1ָ��,�趨Ŀ������겢����
		else {
			for (int i = 3; i < 6; i++) {
				if (!rx.cap(i).isEmpty()) {
					QString s = rx.cap(i);
					s.remove(0, 1);//��������
					m_targetPoint[i - 3] = s.toFloat();
				}
				else
					m_targetPoint[i - 3] = m_lastPoint[i - 3];
			}
			//���Ŀ�����꼰��ǰ���꣬����m_triangles.
			ShaderDrawable::update();
			updateData();
			emit updateGraph();	//ΪʲôGLWidget��update()�������ú�û����������ͼ��
		}
	}
}

bool GcodeDrawer::updateData()
{
	if (m_lastPoint == m_targetPoint)
		return true;

	//����任����
	QVector3D difference = m_targetPoint - m_lastPoint;
	float length = difference.length();
	QVector3D axis = QVector3D(0.0f, 0.0f, length);
	QVector3D crossProduct = QVector3D::normal(axis, difference);
	QMatrix4x4 changeMatrix;
	changeMatrix.translate(m_lastPoint);
	changeMatrix.rotate(90.0f, crossProduct);	//ע�⣬�൱��translate*rotate*point���������ԭ������ת��ƽ�ơ�
	QMatrix4x4 normalMatrix;
	normalMatrix = QMatrix4x4(changeMatrix.normalMatrix());
	//����Բ����������
	int pointCircleCount=6;
	double r = 0.2;
	double diffRadians = 2 * M_PI / pointCircleCount;

	QVector3D color = { 0.1f,0.3f,0.8f };
	for (int i = 0; i < pointCircleCount; i++) {
		//��һ��������Ƭ
		QVector3D point0 = (changeMatrix * QVector4D(r*qCos(i*diffRadians), r*qSin(i*diffRadians), 0, 1)).toVector3D();
		QVector3D point1 = (changeMatrix * QVector4D(r*qCos(i*diffRadians), r*qSin(i*diffRadians), length, 1)).toVector3D();
		QVector3D point2 = (changeMatrix * QVector4D(r*qCos((i + 1)*diffRadians), r*qSin((i + 1)*diffRadians), 0, 1)).toVector3D();
		QVector3D normal = QVector3D::normal((point2 - point0), (point1 - point0));	//������Ƭ�ķ�����������
		
		m_triangles.append({ point0,color, normal });
		m_triangles.append({ point1,color, normal });
		m_triangles.append({ point2,color, normal });
		//�ڶ���������Ƭ
		QVector3D point3 = (changeMatrix * QVector4D(r*qCos((i + 1)*diffRadians), r*qSin((i + 1)*diffRadians), length, 1)).toVector3D();
		m_triangles.append({ point1,color, normal });
		m_triangles.append({ point2,color, normal });
		m_triangles.append({ point3,color, normal });
		//���� ע��Qt�������򣬶�opengl��������
		normal = normalMatrix * QVector3D(0, 0, -1);
		m_triangles.append({ changeMatrix * QVector4D(0, 0, 0, 1).toVector3D(),color, normal });
		m_triangles.append({ point0,color, normal });
		m_triangles.append({ point2,color, normal });
		//����
		normal = normalMatrix * QVector3D(0, 0, 1);
		m_triangles.append({ changeMatrix * QVector4D(0, 0, length, 1).toVector3D(),color, normal });
		m_triangles.append({ point1,color, normal });
		m_triangles.append({ point3,color, normal });
	}

	m_lastPoint = m_targetPoint;
	return true;
}
