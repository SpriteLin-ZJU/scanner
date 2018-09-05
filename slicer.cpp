#include "slicer.h"

Slicer::Slicer()
{
	m_beginLayer = 0.30;
	m_layerHeight = 2.50;
}

Slicer::~Slicer()
{
	m_stlManager = NULL;
}

void Slicer::setSTLManager(STLManager * manager)
{
	m_stlManager = manager;
}

void Slicer::slice()
{
	m_stlManager->STLTopologize();
	clear();
	m_sliceTris = m_stlManager->m_tris;

	double zMin, zMax;
	double ex[6];
	m_stlManager->findExtreme(ex);
	zMin = ex[4];
	zMax = ex[5];
	double zLayer = 0.0;
	zLayer = zMin + m_beginLayer;

	while (zLayer < zMax) {
		getPolyLinePoints(zLayer);
		zLayer += m_layerHeight;
	}
}

void Slicer::getPolyLinePoints(double z)
{
	QSharedPointer<Layer> spLayer = QSharedPointer<Layer>(new Layer());
	spLayer->z = z;
	QVector<QSharedPointer<Triangle>> status;
	for (auto it = m_sliceTris.begin(); it != m_sliceTris.end(); it++) {		//����������Ƭ��ɸѡ���ཻ��Ƭ,��������ƽ���غϵĵ����㶯�������ƶ�
		
		//���㶯������������
		QVector<QSharedPointer<Vertex>> vertexInTri = { (*it)->spV1, (*it)->spV2, (*it)->spV3 };
		for (auto i = vertexInTri.begin(); i != vertexInTri.end(); i++) {
			if ((*i)->position.z() == z)
				(*i)->position += QVector3D(0.0, 0.0, perturbationVal);
		}
		(*it)->sortVertex();	//���㶯���ı������ĵ���������

		//�ж��ཻ��Ƭ,�����붯̬����
		if (((*it)->spVMin->position.z() < z) && ((*it)->spVMax->position.z() > z)) {
			//���ཻ��Ƭ���з��࣬��Ϊ����������������������
			judgeFaceType(z, *it);
			status.push_back(*it);
		}
	}
	//ѡȡ��һ����ƬΪ��ʼ��Ƭ
	QSharedPointer<Triangle> spSectSurface = status.at(0);
	spSectSurface->bUse = true;
	while (true) {
		//�����һ���ཻ��Ƭ��һ���ཻ�ߵĽ���
		QSharedPointer<PolyLine> spPolyLine = QSharedPointer<PolyLine>(new PolyLine());
		interSect(z, spSectSurface);	//�趨��ʼ�ཻ�ߣ�����m_sliceEdge����
		spSectSurface->spSelectIntersectLine = m_sliceEdges.back();	//����ǰ�ཻ�ıߴ���
		QVector3D tempLinkPoint = calcuInterSectPoint(z, spSectSurface->spSelectIntersectLine);
		QSharedPointer<PointTri> spPt0 = QSharedPointer<PointTri>(new PointTri(tempLinkPoint));
		spPt0->tri = spSectSurface->spSelectIntersectLine->spEdgeAdja.lock()->spTri.lock();
		spPolyLine->m_linkPoints.push_back(spPt0);
		//��һ����Ƭ����һ�ཻ�ߵĽ���
		judgeOtherLine(z, spSectSurface);
		tempLinkPoint = calcuInterSectPoint(z, spSectSurface->spOtherIntersectLine);
		spPolyLine->m_linkPoints.push_back(QSharedPointer<PointTri>(new PointTri(tempLinkPoint, spSectSurface)));
		//��ʼ�������ڵ��ཻ��Ƭ
		while (true) {
			//���������ཻ������ıߵĻ������Ѱ����һ�ཻ��Ƭ
			spSectSurface = m_sliceEdges.back()->spEdgeAdja.lock()->spTri.lock();
			spSectSurface->bUse = true;
			//������m_sliceEdges���ཻ�߾�����һ��Ƭ�ĵڶ����ཻ�ߣ����Ļ���߾��ǵ�ǰ��Ƭ��ѡ���ཻ��
			spSectSurface->spSelectIntersectLine = m_sliceEdges.back()->spEdgeAdja.lock();
			judgeOtherLine(z, spSectSurface);
			//����һ���ཻ�ߵĽ���
			tempLinkPoint = calcuInterSectPoint(z, spSectSurface->spOtherIntersectLine);
			spPolyLine->m_linkPoints.push_back(QSharedPointer<PointTri>(new PointTri(tempLinkPoint, spSectSurface)));
			//�жϽ����Ƿ�Ϊ��ʼ��,���ǣ������ڲ�ѭ��������һ�������㼯������ֱ�ӷ����ڲ�ѭ��
			if (*(spPolyLine->m_linkPoints.at(0)) ^= *(spPolyLine->m_linkPoints.back())) {
				spLayer->m_polyLines.push_back(spPolyLine);
				break;
			}
		}
		//������һ���������������ж�status���Ƿ���δʹ�õ���Ƭ
		//���У������Ƭ��λ��ǰ����ʼ����ĳ���һ����������û�У����������ѭ��
		bool finishFlag = true;
		for (auto it = status.begin(); it != status.end(); it++) {
			if ((*it)->bUse == false) {
				finishFlag = false;
				spSectSurface = *it;
				spSectSurface->bUse = true;
				break;
			}
		}
		//���û��δ��ʹ�õ���Ƭ���������ⲿѭ������ɸĳ���������
		if (finishFlag)
			break;
	}
	m_layers.push_back(spLayer);
	//����
	for (auto it = status.begin(); it != status.end(); it++) {
		(*it)->bUse = false;
		(*it)->faceType = NoSectSurface;
	}
	m_sliceEdges.clear();
}

void Slicer::clear()
{
	m_sliceEdges.clear();
	m_layers.clear();
	m_sliceTris.clear();
}

void Slicer::interSect(double z, QSharedPointer<Triangle> spSurface)
{
	if ((spSurface->spV1->position.z() - z)*(spSurface->spV2->position.z() - z) < 0)
		m_sliceEdges.push_back(spSurface->spEdge1);
	else
		m_sliceEdges.push_back(spSurface->spEdge2);		//��һ���߲��ཻ����ڶ����߿϶��ཻ
}

QVector3D Slicer::calcuInterSectPoint(double z, QSharedPointer<Edge> spEdge)
{
	QVector3D sectPoint;
	double lamda = (z - spEdge->spV1->position.z()) / (spEdge->spV2->position.z() - spEdge->spV1->position.z());
	double dx = spEdge->spV2->position.x() - spEdge->spV1->position.x();
	double dy = spEdge->spV2->position.y() - spEdge->spV1->position.y();
	sectPoint.setX(spEdge->spV1->position.x() + lamda * dx);
	sectPoint.setY(spEdge->spV1->position.y() + lamda * dy);
	sectPoint.setZ(z);

	return sectPoint;
}

void Slicer::judgeFaceType(double z, QSharedPointer<Triangle> spSurface)
{
	double z_min = getZMin(spSurface);
	double z_max = getZMax(spSurface);
	double z_mid = getZMid(spSurface);
	//��������Ƭ����ƽ���ཻ�����ڲ������㶯�������ֻ������������������»���������
	if (z_mid < z)
		spSurface->faceType = TwoPointBelow;
	else if (z_mid > z)
		spSurface->faceType = OnePointBelow;
	
}

void Slicer::judgeOtherLine(double z, QSharedPointer<Triangle> spSurface)
{
	//���ڲ����㶯��,������Ƭ����ƽ����ཻ���Ӧ��ֻ����������ƽ���Ϸ������·��������
	if ((spSurface->spSelectIntersectLine->spEdgePrev.lock()->spV1->position.z() - z)
		*(spSurface->spSelectIntersectLine->spEdgePrev.lock()->spV2->position.z() - z) < 0)
		spSurface->spOtherIntersectLine = spSurface->spSelectIntersectLine->spEdgePrev;
	else
		spSurface->spOtherIntersectLine = spSurface->spSelectIntersectLine->spEdgeNext;
	m_sliceEdges.push_back(spSurface->spOtherIntersectLine);
}

double Slicer::getZMin(QSharedPointer<Triangle> spSurface)
{
	double z1, z2, z3;
	z1 = spSurface->spV1->position.z();
	z2 = spSurface->spV2->position.z();
	z3 = spSurface->spV3->position.z();

	if (z1 > z2)
		qSwap(z1, z2);
	if (z2 > z3)
		qSwap(z2, z3);
	if (z1 > z2)
		qSwap(z1, z2);

	return z1;
}

double Slicer::getZMax(QSharedPointer<Triangle> spSurface)
{
	double z1, z2, z3;
	z1 = spSurface->spV1->position.z();
	z2 = spSurface->spV2->position.z();
	z3 = spSurface->spV3->position.z();

	if (z1 > z2)
		qSwap(z1, z2);
	if (z2 > z3)
		qSwap(z2, z3);
	if (z1 > z2)
		qSwap(z1, z2);

	return z3;
}

double Slicer::getZMid(QSharedPointer<Triangle> spSurface)
{
	double z1, z2, z3;
	z1 = spSurface->spV1->position.z();
	z2 = spSurface->spV2->position.z();
	z3 = spSurface->spV3->position.z();

	if (z1 > z2)
		qSwap(z1, z2);
	if (z2 > z3)
		qSwap(z2, z3);
	if (z1 > z2)
		qSwap(z1, z2);

	return z2;
}

void Slicer::drawPolyLine()
{
	slice();
	ShaderDrawable::update();
	showGraph();
	emit updateGraph();
}

bool Slicer::updateData()
{
	m_triangles.clear();
	m_lines.clear();
	QVector3D color = { 0.5f,0.3f,0.6f };
	//����������
	for (auto itLayer = m_layers.begin(); itLayer != m_layers.end(); itLayer++) {
		for (auto itPolyLine = (*itLayer)->m_polyLines.begin(); itPolyLine != (*itLayer)->m_polyLines.end(); itPolyLine++) {
			for (int i = 0; i < (*itPolyLine)->m_linkPoints.size() - 1; i++) {
				QVector3D point1((*itPolyLine)->m_linkPoints[i]->position);
				QVector3D point2((*itPolyLine)->m_linkPoints[i+1]->position);
				m_lines.append({ point1,color,m_vectorNaN });
				m_lines.append({ point2,color,m_vectorNaN });
			}
		}
	}

	return true;
}
