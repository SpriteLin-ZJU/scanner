#include "stlmanager.h"
#include <QRegexp>
#include <QMatrix4x4>
#include <QMatrix>
#include <QDeBug>
#include <QTime>
#include <QtAlgorithms>
#include <QList>
#include <QtCore/qmath.h>
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

void STLManager::readBinarySTL(QFile& binaryFile)
{
	m_STLPoint.clear();

	binaryFile.seek(0);
	QDataStream dataStream(&binaryFile);
	dataStream.setVersion(QDataStream::Qt_5_9);
	dataStream.setByteOrder(QDataStream::LittleEndian);	//务必设置！！！
	dataStream.setFloatingPointPrecision(QDataStream::SinglePrecision);

	binaryFile.seek(80);
	quint32 triNum = 0;
	dataStream >> triNum;
	//点坐标
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
	QVector3D point;
	//面片法向量
	float nx = 0.0f;
	float ny = 0.0f;
	float nz = 0.0f;
	QVector3D normal;
	for (int i = 0; i < triNum; i++) {
		//读取法向量
		dataStream >> nx;
		dataStream >> ny;
		dataStream >> nz;
		normal = QVector3D(nx, ny, nz);
		//读取三个点坐标
		for (int j = 0; j < 3; j++) {
			dataStream >> x;
			dataStream >> y;
			dataStream >> z;
			point = QVector3D(round(x,2), round(y,2), round(z,2));
			m_STLPoint.append({ point,normal });
		}
		//舍弃最后两字节标志位
		dataStream.skipRawData(2);
	}
	m_currentSTLPoint = m_STLPoint;
	binaryFile.close();

}

void STLManager::readAsciiSTL(QFile& asciiFile)
{
	clear();
	asciiFile.seek(0);
	QTextStream textStream(&asciiFile);
	while (!textStream.atEnd())
		addLine(textStream.readLine());

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
	for (auto it = m_fileSTL.cbegin(); it != m_fileSTL.cend(); it++) {
		//若为法向量
		if ((pos = normalRx.indexIn(*it)) != -1) {							//注意indexIn(str,int offset=0)!offset不能为-1；
			nx = normalRx.cap(1).toFloat();
			ny = normalRx.cap(2).toFloat();
			nz = normalRx.cap(3).toFloat();
		}
		//若为顶点坐标
		else if ((pos = positionRx.indexIn(*it)) != -1) {
			x = round(positionRx.cap(1).toFloat(), 2);
			y = round(positionRx.cap(2).toFloat(), 2);
			z = round(positionRx.cap(3).toFloat(), 2);
			m_STLPoint.append({ QVector3D(x,y,z),QVector3D(nx,ny,nz) });
		}
	}
	m_currentSTLPoint = m_STLPoint;
	asciiFile.close();
}

float STLManager::round(float val, int n)
{
	val = ((float)(int)(val * qPow(10, n) + 0.5)) / qPow(10, n);
	return val;
}

//二分法查找坐标点在无重复点指针表m_vertices中的位置，并将指向该点的指针返回
QSharedPointer<Vertex> STLManager::searchPtInVertices(const QVector3D & pt)
{
	Vertex vertex(pt);
	int sz = m_vertices.size();
	int min_i = 0;
	int max_i = sz - 1;

	if (*m_vertices[min_i] == vertex)
		return m_vertices[min_i];
	if (*m_vertices[max_i] == vertex)
		return m_vertices[max_i];

	int mid_i=0;
	while (true) {
		mid_i = (min_i + max_i) / 2;
		if (*m_vertices[mid_i] < vertex)
			min_i = mid_i;
		else if (*m_vertices[mid_i] > vertex)
			max_i = mid_i;
		else
			return m_vertices[mid_i];
	}
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

void STLManager::setRotateVal(QVector3D rotate)
{
	m_xRot = rotate.x();
	m_yRot = rotate.y();
	m_zRot = rotate.z();
	updateSTL();
}

void STLManager::setScaleVal(QVector3D scale)
{
	m_xScale = scale.x();
	m_yScale = scale.y();
	m_zScale = scale.z();
	updateSTL();
}


QVector3D STLManager::getMoveValue()
{
	return QVector3D(m_xMove, m_yMove, m_zMove);
}

QVector3D STLManager::getRotateValue()
{
	return QVector3D(m_xRot, m_yRot, m_zRot);
}

QVector3D STLManager::getScaleValue()
{
	return QVector3D(m_xScale,m_yScale,m_zScale);
}

void STLManager::updateSTL()
{
	//若STL模型未旋转也未移动
	if ((m_xMove == m_xMoveLast) && (m_yMove == m_yMoveLast) && (m_zMove == m_zMoveLast)
		&& (m_xRot == m_xRotLast) && (m_yRot == m_yRotLast) && (m_zRot == m_zRotLast)
		&& (m_xScale == m_xScaleLast) && (m_yRot == m_yScaleLast) && (m_zRot == m_zScaleLast))	return;
	//模型需要旋转或者移动或者缩放
	//平移矩阵
	QMatrix4x4 moveMatrix;
	moveMatrix.setToIdentity();
	moveMatrix.translate(m_xMove, m_yMove, m_zMove);
	//旋转矩阵
	QMatrix4x4 rotateMatrix;
	rotateMatrix.setToIdentity();
	rotateMatrix.rotate(m_xRot, 1.0, 0.0, 0.0);
	rotateMatrix.rotate(m_yRot, 0.0, 1.0, 0.0);
	rotateMatrix.rotate(m_zRot, 0.0, 0.0, 1.0);
	//缩放矩阵
	QMatrix4x4 scaleMatrix;
	scaleMatrix.setToIdentity();
	scaleMatrix.scale(m_xScale/100, m_yScale/100, m_zScale/100);
	//变换矩阵
	QMatrix4x4 positionMatrix;
	positionMatrix = moveMatrix * rotateMatrix * scaleMatrix;
	//法向量变换矩阵
	QMatrix4x4 normalMatrix;
	normalMatrix = QMatrix4x4(positionMatrix.normalMatrix());	//右下角为1

	m_currentSTLPoint.clear();
	for (auto it = m_STLPoint.begin(); it != m_STLPoint.end(); it++) {
		PointData transedPoint;
		transedPoint.position = positionMatrix * it->position;
		transedPoint.normal = normalMatrix * it->normal;		//因为w分量为1，所以返回计算后的QVector3D（x，y，z）
		
		m_currentSTLPoint.push_back(transedPoint);
	}
	m_xMoveLast = m_xMove;
	m_yMoveLast = m_yMove;
	m_zMoveLast = m_zMove;
	m_xRotLast = m_xRot;
	m_yRotLast = m_yRot;
	m_zRotLast = m_zRot;
	m_xScaleLast = m_xScale;
	m_yScaleLast = m_yScale;
	m_zScaleLast = m_zScale;

	emit drawSTLPoint();
}

void STLManager::centreSTL()
{
	
	QMatrix4x4 rotateMatrix;
	rotateMatrix.setToIdentity();
	rotateMatrix.rotate(m_xRot, 1.0, 0.0, 0.0);
	rotateMatrix.rotate(m_yRot, 0.0, 1.0, 0.0);
	rotateMatrix.rotate(m_zRot, 0.0, 0.0, 1.0);
	for (auto it = m_STLPoint.begin(); it != m_STLPoint.end(); it++)
		it->position = rotateMatrix * it->position;

	double ext[6];
	findOrigianlExtrem(ext);
	double midX = (ext[0] + ext[1]) / 2;
	double midY = (ext[2] + ext[3]) / 2;
	double minZ = ext[4];

	QMatrix4x4 moveMatrix;
	moveMatrix.setToIdentity();
	moveMatrix.translate(-midX, -midY, -minZ);
	for (auto it = m_STLPoint.begin(); it != m_STLPoint.end(); it++)
		it->position = moveMatrix * it->position;

	m_xMove = m_xMoveLast = m_yMove = m_yMoveLast = m_zMove = m_zMoveLast = 0;
	m_xRot = m_xRotLast = m_yRot = m_yRotLast = m_zRot = m_zRotLast = 0;
	m_xScaleLast = m_yScaleLast = m_zScaleLast = 100;

	updateSTL();
}

void STLManager::STLTopologize()
{
	m_edges.clear();
	m_normals.clear();
	m_tris.clear();
	m_vertices.clear();

	QVector<Vertex> tmp_vertices;
	int szVertex = m_currentSTLPoint.size();
	tmp_vertices.reserve(szVertex);			//预先分配大小，提高效率

	//将之前保存的STL顶点存入至临时动态数组中
	for (int i = 0; i < szVertex; i++)
		tmp_vertices.push_back(Vertex(m_currentSTLPoint[i].position));
	qSort(tmp_vertices.begin(), tmp_vertices.end());	//将所有点从小到大排序，便于后续二分查找

	QSharedPointer<Vertex> spVertex(new Vertex(tmp_vertices[0]));
	m_vertices.push_back(spVertex);						

	//将点存入拓扑数组中，并删除重复点
	for (int i = 1; i < szVertex; i++) {
		if (tmp_vertices[i].position == tmp_vertices[i - 1].position)
			continue;
		else {
			spVertex = QSharedPointer<Vertex>(new Vertex(tmp_vertices[i]));
			m_vertices.push_back(spVertex);
		}
	}
	//QVector<Vertex>().swap(tmp_vertices);		//实际上离开作用域后会自动释放，引用计数-1？用QDUBUG测试一下

	//建立拓扑关系
	int szTris = szVertex / 3;
	//建立面表
	for (int i = 0; i < szTris; i++) {
		QSharedPointer<Triangle> spFace = QSharedPointer<Triangle>(new Triangle());
		QSharedPointer<QVector3D> spNormal = QSharedPointer<QVector3D>(new QVector3D(m_currentSTLPoint[i * 3].normal));
		m_normals.push_back(spNormal);
		spFace->spV1 = searchPtInVertices(m_currentSTLPoint[i * 3].position);
		spFace->spV2 = searchPtInVertices(m_currentSTLPoint[i * 3 + 1].position);
		spFace->spV3 = searchPtInVertices(m_currentSTLPoint[i * 3 + 2].position);
		spFace->spNormal = spNormal;
		//对vmax,vmid,vmin进行排序
		spFace->sortVertex();
		m_tris.push_back(spFace);
	}
	//建立边表
	for (int i = 0; i < szTris; i++) {
		QSharedPointer<Edge> e1(new Edge(m_tris[i]->spV1, m_tris[i]->spV2));
		QSharedPointer<Edge> e2(new Edge(m_tris[i]->spV2, m_tris[i]->spV3));
		QSharedPointer<Edge> e3(new Edge(m_tris[i]->spV3, m_tris[i]->spV1));
		e1->spTri = e2->spTri = e3->spTri = m_tris[i];
		e1->spEdgePrev = e2->spEdgeNext = e3;	//e1的前一条边和e2的后一条边即为e3
		e2->spEdgePrev = e3->spEdgeNext = e1;
		e3->spEdgePrev = e1->spEdgeNext = e2;
		m_edges.push_back(e1);
		m_edges.push_back(e2);
		m_edges.push_back(e3);					//存入半边数组
		m_tris[i]->spEdge1 = e1;
		m_tris[i]->spEdge2 = e2;
		m_tris[i]->spEdge3 = e3;
	}
	int szEdge = m_edges.size();
	QVector<EdgeHull> edgeHulls;
	EdgeHull edgeHull;						//接下来要做的事找到edge的向量边edgeAdja
	for (int i = 0; i < szEdge; i++) {
		edgeHull.spEdge = m_edges[i];
		edgeHulls.push_back(edgeHull);		//将半边数据结构中的边存入edgeHulls数组
	}
	qSort(edgeHulls.begin(), edgeHulls.end());
	for (int i = 0; i < edgeHulls.size(); i++)
		edgeHulls[i].spEdge->num = i;		//半边数据结构中的边的编号，按边从小到大排序

	for (int i = 0; i < szEdge - 1; i++) {
		if (edgeHulls[i].isOpposite(edgeHulls[i + 1])) {
			edgeHulls[i].spEdge->spEdgeAdja = edgeHulls[i + 1].spEdge;
			edgeHulls[i + 1].spEdge->spEdgeAdja = edgeHulls[i].spEdge;
			i++;
		}
	}

	//处理还没有伙伴半边的边，可能为裂缝
	QList<EdgeHull> tmpEdgeHulls;
	unsigned int szEdgeError=0;
	int iterTimes = 0;
	for (int i = 0; i < edgeHulls.size(); i++) {
		if (edgeHulls[i].spEdge->spEdgeAdja == NULL) {
			tmpEdgeHulls.append(edgeHulls[i]);
			szEdgeError++;
		}
	}
	while (szEdgeError > 1 && iterTimes < 3) {
		auto it = tmpEdgeHulls.begin();
		while (it != tmpEdgeHulls.end()) {
			if (it + 1 == tmpEdgeHulls.end())
				break;
			//如果两条边在误差范围内（0.01），则认为为相邻边，***并取平均点作为新点
			if (it->isOppositeApp(*(it + 1))) {
				it->spEdge->spEdgeAdja = (it + 1)->spEdge;
				(it + 1)->spEdge->spEdgeAdja = it->spEdge;
				QVector3D newVertexPos = (it->spEdge->spV1->position + (it + 1)->spEdge->spV2->position) / 2;
				it->spEdge->spV1->position = (it + 1)->spEdge->spV2->position = newVertexPos;//应该将所有指向V2的指针指向V1
				newVertexPos = (it->spEdge->spV2->position + (it + 1)->spEdge->spV1->position) / 2;
				it->spEdge->spV2->position = (it + 1)->spEdge->spV1->position = newVertexPos;
				//将已修复的边从链表删除
				it = tmpEdgeHulls.erase(it);
				szEdgeError--;
				it = tmpEdgeHulls.erase(it);
				szEdgeError--;
			}
			else
				it++;
		}
		iterTimes++;
	}
}

void STLManager::findExtreme(double ext[])
{
	double x_min = m_vertices[0]->position.x();
	double x_max = x_min;
	double y_min = m_vertices[0]->position.y();
	double y_max = y_min;
	double z_min = m_vertices[0]->position.z();
	double z_max = z_min;

	int sz = m_vertices.size();
	for (int i = 1; i<sz; i++)
	{
		if (m_vertices[i]->position.x() > x_max)
		{
			x_max = m_vertices[i]->position.x();
		}
		else if (m_vertices[i]->position.x() < x_min)
		{
			x_min = m_vertices[i]->position.x();
		}
		if (m_vertices[i]->position.y() > y_max)
		{
			y_max = m_vertices[i]->position.y();
		}
		else if (m_vertices[i]->position.y() < y_min)
		{
			y_min = m_vertices[i]->position.y();
		}
		if (m_vertices[i]->position.z() > z_max)
		{
			z_max = m_vertices[i]->position.z();
		}
		else if (m_vertices[i]->position.z() < z_min)
		{
			z_min = m_vertices[i]->position.z();
		}
	}

	ext[0] = x_min;
	ext[1] = x_max;
	ext[2] = y_min;
	ext[3] = y_max;
	ext[4] = z_min;
	ext[5] = z_max;
}

void STLManager::findOrigianlExtrem(double ext[])
{
	double x_min = m_STLPoint.at(0).position.x();
	double x_max = x_min;
	double y_min = m_STLPoint.at(0).position.y();
	double y_max = y_min;
	double z_min = m_STLPoint.at(0).position.z();
	double z_max = z_min;

	int sz = m_STLPoint.size();
	for (int i = 1; i<sz; i++)
	{
		if (m_STLPoint[i].position.x() > x_max)
		{
			x_max = m_STLPoint[i].position.x();
		}
		else if (m_STLPoint[i].position.x() < x_min)
		{
			x_min = m_STLPoint[i].position.x();
		}
		if (m_STLPoint[i].position.y() > y_max)
		{
			y_max = m_STLPoint[i].position.y();
		}
		else if (m_STLPoint[i].position.y() < y_min)
		{
			y_min = m_STLPoint[i].position.y();
		}
		if (m_STLPoint[i].position.z() > z_max)
		{
			z_max = m_STLPoint[i].position.z();
		}
		else if (m_STLPoint[i].position.z() < z_min)
		{
			z_min = m_STLPoint[i].position.z();
		}
	}

	ext[0] = x_min;
	ext[1] = x_max;
	ext[2] = y_min;
	ext[3] = y_max;
	ext[4] = z_min;
	ext[5] = z_max;
}


