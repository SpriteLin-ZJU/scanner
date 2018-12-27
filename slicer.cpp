#include "slicer.h"
#include <QSettings>

//point cloud library
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

Slicer::Slicer()
{
	m_beginLayer = 1;
	updateColor();


	qRegisterMetaType<QVector<double>>("QVector<double>");
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
	drawPolyLine();
	m_sliced = true;
}
/*linshi
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
void Slicer::linshiyongyong(double _z)
{
	for (auto itLayer = m_layers.begin(); itLayer != m_layers.end(); itLayer++) {
		if ((*itLayer)->z == _z) {
			PointCloudT::Ptr tempCloud(new PointCloudT);
			PointT tempPoint;
			//遍历每个轮廓线的线段
			for (auto itPolyLine = (*itLayer)->m_polyLines.begin(); itPolyLine != (*itLayer)->m_polyLines.end(); itPolyLine++) {
				for (int i = 0; i < (*itPolyLine)->m_linkPoints.size()-1; i++) {
					QVector3D point1((*itPolyLine)->m_linkPoints[i]->position);
					QVector3D point2((*itPolyLine)->m_linkPoints[i+1]->position);
					if ((point2 - point1).length() > 0.15) {
						double deltX = point2.x() - point1.x();
						double deltY = point2.y() - point1.y();
						double lamb = 0.1 / (point2 - point1).length();
						QVector3D currentPoint(point1);
						while ((currentPoint - point1).length() < (point2 - point1).length()) {
							tempPoint.x = currentPoint.x();
							tempPoint.y = currentPoint.y();
							tempPoint.z = 29.2F;
							tempPoint.intensity = 0;
							tempCloud->push_back(tempPoint);
							currentPoint.setX(lamb*deltX + currentPoint.x());
							currentPoint.setY(lamb*deltY + currentPoint.y());
						}
					}
					else {
						tempPoint.x = point1.x();
						tempPoint.y = point1.y();
						tempPoint.z = 29.2F;
						tempPoint.intensity = 0;
						tempCloud->push_back(tempPoint);
					}
				}
			}
			//输出
			pcl::io::savePCDFileASCII<PointT>("STL29_2.pcd", *tempCloud);
			//qDebug() << "Layer Height: " << _z;
			//qDebug() << "Line Count: " << lineCount << "  MaxLength: " << maxLength << "  MinLength: " << minLength << "AverageLength: " << aveLength / lineCount;
		}
	}
}
*/

void Slicer::getPolyLinePoints(double z)
{
	QSharedPointer<Layer> spLayer = QSharedPointer<Layer>(new Layer());
	spLayer->z = z;
	QVector<QSharedPointer<Triangle>> status;
	for (auto it = m_sliceTris.begin(); it != m_sliceTris.end(); it++) {		//遍历所有面片，筛选出相交面片,并将与切平面重合的点用摄动法向上移动
		
		//用摄动法处理顶点坐标
		QVector<QSharedPointer<Vertex>> vertexInTri = { (*it)->spV1, (*it)->spV2, (*it)->spV3 };
		for (auto i = vertexInTri.begin(); i != vertexInTri.end(); i++) {
			if ((*i)->position.z() == z)
				(*i)->position += QVector3D(0.0, 0.0, perturbationVal);
		}
		(*it)->sortVertex();	//对摄动法改变坐标后的点重新排序

		//判断相交面片,并存入动态数组
		if (((*it)->spVMin->position.z() < z) && ((*it)->spVMax->position.z() > z)) {
			//对相交面片进行分类，分为两点在上与两点在下两类
			judgeFaceType(z, *it);
			status.push_back(*it);
		}
	}
	//选取第一个面片为起始面片
	QSharedPointer<Triangle> spSectSurface = status.at(0);
	spSectSurface->bUse = true;
	while (true) {
		//处理第一个相交面片第一条相交边的交点
		QSharedPointer<PolyLine> spPolyLine = QSharedPointer<PolyLine>(new PolyLine());
		interSect(z, spSectSurface);	//设定起始相交边，存入m_sliceEdge容器
		spSectSurface->spSelectIntersectLine = m_sliceEdges.back();	//将当前相交的边存入
		QVector3D tempLinkPoint = calcuInterSectPoint(z, spSectSurface->spSelectIntersectLine);
		QSharedPointer<PointTri> spPt0 = QSharedPointer<PointTri>(new PointTri(tempLinkPoint));
		spPt0->tri = spSectSurface->spSelectIntersectLine->spEdgeAdja.lock()->spTri.lock();
		spPolyLine->m_linkPoints.push_back(spPt0);
		//第一个面片的另一相交边的交点
		judgeOtherLine(z, spSectSurface);
		tempLinkPoint = calcuInterSectPoint(z, spSectSurface->spOtherIntersectLine);
		spPolyLine->m_linkPoints.push_back(QSharedPointer<PointTri>(new PointTri(tempLinkPoint, spSectSurface)));
		//开始处理相邻的相交面片
		while (true) {
			//由最后存入相交边数组的边的伙伴半边来寻找下一相交面片
			spSectSurface = m_sliceEdges.back()->spEdgeAdja.lock()->spTri.lock();
			spSectSurface->bUse = true;
			//最后存入m_sliceEdges的相交边就是上一面片的第二条相交边，它的伙伴半边就是当前面片已选的相交边
			spSectSurface->spSelectIntersectLine = m_sliceEdges.back()->spEdgeAdja.lock();
			judgeOtherLine(z, spSectSurface);
			//求另一条相交边的交点
			tempLinkPoint = calcuInterSectPoint(z, spSectSurface->spOtherIntersectLine);
			spPolyLine->m_linkPoints.push_back(QSharedPointer<PointTri>(new PointTri(tempLinkPoint, spSectSurface)));
			//判断交点是否为起始点,若是，跳出内层循环，生成一个轮廓点集，若否，直接返回内层循环
			if (*(spPolyLine->m_linkPoints.at(0)) ^= *(spPolyLine->m_linkPoints.back())) {
				spLayer->m_polyLines.push_back(spPolyLine);
				break;
			}
		}
		//处理完一个完整的轮廓后，判断status中是否还有未使用的面片
		//若有，则把面片置位当前，开始处理改成另一个轮廓，若没有，则跳出外层循环
		bool finishFlag = true;
		for (auto it = status.begin(); it != status.end(); it++) {
			if ((*it)->bUse == false) {
				finishFlag = false;
				spSectSurface = *it;
				spSectSurface->bUse = true;
				break;
			}
		}
		//如果没有未被使用的面片，则跳出外部循环，完成该层轮廓处理
		if (finishFlag)
			break;
	}
	m_layers.push_back(spLayer);
	//后处理
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
		m_sliceEdges.push_back(spSurface->spEdge2);		//第一条边不相交，则第二条边肯定相交
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

QVector3D Slicer::calcuScanLineSectPoint(double scanLineY, QSharedPointer<LayerEdge> spLayerEdge)
{
	QVector3D sectPoint;
	double lamda = (scanLineY - spLayerEdge->spLEHead->position.y()) / (spLayerEdge->spLEEnd->position.y() - spLayerEdge->spLEHead->position.y());
	double dx = spLayerEdge->spLEEnd->position.x() - spLayerEdge->spLEHead->position.x();
	sectPoint.setX(spLayerEdge->spLEHead->position.x() + lamda * dx);
	sectPoint.setY(scanLineY);
	sectPoint.setZ(spLayerEdge->spLEHead->position.z());
	return sectPoint;
}

void Slicer::judgeFaceType(double z, QSharedPointer<Triangle> spSurface)
{
	double z_min = getZMin(spSurface);
	double z_max = getZMax(spSurface);
	double z_mid = getZMid(spSurface);
	//若三角面片与切平面相交，由于采用了摄动法，因此只有两种情况：两点在下或两点在上
	if (z_mid < z)
		spSurface->faceType = TwoPointBelow;
	else if (z_mid > z)
		spSurface->faceType = OnePointBelow;
	
}

void Slicer::judgeOtherLine(double z, QSharedPointer<Triangle> spSurface)
{
	//由于采用摄动法,三角面片与切平面的相交情况应该只有两点在切平面上方或者下方两种情况
	if ((spSurface->spSelectIntersectLine->spEdgePrev.lock()->spV1->position.z() - z)
		*(spSurface->spSelectIntersectLine->spEdgePrev.lock()->spV2->position.z() - z) < 0)
		spSurface->spOtherIntersectLine = spSurface->spSelectIntersectLine->spEdgePrev.lock();
	else
		spSurface->spOtherIntersectLine = spSurface->spSelectIntersectLine->spEdgeNext.lock();
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

void Slicer::updateColor()
{
	QSettings settings("ZJU", "scanner");
	m_color = { settings.value("sliceR",0.5f).toFloat(),settings.value("sliceG",0.3f).toFloat(),settings.value("sliceB",0.6f).toFloat() };
	m_layerHeight = settings.value("layerHeight", 0.2f).toDouble();
	update();
}

void Slicer::drawPolyLine()
{
	ShaderDrawable::update();
	showGraph();
	emit updateGraph();
}

void Slicer::convertLayertoPointCloud(double layer)
{
	if (m_sliced == false)	return;

	QVector<double> pointX;
	QVector<double> pointY;
	QVector<QSharedPointer<LayerEdge>> layerEdges;	//轮廓边
	double yScanLineMin = 10000;	//扫描线最低点
	double yScanLineMax = -10000;	//扫描线最高点

	for (auto itLayer = m_layers.begin(); itLayer != m_layers.end(); itLayer++) {
		if (qAbs((*itLayer)->z - layer)<0.0001) {
			//遍历每个轮廓线的线段
			for (auto itPolyLine = (*itLayer)->m_polyLines.begin(); itPolyLine != (*itLayer)->m_polyLines.end(); itPolyLine++) {
				for (int i = 0; i < (*itPolyLine)->m_linkPoints.size() - 1; i++) {
					//建立轮廓边表
					QSharedPointer<LayerEdge> tempLayerEdge(QSharedPointer<LayerEdge>(new LayerEdge((*itPolyLine)->m_linkPoints[i], (*itPolyLine)->m_linkPoints[i + 1])));
					layerEdges.push_back(tempLayerEdge);

					QVector3D point1((*itPolyLine)->m_linkPoints[i]->position);
					QVector3D point2((*itPolyLine)->m_linkPoints[i + 1]->position);
					//确定扫描线范围
					if (point1.y() > yScanLineMax)	yScanLineMax = point1.y();
					if (point1.y() < yScanLineMin)	yScanLineMin = point1.y();

					//如果线段大于0.15mm,密化
					if ((point2 - point1).length() > 0.15) {
						double deltX = point2.x() - point1.x();
						double deltY = point2.y() - point1.y();
						double lamb = 0.1 / (point2 - point1).length();
						QVector3D currentPoint(point1);
						while ((currentPoint - point1).length() < (point2 - point1).length()) {
							pointX.push_back(currentPoint.x());
							pointY.push_back(currentPoint.y());
							currentPoint.setX(lamb*deltX + currentPoint.x());
							currentPoint.setY(lamb*deltY + currentPoint.y());
						}
					}
					else {
						pointX.push_back(point1.x());
						pointY.push_back(point1.y());
					}
				}
			}
			break;
		}
	}
	//填充内部点
	double scanResolution = 0.1;
	double scanLine = yScanLineMin + scanResolution;
	while(scanLine < yScanLineMax) {
		QVector<QSharedPointer<LayerEdge>> intsectEdges;
		for (auto it = layerEdges.begin(); it != layerEdges.end(); it++) {
			//摄动法处理相交顶点
			if (qAbs((*it)->spLEHead->position.y() - scanLine)<0.000001)
				(*it)->spLEHead->position += QVector3D(0.0, perturbationVal, 0.0);
			if (qAbs((*it)->spLEEnd->position.y() - scanLine) < 0.000001)
				(*it)->spLEEnd->position += QVector3D(0.0, perturbationVal, 0.0);
			(*it)->sortVertex();
		}
		//判断相交
		for (auto it = layerEdges.begin(); it != layerEdges.end(); it++) {
			if ((*it)->spVMin->position.y() < scanLine && (*it)->spVMax->position.y() > scanLine)
				intsectEdges.push_back(*it);
		}
		//求交点
		QVector<QVector3D> sectPoints;
		for (auto it = intsectEdges.begin(); it != intsectEdges.end(); it++)
			sectPoints.push_back(calcuScanLineSectPoint(scanLine, *it));
		//交点按X值从小到大排序
		qSort(sectPoints.begin(), sectPoints.end(), [](const QVector3D& pointA, const QVector3D& pointB) {
			return pointA.x() < pointB.x(); });
		//填充
		for (int i = 0; i < sectPoints.size()-1; i++) {
			if ((sectPoints[i+1].x() - sectPoints[i].x()) > 0.15) {
				QVector3D currentPoint(sectPoints[i]);
				currentPoint.setX(currentPoint.x() + 0.1);
				while (currentPoint.x()<sectPoints[i+1].x()) {
					pointX.push_back(currentPoint.x());
					pointY.push_back(scanLine);
					currentPoint.setX(currentPoint.x() + 0.1);
				}
			}
			i++;
		}
		scanLine += scanResolution;
	}
	//输出
	emit stlLayerToPointCloud(pointX, pointY, layer);
}

bool Slicer::updateData()
{
	m_triangles.clear();
	m_lines.clear();
	//QVector3D color = { 0.5f,0.3f,0.6f };
	//绘制轮廓线
	for (auto itLayer = m_layers.begin(); itLayer != m_layers.end(); itLayer++) {
		for (auto itPolyLine = (*itLayer)->m_polyLines.begin(); itPolyLine != (*itLayer)->m_polyLines.end(); itPolyLine++) {
			for (int i = 0; i < (*itPolyLine)->m_linkPoints.size() - 1; i++) {
				QVector3D point1((*itPolyLine)->m_linkPoints[i]->position);
				QVector3D point2((*itPolyLine)->m_linkPoints[i+1]->position);
				m_lines.append({ point1,m_color,m_vectorNaN });
				m_lines.append({ point2,m_color,m_vectorNaN });
			}
		}
	}

	return true;
}
