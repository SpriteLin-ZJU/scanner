#include "glwidget.h"

#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <math.h>
#include <QDebug>

QVector<GLfloat> netVertices;
void fillNetVertices()
{
	for (int i = 0; i <= 10; i++) {
		GLfloat y = -50.0f + i * 10.0f;
		netVertices.push_back(-30.0f);
		netVertices.push_back(y);
		netVertices.push_back(0.0f);
		netVertices.push_back(30.0f);
		netVertices.push_back(y);
		netVertices.push_back(0.0f);
	}
	for (int i = 0; i <= 5; i++) {
		GLfloat z = i * 10.0f;
		netVertices.push_back(-30.0f);
		netVertices.push_back(-50.0f);
		netVertices.push_back(z);
		netVertices.push_back(30.0f);
		netVertices.push_back(-50.0f);
		netVertices.push_back(z);
	}
	//平行于y轴的网格
	for (int i = 0; i <= 6; i++) {
		GLfloat x = -30.0f + i * 10.0f;
		netVertices.push_back(x);
		netVertices.push_back(-50.0f);
		netVertices.push_back(0.0f);
		netVertices.push_back(x);
		netVertices.push_back(50.0f);
		netVertices.push_back(0.0f);
	}
	for (int i = 0; i <= 5; i++) {
		GLfloat z = i * 10.0f;
		netVertices.push_back(-30.0f);
		netVertices.push_back(-50.0f);
		netVertices.push_back(z);
		netVertices.push_back(-30.0f);
		netVertices.push_back(50.0f);
		netVertices.push_back(z);
	}
	//平行于z轴的网格
	for (int i = 0; i <= 6; i++) {
		GLfloat x = -30.0f + i * 10.0f;
		netVertices.push_back(x);
		netVertices.push_back(-50.0f);
		netVertices.push_back(0.0f);
		netVertices.push_back(x);
		netVertices.push_back(-50.0f);
		netVertices.push_back(50.0f);
	}
	for (int i = 0; i <= 10; i++) {
		GLfloat y = -50.0f + i * 10.0f;
		netVertices.push_back(-30.0f);
		netVertices.push_back(y);
		netVertices.push_back(0.0f);
		netVertices.push_back(-30.0f);
		netVertices.push_back(y);
		netVertices.push_back(50.0f);
	}
}

GLWidget::GLWidget(QWidget* parent)
	:QOpenGLWidget(parent), 
	m_program(0),
	m_vbo(QOpenGLBuffer::VertexBuffer),
	m_ebo(QOpenGLBuffer::IndexBuffer),
	m_netVbo(QOpenGLBuffer::VertexBuffer)
{
	m_xRot = 90;
	m_yRot = 0;

	m_zoom = 1;

	m_xPan = 0;
	m_yPan = 0;
	m_distance = 200;

	m_xLookAt = 0;
	m_yLookAt = 0;
	m_zLookAt = 0;

	updateProjection();
	updateView();
}

GLWidget::~GLWidget()
{
	cleanup();
}

void GLWidget::cleanup()
{
	makeCurrent();
	m_vbo.destroy();
	m_netVbo.destroy();
	delete m_program;
	m_program = 0;
	doneCurrent();
}

//顶点着色器
static const char *vertexShaderSource =
"#version 430 core\n"
"in vec3 aPos;\n"
"uniform mat4 mvpMatrix;\n"
"uniform mat4 mvMatrix;\n"
"void main() {\n"
"    gl_Position = mvpMatrix * vec4(aPos,1.0);\n"
"}\n";

//片段着色器
static const char *fragmentShaderSource =
"#version 430 core\n"
"out vec4 FragColor;\n"
"uniform vec4 ourColor;\n"
"void main() {\n"
"   FragColor = ourColor;\n"
"}\n";


void GLWidget::initializeGL()
{
	initializeOpenGLFunctions();
	glEnable(GL_DEPTH_TEST);
	glClearColor(1, 1, 1, 1);


	//编译并链接着色器
	m_program = new QOpenGLShaderProgram();
	if (!m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource))
		return;
	if (!m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource))
		return;
	m_program->bindAttributeLocation("aPos", 0);
	if (!m_program->link())
		return;
	
	//获取着色器属性位置
	m_program->bind();
	m_mvpMatrixLoc = m_program->uniformLocation("mvpMatrix");
	m_mvMatrixLoc = m_program->uniformLocation("mvMatrix");
	m_colorLoc = m_program->uniformLocation("ourColor");

	//创建点云VAO
	m_vao.create();
	m_vao.bind();
	//创建顶点缓冲对象VBO
	m_vbo.create();
	m_vbo.bind();
	m_program->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(GLfloat) * 3);		//两种方式均可
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_TRUE, 3 * sizeof(GLfloat), 0);
	m_program->enableAttributeArray(0);
	m_vao.release();

	//创建网格
	fillNetVertices();
	m_netVao.create();
	m_netVao.bind();
	m_netVbo.create();
	m_netVbo.bind();
	m_netVbo.allocate(netVertices.constData(), netVertices.size() * sizeof(GLfloat));
	m_program->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(GLfloat) * 3);
	m_program->enableAttributeArray(0);
	m_netVao.release();

	m_program->release();
}

void GLWidget::resizeGL(int width, int height)
{
	glViewport(0, 0, width, height);
	updateProjection();
}

void GLWidget::updateProjection()
{
	m_projectionMatrix.setToIdentity();
	double asp = (double)width() / height();
	//m_projectionMatrix.perspective(45.0f, GLfloat(asp), 0.01f, 300.0f);
	m_projectionMatrix.frustum((-0.5 + m_xPan) * asp, (0.5 + m_xPan) * asp, -0.5 + m_yPan, 0.5 + m_yPan, 2, m_distance * 2);
	update();
}

void GLWidget::updateView()
{
	m_viewMatrix.setToIdentity();

	double r = m_distance;
	double angX = M_PI / 180 * m_xRot;
	double angY = M_PI / 180 * m_yRot;

	QVector3D eye(r * cos(angX) * sin(angY) + m_xLookAt, r * sin(angX) + m_yLookAt, r * cos(angX) * cos(angY) + m_zLookAt);
	QVector3D center(m_xLookAt, m_yLookAt, m_zLookAt);
	QVector3D up(fabs(m_xRot) == 90 ? -sin(angY + (m_xRot < 0 ? M_PI : 0)) : 0, cos(angX), fabs(m_xRot) == 90 ? -cos(angY + (m_xRot < 0 ? M_PI : 0)) : 0);

	m_viewMatrix.lookAt(eye, center, up.normalized());

	m_viewMatrix.translate(m_xLookAt, m_yLookAt, m_zLookAt);
	m_viewMatrix.scale(m_zoom, m_zoom, m_zoom);
	m_viewMatrix.translate(-m_xLookAt, -m_yLookAt, -m_zLookAt);

	m_viewMatrix.rotate(-90, 1.0, 0.0, 0.0);
	update();
}

void GLWidget::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glPointSize(2);


	m_program->bind();
	m_program->setUniformValue(m_mvpMatrixLoc, m_projectionMatrix * m_viewMatrix);
	m_program->setUniformValue(m_mvMatrixLoc, m_viewMatrix);

	//绘制点云数据
	m_vao.bind();
	m_program->setUniformValue(m_colorLoc, 1.0f, 0.0f, 0.0f, 1.0f);
	glDrawArrays(GL_POINTS, 0, (GLsizei)m_profileCount*m_resolution);
	m_vao.release();

	//绘制坐标轴-网格
	m_netVao.bind();
	m_program->setUniformValue(m_colorLoc, 0.0f, 0.0f, 0.0f, 0.5f);
	glDrawArrays(GL_LINES, 0, (GLsizei)96);
	m_netVao.release();

	m_program->release();

}

void GLWidget::init_vbo(unsigned int resolution)
{
	//更新顶点数据
	QVector<GLfloat> vertices;

	m_profileCount = vdValueX.size() / resolution;
	m_resolution = resolution;

	double zMin, zMax;
	bool firstFlag=false;
	for (auto it = vdValueZ.cbegin(); it != vdValueZ.cend(); it++) {
		if (*it >= 70.0 && *it <= 120.0) {
			//确定第一个有效数据
			if (!firstFlag) {
				zMin = zMax = *it;
				firstFlag = true;
			}
			//找出最大最小值
			if (*it < zMin) 
				zMin = *it;
			else if (*it > zMin)
				zMax = *it;
		}
	}

	double zMid = (zMax + zMin) / 2.0;
	for (int i = 0; i < m_profileCount; i++) {
		for (int j = 0; j < resolution; j++) {
			vertices.push_back(vdValueX[i*resolution + j]);
			vertices.push_back(i-m_profileCount/2.0);
			vertices.push_back(vdValueZ[i*resolution + j]-zMid);
		}
	}
	
	//更新索引数组数据
	

	//更新vbo数据
	if(!m_vbo.bind())
		return;
	m_vbo.allocate(vertices.constData(), 3 * m_profileCount*m_resolution * sizeof(GLfloat));
	
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
	m_lastPos = event->pos();
	m_xLastRot = m_xRot;
	m_yLastRot = m_yRot;
	m_xLastPan = m_xPan;
	m_yLastPan = m_yPan;
	qDebug("%02f", m_yRot);
	qDebug("%02f", m_xLookAt);
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
	if ((event->buttons() & Qt::MiddleButton && !(event->modifiers() & Qt::ShiftModifier)) || event->buttons() & Qt::LeftButton) {

		//stopViewAnimation();

		//m_yRot = m_yLastRot - (event->x() - m_lastPos.x()) * 0.5;
		m_yRot = normalizeAngle( m_yLastRot - (event->x() - m_lastPos.x()) * 0.5);
		qDebug ("%02f",m_yRot);
		m_xRot = m_xLastRot + (event->y() - m_lastPos.y()) * 0.5;

		if (m_xRot < -90) m_xRot = -90;
		if (m_xRot > 90) m_xRot = 90;

		updateView();
		emit rotationChanged();
	}

	if ((event->buttons() & Qt::MiddleButton && event->modifiers() & Qt::ShiftModifier) || event->buttons() & Qt::RightButton) {
		m_xPan = m_xLastPan - (event->pos().x() - m_lastPos.x()) * 1 / (double)width();
		m_yPan = m_yLastPan + (event->pos().y() - m_lastPos.y()) * 1 / (double)height();

		updateProjection();
	}
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
	if (m_zoom > 0.1 && event->delta() < 0) {
		m_xPan -= ((double)event->pos().x() / width() - 0.5 + m_xPan) * (1 - 1 / ZOOM_STEP);
		m_yPan += ((double)event->pos().y() / height() - 0.5 - m_yPan) * (1 - 1 / ZOOM_STEP);

		m_zoom /= ZOOM_STEP;
	}
	else if (m_zoom < 10 && event->delta() > 0)
	{
		m_xPan -= ((double)event->pos().x() / width() - 0.5 + m_xPan) * (1 - ZOOM_STEP);
		m_yPan += ((double)event->pos().y() / height() - 0.5 - m_yPan) * (1 - ZOOM_STEP);

		m_zoom *= ZOOM_STEP;
	}

	updateProjection();
	updateView();
}

double GLWidget::normalizeAngle(double angle)
{
	while (angle < 0)	angle += 360;
	while (angle > 360) angle -= 360;

	return angle;
}

