#include "glwidget.h"

#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <math.h>




GLWidget::GLWidget(QWidget* parent)
	:QOpenGLWidget(parent), 
	m_program(0),
	m_vbo(QOpenGLBuffer::VertexBuffer),
	m_ebo(QOpenGLBuffer::IndexBuffer)
{
}

GLWidget::~GLWidget()
{
	cleanup();
}

static void qNormalizeAngle(int &angle)
{
	while (angle < 0)
		angle += 360 * 16;
	while (angle > 360 * 16)
		angle -= 360 * 16;
}

void GLWidget::setXRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != m_xRot) {
		m_xRot = angle;
		update();
	}
}

void GLWidget::setYRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != m_yRot) {
		m_yRot = angle;
		update();
	}
}

void GLWidget::setZRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != m_zRot) {
		m_zRot = angle;
		update();
	}
}

void GLWidget::cleanup()
{
	makeCurrent();
	m_vbo.destroy();
	delete m_program;
	m_program = 0;
	doneCurrent();
}

//顶点着色器
static const char *vertexShaderSource =
"#version 430 core\n"
"in vec3 aPos;\n"
"uniform mat4 projMatrix;\n"
"uniform mat4 viewMatrix;\n"
"uniform mat4 modelMatrix;\n"
"void main() {\n"
"    gl_Position = projMatrix * viewMatrix * modelMatrix * vec4(aPos,1.0);\n"
"}\n";

//片段着色器
static const char *fragmentShaderSource =
"#version 430 core\n"
"out vec4 FragColor;\n"
"void main() {\n"
"   FragColor = vec4(1.0,0.0,0.0,1.0);\n"
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
	m_program->bind();
	m_projMatrixLoc = m_program->uniformLocation("projMatrix");
	m_viewMatrixLoc = m_program->uniformLocation("viewMatrix");
	m_modelMatrixLoc = m_program->uniformLocation("modelMatrix");

	//创建VAO
	m_vao.create();
	m_vao.bind();
	
	//创建顶点缓冲对象VBO
	m_vbo.create();
	m_vbo.bind();
	
	m_program->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(GLfloat) * 3);		//两种方式均可
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_TRUE, 3 * sizeof(GLfloat), 0);
	m_program->enableAttributeArray(0);
	
	// Our camera never changes in this example.
	m_camera.setToIdentity();
	m_camera.translate(0, 0, -100);

	//创建EBO
	//m_ebo.create();
	//m_ebo.bind();

	m_vao.release();
	m_program->release();
}

void GLWidget::resizeGL(int width, int height)
{
	glViewport(0, 0, width, height);
	m_proj.setToIdentity();
	//m_proj.perspective(45.0f, GLfloat(width) / height, 0.01f, 200.0f);
	m_proj.ortho(-40.0f, 40.0f, -50.0f, 50.0f, 0.1f, 200.0f);
}

void GLWidget::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glPointSize(2);

	m_model.setToIdentity();
	m_model.rotate(180.0f - (m_xRot / 16.0f), 1, 0, 0);
	m_model.rotate(m_yRot / 16.0f, 0, 1, 0);
	m_model.rotate(m_zRot / 16.0f, 0, 0, 1);

	m_program->bind();
	m_program->setUniformValue(m_projMatrixLoc, m_proj);
	m_program->setUniformValue(m_viewMatrixLoc, m_camera);
	m_program->setUniformValue(m_modelMatrixLoc, m_model);



	m_vao.bind();

	glDrawArrays(GL_POINTS, 0, (GLsizei)m_profileCount*m_resolution);

	m_vao.release();
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
			vertices.push_back(i);
			vertices.push_back(vdValueZ[i*resolution + j]-zMid);
		}
	}
	
	//更新索引数组数据
	

	//更新vbo数据
	if(!m_vbo.bind())
		return;
	m_vbo.allocate(vertices.constData(), m_profileCount*m_resolution*sizeof(GLfloat));
	
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
	m_lastPos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - m_lastPos.x();
	int dy = event->y() - m_lastPos.y();

	if (event->buttons() & Qt::LeftButton) {
		setXRotation(m_xRot + 8 * dy);
		setYRotation(m_yRot + 8 * dx);
	}
	else if (event->buttons() & Qt::RightButton) {
		setXRotation(m_xRot + 8 * dy);
		setZRotation(m_zRot + 8 * dx);
	}
	m_lastPos = event->pos();
}

