#include "glwidget.h"

#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>


GLWidget::GLWidget(QWidget* parent)
	:QOpenGLWidget(parent), m_program(0)
{
	
}

GLWidget::~GLWidget()
{
	cleanup();
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
"void main() {\n"
"    gl_Position = vec4(aPos,1.0);\n"
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
	glClearColor(0, 0, 0, 1);

	m_program = new QOpenGLShaderProgram;
	if (!m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource))
		return;
	if (!m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource))
		return;
	m_program->bindAttributeLocation("aPos", 0);
	if (!m_program->link())
		return;

	//创建顶点缓冲对象VBO
	m_vbo.create();
	m_vbo.bind();

}

void GLWidget::resizeGL(int width, int height)
{
}

void GLWidget::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	m_program->bind();
	m_vbo.bind();
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_TRUE, 3 * sizeof(GLfloat), 0);

	glDrawArrays(GL_TRIANGLES, 0, m_profileCount*m_resolution);

	m_program->release();

}

void GLWidget::init_vbo(unsigned int resolution)
{
	//更新顶点数据
	QVector<GLfloat> vertices;
	m_profileCount = vdValueX.size() / resolution;
	m_resolution = resolution;

	for (int i = 0; i < m_profileCount; i++) {
		for (int j = 0; j < resolution; j++) {
			vertices.push_back(vdValueX[i*resolution + j]);
			vertices.push_back(i);
			vertices.push_back(vdValueZ[i*resolution + j]);
		}
	}

	m_vbo.bind();
	m_vbo.allocate(vertices.constData(), vertices.count() * sizeof(GLfloat));

}

