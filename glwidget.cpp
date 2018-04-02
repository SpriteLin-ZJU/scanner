#include "glwidget.h"

#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>


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

void GLWidget::cleanup()
{
	makeCurrent();
	m_vbo.destroy();
	delete m_program;
	m_program = 0;
	doneCurrent();
}

//������ɫ��
static const char *vertexShaderSource =
"#version 430 core\n"
"in vec3 aPos;\n"
"void main() {\n"
"    gl_Position = vec4(aPos,1.0);\n"
"}\n";

//Ƭ����ɫ��
static const char *fragmentShaderSource =
"#version 430 core\n"
"out vec4 FragColor;\n"
"void main() {\n"
"   FragColor = vec4(1.0,0.0,0.0,1.0);\n"
"}\n";


void GLWidget::initializeGL()
{
	initializeOpenGLFunctions();
	glClearColor(1, 1, 1, 1);
	
	//���벢������ɫ��
	m_program = new QOpenGLShaderProgram();
	if (!m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource))
		return;
	if (!m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource))
		return;
	m_program->bindAttributeLocation("aPos", 0);
	if (!m_program->link())
		return;
	m_program->bind();

	//����VAO
	m_vao.create();
	m_vao.bind();
	
	//�������㻺�����VBO
	m_vbo.create();
	m_vbo.bind();
	//....
	float vertices[] = {
		-0.5f, -0.5f, 0.0f,
		0.5f, -0.5f, 0.0f,
		0.0f,  0.5f, 0.0f
	};
	m_vbo.allocate(vertices, sizeof(vertices));
	m_program->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(GLfloat) * 3);		//���ַ�ʽ����
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
	m_program->enableAttributeArray(0);
	
	//����EBO
	m_ebo.create();
	m_ebo.bind();
	unsigned int index[] = {
		0,1,2
	};
	m_ebo.allocate(index, sizeof(index));

	m_vao.release();
}

void GLWidget::resizeGL(int width, int height)
{
	//glViewport(0, 0, width, height);
}

void GLWidget::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);

	m_program->bind();
	m_vao.bind();

	//glDrawArrays(GL_TRIANGLES, 0, 3);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
	m_vao.release();
	m_program->release();

}

void GLWidget::init_vbo(unsigned int resolution)
{
	//���¶�������
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
	
	//����������������


	//����vao
	m_vao.bind();

	//����vbo����
	m_vbo.bind();
	m_vbo.allocate(vertices.constData(), m_profileCount*m_resolution*sizeof(GLfloat));
	m_vbo.release();
	m_program->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(GLfloat) * 3);	//�趨��������ָ�룬�൱��glVertexAttribPointer(...)
	m_program->enableAttributeArray(0);
	
	//����ebo����
	m_ebo.bind();
	//m_ebo.allocate(indices, sizeof(indices));
	
	m_vao.release();
}

