#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenglVertexArrayObject>

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram)
QT_FORWARD_DECLARE_CLASS(QOpenGLTexture)

//test

extern std::vector<double>vdValueX;
extern std::vector<double>vdValueZ;

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT
public:
	explicit GLWidget(QWidget *parent = Q_NULLPTR);
	~GLWidget();

	void cleanup();
	void updateGraph(unsigned int resolution) {
		init_vbo(resolution);
		update();
	}

protected:
	void initializeGL() override;
	void resizeGL(int width, int height) override;
	void paintGL() override;

private:
	QOpenGLVertexArrayObject m_vao;
	QOpenGLBuffer m_vbo;
	QOpenGLBuffer m_ebo;

	QOpenGLShaderProgram* m_program;
	void init_vbo(unsigned int resolution);

	int m_profileCount;
	unsigned int m_resolution;
};