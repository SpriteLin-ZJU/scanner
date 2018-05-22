#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenglVertexArrayObject>
#include <QMatrix4x4>


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

	void setXRotation(int angle);
	void setYRotation(int angle);
	void setZRotation(int angle);
	void cleanup();

	void updateGraph(unsigned int resolution) {
		init_vbo(resolution);
		update();
	}

protected:
	void initializeGL() override;
	void resizeGL(int width, int height) override;
	void paintGL() override;

	void mousePressEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;

private:
	QOpenGLShaderProgram * m_program;
	QOpenGLVertexArrayObject m_vao;
	QOpenGLBuffer m_vbo;
	QOpenGLBuffer m_ebo;

	QOpenGLVertexArrayObject m_netVao;
	QOpenGLBuffer m_netVbo;

	int m_xRot;
	int m_yRot;
	int m_zRot;
	QPoint m_lastPos;
	int m_projMatrixLoc;
	int m_viewMatrixLoc;
	int m_modelMatrixLoc;
	int m_colorLoc;
	int m_normalMatrixLoc;
	int m_lightPosLoc;
	QMatrix4x4 m_proj;
	QMatrix4x4 m_camera;
	QMatrix4x4 m_model;

	void init_vbo(unsigned int resolution);

	int m_profileCount;
	unsigned int m_resolution;
};