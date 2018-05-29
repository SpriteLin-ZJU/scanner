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
	void cleanup();

	void updateGraph(unsigned int resolution) {
		init_vbo(resolution);
		update();
	}

signals:
	void rotationChanged();

protected:
	void initializeGL() override;
	void resizeGL(int width, int height) override;
	void paintGL() override;

	void mousePressEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event) override;

private:
	const double M_PI = 3.14159265358979323846;
	const double ZOOM_STEP = 1.1;

	QOpenGLShaderProgram * m_program;
	QOpenGLVertexArrayObject m_vao;
	QOpenGLBuffer m_vbo;
	QOpenGLBuffer m_ebo;

	QOpenGLVertexArrayObject m_netVao;
	QOpenGLBuffer m_netVbo;

	double m_xRot, m_yRot, m_xLastRot, m_yLastRot;
	double m_xPan, m_yPan, m_xLastPan, m_yLastPan;
	double m_xLookAt, m_yLookAt, m_zLookAt;
	QPoint m_lastPos;
	double m_zoom;
	double m_distance;

	int m_mvpMatrixLoc;
	int m_mvMatrixLoc;
	int m_colorLoc;
	int m_normalMatrixLoc;
	int m_lightPosLoc;
	QMatrix4x4 m_projectionMatrix;
	QMatrix4x4 m_viewMatrix;

	void init_vbo(unsigned int resolution);
	double normalizeAngle(double angle);
	void updateProjection();
	void updateView();

	int m_profileCount;
	unsigned int m_resolution;
};