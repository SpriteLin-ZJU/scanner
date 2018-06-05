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
		updateScannerVbo(resolution);
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
	QOpenGLVertexArrayObject m_scannerVao;
	QOpenGLBuffer m_scannerVbo;
	QOpenGLBuffer m_scannerEbo;

	QOpenGLVertexArrayObject m_netVao;
	QOpenGLBuffer m_netVbo;

	double m_xRot=45, m_yRot=135, m_xLastRot=0, m_yLastRot=0;
	double m_xPan=0, m_yPan=0.1, m_xLastPan=0, m_yLastPan=0;
	double m_xLookAt=0, m_yLookAt=0, m_zLookAt=0;
	QPoint m_lastPos;
	double m_zoom=0.8;
	double m_distance=200;

	QVector<GLfloat> netVertices;

	int m_mvpMatrixLoc=0;
	int m_mvMatrixLoc=0;
	int m_colorLoc=0;
	int m_normalMatrixLoc=0;
	int m_lightPosLoc=0;
	QMatrix4x4 m_projectionMatrix;
	QMatrix4x4 m_viewMatrix;

	void creatScannerVao();
	void creatNetVao();
	void fillNetVertices();

	void updateScannerVbo(unsigned int resolution);
	double normalizeAngle(double angle);
	void updateProjection();
	void updateView();

	int m_profileCount;
	unsigned int m_resolution;
};