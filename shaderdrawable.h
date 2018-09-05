#pragma once

#include <QObject>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenglVertexArrayObject>

#define sNan 65536.0
struct VertexData
{
	QVector3D position;
	QVector3D color;
	QVector3D normal;
};

class ShaderDrawable :public QObject, protected QOpenGLFunctions
{
	Q_OBJECT
public:
	explicit ShaderDrawable();
	~ShaderDrawable();

	void update();
	bool canSee();
	void showGraph();
	void hideGraph();
	void draw(QOpenGLShaderProgram *shaderProgram);
	bool needsUpdateGeometry() const;
	void updateGeometry(QOpenGLShaderProgram *shaderProgram = 0);
signals:
	void updateGraph();
protected:
	double m_lineWidth;
	double m_pointSize;
	QVector<VertexData> m_lines;
	QVector<VertexData> m_points;
	QVector<VertexData> m_triangles;
	
	QVector3D m_vectorNaN = { sNan,0,0 };

	QOpenGLBuffer m_vbo;//Protected for direct vbp access 子类可以直接访问

	virtual bool updateData();
	void init(QOpenGLShaderProgram *shaderProgram = 0);

private:
	bool m_needsUpdateGeometry;
	bool m_canSee;
	QOpenGLVertexArrayObject m_vao;
};