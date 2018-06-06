#pragma once

#include <QObject>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenglVertexArrayObject>

struct VertexData
{
	QVector3D position;
	QVector3D color;
};

class ShaderDrawable :protected QOpenGLFunctions
{
public:
	explicit ShaderDrawable();
	~ShaderDrawable();

	void draw(QOpenGLShaderProgram *shaderProgram);
	void updateGeometry(QOpenGLShaderProgram *shaderProgram = 0);


protected:

	double m_lineWidth;
	double m_pointSize;
	QVector<VertexData> m_lines;
	QVector<VertexData> m_points;
	QVector<VertexData> m_triangles;
	
	QOpenGLBuffer m_vbo;//Protected for direct vbp access 子类可以直接访问

	virtual bool updateData();
	void init(QOpenGLShaderProgram *shaderProgram = 0);

private:
	QOpenGLVertexArrayObject m_vao;
};