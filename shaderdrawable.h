#pragma once

#include <QObject>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenglVertexArrayObject>

#define sNan 65536
struct VertexData
{
	QVector3D position;
	QVector3D color;
	QVector3D normal;
};

class ShaderDrawable :protected QOpenGLFunctions
{
public:
	explicit ShaderDrawable();
	~ShaderDrawable();

	void update();
	void draw(QOpenGLShaderProgram *shaderProgram);
	bool needsUpdateGeometry() const;
	void updateGeometry(QOpenGLShaderProgram *shaderProgram = 0);


protected:
	double m_lineWidth;
	double m_pointSize;
	QVector<VertexData> m_lines;
	QVector<VertexData> m_points;
	QVector<VertexData> m_triangles;
	
	QVector3D m_vectorNaN = { sNan,0,0 };

	QOpenGLBuffer m_vbo;//Protected for direct vbp access �������ֱ�ӷ���

	virtual bool updateData();
	void init(QOpenGLShaderProgram *shaderProgram = 0);

private:
	bool m_needsUpdateGeometry;
	QOpenGLVertexArrayObject m_vao;
};