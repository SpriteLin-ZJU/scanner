#include "shaderdrawable.h"

ShaderDrawable::ShaderDrawable()
{
	m_lineWidth = 1.0;
	m_pointSize = 1.0;
}

ShaderDrawable::~ShaderDrawable()
{
}

void ShaderDrawable::init(QOpenGLShaderProgram * shaderProgram)
{
	m_vao.create();
	m_vao.bind();
	m_vbo.create();
	m_vbo.bind();

	quintptr offset = 0;

	// Tell OpenGL programmable pipeline how to locate vertex position data
	int vertexLocation = shaderProgram->attributeLocation("aPos");
	shaderProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));
	shaderProgram->enableAttributeArray(vertexLocation);

	offset = sizeof(QVector3D);

	// Tell OpenGL programmable pipeline how to locate vertex color data
	int colorLocation = shaderProgram->attributeLocation("aColor");
	shaderProgram->setAttributeBuffer(colorLocation, GL_FLOAT, offset, 3, sizeof(VertexData));
	shaderProgram->enableAttributeArray(colorLocation);

	m_vao.release();
}

bool ShaderDrawable::updateData()
{
	return true;
}

void ShaderDrawable::draw(QOpenGLShaderProgram * shaderProgram)
{
	m_vao.bind();
	if (!m_lines.isEmpty()) {
		glLineWidth(m_lineWidth);
		glDrawArrays(GL_LINES, 0, m_lines.count());
	}
	if (!m_points.isEmpty()) {
		glPointSize(m_pointSize);
		glDrawArrays(GL_POINTS, m_lines.count(), m_points.count());
	}
	m_vao.release();
}

void ShaderDrawable::updateGeometry(QOpenGLShaderProgram * shaderProgram)
{
	if (!m_vao.isCreated()) init(shaderProgram);

	if (updateData()) {
		QVector<VertexData> vertexData(m_lines);
		vertexData += m_points;
		m_vbo.allocate(vertexData.constData(), vertexData.count() * sizeof(VertexData));
	}
}
