#include "shaderdrawable.h"

ShaderDrawable::ShaderDrawable()
{
	m_needsUpdateGeometry = true;
	m_canSee = true;
	m_lineWidth = 2.0;
	m_pointSize = 1.0;
}

ShaderDrawable::~ShaderDrawable()
{
}

void ShaderDrawable::init(QOpenGLShaderProgram * shaderProgram)
{
	//注意！init函数内(创建vao或vbo之前)必须添加该函数.
	initializeOpenGLFunctions();

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

	offset += sizeof(QVector3D);

	// Tell OpenGL programmable pipeline how to locate vertex normal data
	int normalLocation = shaderProgram->attributeLocation("aNormal");
	shaderProgram->setAttributeBuffer(normalLocation, GL_FLOAT, offset, 3, sizeof(VertexData));
	shaderProgram->enableAttributeArray(normalLocation);

	m_vao.release();
}

bool ShaderDrawable::updateData()
{
	return true;
}

void ShaderDrawable::update()
{
	m_needsUpdateGeometry = true;
}

bool ShaderDrawable::canSee()
{
	return m_canSee;
}

void ShaderDrawable::showGraph()
{
	m_canSee = true;
}

void ShaderDrawable::hideGraph()
{
	m_canSee = false;
}

bool ShaderDrawable::needsUpdateGeometry() const
{
	return m_needsUpdateGeometry;
}

void ShaderDrawable::draw(QOpenGLShaderProgram * shaderProgram)
{
	m_vao.bind();
	if (!m_triangles.isEmpty()) {
		glDrawArrays(GL_TRIANGLES, 0, m_triangles.count());
	}
	if (!m_lines.isEmpty()) {
		glLineWidth(m_lineWidth);
		glDrawArrays(GL_LINES, m_triangles.count(), m_lines.count());
	}
	if (!m_points.isEmpty()) {
		glPointSize(m_pointSize);
		glDrawArrays(GL_POINTS, m_triangles.count()+m_lines.count(), m_points.count());
	}
	m_vao.release();
}



void ShaderDrawable::updateGeometry(QOpenGLShaderProgram * shaderProgram)
{
	if (!m_vao.isCreated()) init(shaderProgram);

	m_vbo.bind();
	if (updateData()) {
		QVector<VertexData> vertexData(m_triangles);
		vertexData += m_lines;
		vertexData += m_points;
		m_vbo.allocate(vertexData.constData(), vertexData.count() * sizeof(VertexData));
	}

	m_vbo.release();
	m_needsUpdateGeometry = false;
}
