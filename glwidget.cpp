#include "glwidget.h"

#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <math.h>
#include <QDebug>

GLWidget::GLWidget(QWidget* parent)
	:QOpenGLWidget(parent), 
	m_program(0)
	//m_scannerVbo(QOpenGLBuffer::VertexBuffer),
	//m_scannerEbo(QOpenGLBuffer::IndexBuffer)
{
	updateProjection();
	updateView();
}

GLWidget::~GLWidget()
{
	cleanup();
}

void GLWidget::cleanup()
{
	makeCurrent();
	//m_scannerVbo.destroy();
	delete m_program;
	m_program = 0;
	doneCurrent();
}

//顶点着色器
static const char *vertexShaderSource =
"#version 430 core\n"
"in vec3 aPos;\n"
"in vec3 aColor;\n"
"in vec3 aNormal;\n"
"out vec3 ObjColor;\n"
"out vec3 Normal;\n"
"out vec3 FragPos;\n"
"out vec3 LightPos;\n"
"out vec3 LightsArray[5];\n"
"uniform mat4 mvpMatrix;\n"
"uniform mat4 mvMatrix;\n"
"uniform mat3 normalMatrix;\n"
"uniform vec3 lightPos;\n"

"bool isNan(float val){\n"
"	 return (val > 65535.0);\n"
"}\n"

"void main() {\n"
"    gl_Position = mvpMatrix * vec4(aPos,1.0);\n"
"	 ObjColor = aColor;\n"
"	 if( isNan(aNormal.x) )\n"
"		Normal = aNormal;\n"
"	 else{\n"
"		Normal = normalMatrix * aNormal;\n"
"		FragPos = vec3(mvMatrix * vec4(aPos,1.0));\n"
"		LightPos = vec3(mvMatrix * vec4(lightPos, 1.0));\n"
"		LightsArray[0]=LightPos;\n"
"	 }\n"
"}\n";

//片段着色器
static const char *fragmentShaderSource =
"#version 430 core\n"
"out vec4 FragColor;\n"
"in vec3 ObjColor;\n"
"in vec3 Normal;\n"
"in vec3 FragPos;\n"
"in vec3 LightPos;\n"

"bool isNan(float val){\n"
"	 return (val > 65535.0);\n"
"}\n"

"void main() {\n"
"	if( isNan(Normal.x) )\n"
"		FragColor = vec4(ObjColor,1.0);\n"
"	else{\n"
"		vec3 lightColor = vec3(1.0,1.0,1.0);\n"
"		float ambientStrength=0.4;\n"
"		vec3 ambient = ambientStrength * lightColor;\n"
"		\n"
"		vec3 norm = normalize(Normal);\n"
"		vec3 lightDir = normalize(LightPos-FragPos);\n"
"		float diff = max( dot( norm, lightDir ), 0.0 );\n"
"		float diffuseStrength = 0.5;\n"
"		vec3 diffuse = diffuseStrength * diff * lightColor;\n"
"		\n"
"		float specularStrength = 0.3;\n"
"		vec3 viewDir = normalize(-FragPos);\n"
"		vec3 reflectDir = reflect(-lightDir, norm);\n"
"		float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);\n"
"		vec3 specular = specularStrength * spec * lightColor;\n"
"		\n"
"		vec3 result = (ambient + diffuse + specular) * ObjColor;\n"
"		FragColor = vec4(result, 1.0);\n"
"	}\n"
"}\n";


void GLWidget::addDrawable(ShaderDrawable * drawable)
{
	m_shaderDrawableList.append(drawable);
}

void GLWidget::initializeGL()
{
	initializeOpenGLFunctions();
	glEnable(GL_DEPTH_TEST);
	glClearColor(1, 1, 1, 1);


	//编译并链接着色器
	m_program = new QOpenGLShaderProgram();
	if (!m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource))
		return;
	if (!m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource))
		return;
	if (!m_program->link())
		return;
	
	//获取着色器属性位置
	m_program->bind();
	m_mvpMatrixLoc = m_program->uniformLocation("mvpMatrix");
	m_mvMatrixLoc = m_program->uniformLocation("mvMatrix");
	m_normalMatrixLoc = m_program->uniformLocation("normalMatrix");
	m_lightPosLoc = m_program->uniformLocation("lightPos");

	//创建点云VAO
	//creatScannerVao();
	//创建

	m_program->release();
}

void GLWidget::resizeGL(int width, int height)
{
	glViewport(0, 0, width, height);
	updateProjection();
}

void GLWidget::updateProjection()
{
	m_projectionMatrix.setToIdentity();
	double asp = (double)width() / height();
	m_projectionMatrix.frustum((-0.5 + m_xPan) * asp, (0.5 + m_xPan) * asp, -0.5 + m_yPan, 0.5 + m_yPan, 2, m_distance * 2);
	update();
}

void GLWidget::updateView()
{
	m_viewMatrix.setToIdentity();

	double r = m_distance;
	double angX = M_PI / 180 * m_xRot;
	double angY = M_PI / 180 * m_yRot;

	QVector3D eye(r * cos(angX) * sin(angY) + m_xLookAt, r * sin(angX) + m_yLookAt, r * cos(angX) * cos(angY) + m_zLookAt);
	QVector3D center(m_xLookAt, m_yLookAt, m_zLookAt);
	QVector3D up(fabs(m_xRot) == 90 ? -sin(angY + (m_xRot < 0 ? M_PI : 0)) : 0, cos(angX), fabs(m_xRot) == 90 ? -cos(angY + (m_xRot < 0 ? M_PI : 0)) : 0);

	m_viewMatrix.lookAt(eye, center, up.normalized());

	m_viewMatrix.translate(m_xLookAt, m_yLookAt, m_zLookAt);
	m_viewMatrix.scale(m_zoom, m_zoom, m_zoom);
	m_viewMatrix.translate(-m_xLookAt, -m_yLookAt, -m_zLookAt);

	m_viewMatrix.rotate(-90, 1.0, 0.0, 0.0);

	//计算view矩阵的左上角的逆矩阵的转置矩阵
	m_normalMatrix = m_viewMatrix.normalMatrix();

	update();
}

void GLWidget::paintGL()
{
	//GLfloat lw[2];
	//glGetFloatv(GL_ALIASED_LINE_WIDTH_RANGE, lw);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	
	//设置反走样
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glEnable(GL_BLEND);
	//glEnable(GL_LINE_SMOOTH);
	//glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glPointSize(2);

	m_program->bind();
	m_program->setUniformValue(m_mvpMatrixLoc, m_projectionMatrix * m_viewMatrix);
	m_program->setUniformValue(m_mvMatrixLoc, m_viewMatrix);
	m_program->setUniformValue(m_normalMatrixLoc, m_normalMatrix);
	m_program->setUniformValue(m_lightPosLoc, m_lightPos);

	//绘制点云数据
	//m_scannerVao.bind();
	//glDrawArrays(GL_POINTS, 0, (GLsizei)m_profileCount*m_resolution);
	//m_scannerVao.release();

	foreach(ShaderDrawable* drawable,m_shaderDrawableList)
		if(drawable->needsUpdateGeometry()) drawable->updateGeometry(m_program);
	foreach(ShaderDrawable *drawable, m_shaderDrawableList)
		if(drawable->canSee()) drawable->draw(m_program);

	m_program->release();

}

/*
void GLWidget::updateScannerVbo(unsigned int resolution)
{
	//更新顶点数据
	QVector<GLfloat> vertices;

	m_profileCount = vdValueX.size() / resolution;
	m_resolution = resolution;

	double zMin, zMax;
	bool firstFlag=false;
	for (auto it = vdValueZ.cbegin(); it != vdValueZ.cend(); it++) {
		if (*it >= 70.0 && *it <= 120.0) {
			//确定第一个有效数据
			if (!firstFlag) {
				zMin = zMax = *it;
				firstFlag = true;
			}
			//找出最大最小值
			if (*it < zMin) 
				zMin = *it;
			else if (*it > zMin)
				zMax = *it;
		}
	}

	double zMid = (zMax + zMin) / 2.0;
	for (int i = 0; i < m_profileCount; i++) {
		for (int j = 0; j < resolution; j++) {
			vertices.push_back(vdValueX[i*resolution + j]);
			vertices.push_back(i-m_profileCount/2.0);
			vertices.push_back(vdValueZ[i*resolution + j]-zMid);
		}
	}
	
	//更新索引数组数据
	

	//更新vbo数据
	if(!m_scannerVbo.bind())
		return;
	m_scannerVbo.allocate(vertices.constData(), 3 * m_profileCount*m_resolution * sizeof(GLfloat));
}
*/

void GLWidget::mousePressEvent(QMouseEvent *event)
{
	m_lastPos = event->pos();
	m_xLastRot = m_xRot;
	m_yLastRot = m_yRot;
	m_xLastPan = m_xPan;
	m_yLastPan = m_yPan;
	//qDebug("%02f", m_yRot);
	//qDebug("%02f", m_xLookAt);
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
	if ((event->buttons() & Qt::MiddleButton && !(event->modifiers() & Qt::ShiftModifier)) || event->buttons() & Qt::LeftButton) {

		m_yRot = normalizeAngle( m_yLastRot - (event->x() - m_lastPos.x()) * 0.5);
		//qDebug ("%02f",m_yRot);
		m_xRot = m_xLastRot + (event->y() - m_lastPos.y()) * 0.5;

		if (m_xRot < -90) m_xRot = -90;
		if (m_xRot > 90) m_xRot = 90;

		updateView();
		emit rotationChanged();
	}

	if ((event->buttons() & Qt::MiddleButton && event->modifiers() & Qt::ShiftModifier) || event->buttons() & Qt::RightButton) {
		m_xPan = m_xLastPan - (event->pos().x() - m_lastPos.x()) * 1 / (double)width();
		m_yPan = m_yLastPan + (event->pos().y() - m_lastPos.y()) * 1 / (double)height();

		updateProjection();
	}
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
	if (m_zoom > 0.1 && event->delta() < 0) {
		m_xPan -= ((double)event->pos().x() / width() - 0.5 + m_xPan) * (1 - 1 / ZOOM_STEP);
		m_yPan += ((double)event->pos().y() / height() - 0.5 - m_yPan) * (1 - 1 / ZOOM_STEP);

		m_zoom /= ZOOM_STEP;
	}
	else if (m_zoom < 10 && event->delta() > 0)
	{
		m_xPan -= ((double)event->pos().x() / width() - 0.5 + m_xPan) * (1 - ZOOM_STEP);
		m_yPan += ((double)event->pos().y() / height() - 0.5 - m_yPan) * (1 - ZOOM_STEP);

		m_zoom *= ZOOM_STEP;
	}

	updateProjection();
	updateView();
}

double GLWidget::normalizeAngle(double angle)
{
	while (angle < 0)	angle += 360;
	while (angle > 360) angle -= 360;

	return angle;
}

