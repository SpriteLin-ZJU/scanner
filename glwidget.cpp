#include "glwidget.h"

#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <math.h>
#include <QDebug>
#include <QLayout>

GLWidget::GLWidget(QWidget* parent)
	:QOpenGLWidget(parent), 
	m_program(0)
{
	m_buttonGroup = new QButtonGroup(this);
	//设置不互斥
	m_buttonGroup->setExclusive(false);
	m_scannerCheckBox = new QCheckBox(tr("ScanData"), this);
	m_scannerCheckBox->setChecked(true);
	m_gcodeCheckBox = new QCheckBox(tr("Gcode"), this);
	m_gcodeCheckBox->setChecked(true);
	m_stlCheckBox = new QCheckBox(tr("STL"), this);
	m_stlCheckBox->setChecked(true);
	m_sliceCheckBox = new QCheckBox(tr("SliceLine"), this);
	m_sliceCheckBox->setChecked(true);
	m_buttonGroup->addButton(m_scannerCheckBox,0);
	m_buttonGroup->addButton(m_gcodeCheckBox,1);
	m_buttonGroup->addButton(m_stlCheckBox,2);
	m_buttonGroup->addButton(m_sliceCheckBox,3);
	QHBoxLayout* hLayout = new QHBoxLayout;
	hLayout->addWidget(m_scannerCheckBox);
	hLayout->addWidget(m_gcodeCheckBox);
	hLayout->addWidget(m_stlCheckBox);
	hLayout->addWidget(m_sliceCheckBox);
	hLayout->addStretch();
	QVBoxLayout* vLayout = new QVBoxLayout;
	vLayout->addLayout(hLayout);
	vLayout->addStretch();
	setLayout(vLayout);
	connect(m_buttonGroup, QOverload<int, bool>::of(&QButtonGroup::buttonToggled),
		[=](int id, bool checked) { emit updateVisible(id, checked); });

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
"out vec3 LightsArray[5];\n"
"uniform mat4 mvpMatrix;\n"
"uniform mat4 mvMatrix;\n"
"uniform mat3 normalMatrix;\n"
"uniform vec3 lightsArray[5];\n"


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
"		for(int i=0;i<5;i++){\n"
"			LightsArray[i]=vec3(mvMatrix * vec4(lightsArray[i], 1.0));\n"
"		}\n"
"	 }\n"
"}\n";

//片段着色器
static const char *fragmentShaderSource =
"#version 430 core\n"
"out vec4 FragColor;\n"
"in vec3 ObjColor;\n"
"in vec3 Normal;\n"
"in vec3 FragPos;\n"
"in vec3 LightsArray[5];\n"

"bool isNan(float val){\n"
"	 return (val > 65535.0);\n"
"}\n"

"void main() {\n"
"	if( isNan(Normal.x) )\n"
"		FragColor = vec4(ObjColor,1.0);\n"
"	else{\n"
"		vec3 lightColor = vec3(1.0,1.0,1.0);\n"
"		float ambientStrength=0.3;\n"
"		vec3 ambient = ambientStrength * lightColor;\n"
"		vec3 result=ambient;\n"
"		\n"
"		vec3 norm = normalize(Normal);\n"
"		for(int i = 0; i < 5; i++){\n"
"			vec3 lightDir = normalize(-LightsArray[i]);\n"
"			float diff = max( dot( norm, lightDir ), 0.0 );\n"
"			float diffuseStrength = 0.1;\n"
"			vec3 diffuse = diffuseStrength * diff * lightColor;\n"
"			result+=diffuse;\n"
"			\n"
"			float specularStrength = 0.1;\n"
"			vec3 viewDir = normalize(-FragPos);\n"
"			vec3 reflectDir = reflect(-lightDir, norm);\n"
"			float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);\n"
"			vec3 specular = specularStrength * spec * lightColor;\n"
"			result+=specular;\n"
"		}\n"
"		result = result * ObjColor;\n"
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
	m_lightsArrayLoc = m_program->uniformLocation("lightsArray");

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
	//清除背景
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	
	//点云点大小？
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glPointSize(2);

	m_program->bind();
	m_program->setUniformValue(m_mvpMatrixLoc, m_projectionMatrix * m_viewMatrix);
	m_program->setUniformValue(m_mvMatrixLoc, m_viewMatrix);
	m_program->setUniformValue(m_normalMatrixLoc, m_normalMatrix);
	m_program->setUniformValue(m_lightsArrayLoc, m_lightsArray[5]);

	//绘制
	foreach(ShaderDrawable* drawable,m_shaderDrawableList)
		if(drawable->needsUpdateGeometry()) drawable->updateGeometry(m_program);
	foreach(ShaderDrawable *drawable, m_shaderDrawableList)
		if(drawable->canSee()) drawable->draw(m_program);

	m_program->release();

}


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
	if (event->buttons() & Qt::LeftButton) {

		m_yRot = normalizeAngle( m_yLastRot - (event->x() - m_lastPos.x()) * 0.5);
		//qDebug ("%02f",m_yRot);
		m_xRot = m_xLastRot + (event->y() - m_lastPos.y()) * 0.5;

		if (m_xRot < -90) m_xRot = -90;
		if (m_xRot > 90) m_xRot = 90;

		updateView();
		emit rotationChanged();
	}

	if (event->buttons() & Qt::MiddleButton) {
		m_xPan = m_xLastPan - (event->pos().x() - m_lastPos.x()) * 1 / (double)width();
		m_yPan = m_yLastPan + (event->pos().y() - m_lastPos.y()) * 1 / (double)height();

		updateProjection();
	}

	if (event->buttons()&Qt::RightButton) {

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

