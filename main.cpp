#include "mainwindow.h"
#include <QtWidgets/QApplication>
#include <QOpenGLWidget>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	
	QSurfaceFormat format;
	format.setDepthBufferSize(24);
	format.setStencilBufferSize(8);
	format.setVersion(4, 3);
	format.setProfile(QSurfaceFormat::CoreProfile);
	QSurfaceFormat::setDefaultFormat(format);
	
	MainWindow w;
	w.resize(1280, 720);
	w.show();
	return a.exec();
}
