#include "mainwindow.h"
#include "scannerbox.h"
#include "printerbox.h"
#include "glwidget.h"
#include "origindrawer.h"
#include "netdrawer.h"
#include "scannerdrawer.h"
#include "slicer.h"
#include "gcodemanager.h"
#include "stlmanager.h"
#include "scandatamanager.h"
#include "gcodedrawer.h"
#include "stldrawer.h"
#include "stlmovedialog.h"
#include "stlrotatedialog.h"
#include "stlscaledialog.h"
#include "pointcloudbox.h"

#include <QAction>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QLayout>
#include <QScreen>
#include <QLabel>
#include <QMessageBox>
#include <QFileDialog>
#include <QProgressBar>
#include <QTabWidget>
#include "QVTKWidget.h"
#include <vtkRenderWindow.h>

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	m_scannerBox = new ScannerBox(this);
	m_printerBox = new PrinterBox(this);
	m_pointCloudBox = new PointCloudBox(this);

	//manager
	m_gcodeManager = new GcodeManager();
	m_stlManager = new STLManager();
	//manager-scandataManager
	m_scandataManager = new ScandataManager();
	m_scandataManager->moveToThread(&pclThread);
	connect(&pclThread, &QThread::finished, m_scandataManager, &QObject::deleteLater);
	pclThread.start();

	m_printerBox->setGcodeManager(m_gcodeManager);
	m_printerBox->setSTLManager(m_stlManager);
	m_printerBox->setScandataManager(m_scandataManager);
	m_scannerBox->setScandataManager(m_scandataManager);
	
	//VTK
	m_qvtkWidget = new QVTKWidget();
	m_qvtkWidget->SetRenderWindow(m_scandataManager->m_viewer->getRenderWindow());
	m_scandataManager->m_viewer->setupInteractor(m_qvtkWidget->GetInteractor(), m_qvtkWidget->GetRenderWindow());
	m_qvtkWidget->update();
	connect(m_scandataManager, &ScandataManager::updateVTK, this, &MainWindow::updateVTK);
	
	//opengl widget
	m_glwidget = new GLWidget(this);

	createActions();
	createMenus();
	createToolBars();
	createStatusBar();

	//左侧布局
	QWidget* widget = new QWidget;
	QVBoxLayout* vlayout = new QVBoxLayout;
	vlayout->addWidget(m_scannerBox);
	vlayout->addWidget(m_printerBox);
	vlayout->addWidget(m_pointCloudBox);
	vlayout->addStretch();
	//显示布局
	QTabWidget* GLTabWidget = new QTabWidget;
	GLTabWidget->addTab(m_glwidget, tr("3DP"));
	GLTabWidget->addTab(m_qvtkWidget, tr("Point clouds"));
	//布局
	QHBoxLayout* hlayout = new QHBoxLayout;
	hlayout->addLayout(vlayout,0);
	hlayout->addWidget(GLTabWidget, 1);
	widget->setLayout(hlayout);
	setCentralWidget(widget);

	//添加需要绘制的drawable
	m_originDrawer = new OriginDrawer();
	m_netDrawer = new NetDrawer();
	m_scannerDrawer = new ScannerDrawer();
	m_scannerDrawer->setScandataManager(m_scandataManager);
	m_gcodeDrawer = new GcodeDrawer();
	m_gcodeDrawer->setGcodeManager(m_gcodeManager);
	m_stlDrawer = new STLDrawer();
	m_stlDrawer->setSTLManager(m_stlManager);
	m_slicer = new Slicer();
	m_slicer->setSTLManager(m_stlManager);


	m_glwidget->addDrawable(m_originDrawer);
	m_glwidget->addDrawable(m_netDrawer);
	m_glwidget->addDrawable(m_scannerDrawer);
	m_glwidget->addDrawable(m_gcodeDrawer);
	m_glwidget->addDrawable(m_stlDrawer);
	m_glwidget->addDrawable(m_slicer);

	//dailog
	m_stlMoveDialog = new STLMoveDialog(this);
	m_stlRotateDialog = new STLRotateDialog(this);
	m_stlScaleDialog = new STLScaleDialog(this);

	connect(m_scannerBox, &ScannerBox::updateStatus, this, &MainWindow::updateStatusBar);
	connect(m_scannerBox, &ScannerBox::drawPointClouds, m_scannerDrawer, &ScannerDrawer::drawScandataGL);
	connect(m_scandataManager, &ScandataManager::drawPointClouds, m_scannerDrawer, &ScannerDrawer::drawScandataGL);
	connect(m_printerBox, &PrinterBox::updateStatus, this, &MainWindow::updateStatusBar);
	connect(m_scannerDrawer, &ShaderDrawable::updateGraph, m_glwidget, &GLWidget::updateGraph);
	connect(m_gcodeDrawer, &ShaderDrawable::updateGraph, m_glwidget, &GLWidget::updateGraph);
	connect(m_stlDrawer, &ShaderDrawable::updateGraph, m_glwidget, &GLWidget::updateGraph);
	connect(m_slicer, &ShaderDrawable::updateGraph, m_glwidget, &GLWidget::updateGraph);
	connect(m_printerBox, &PrinterBox::drawSingleGcode, m_gcodeDrawer, &GcodeDrawer::drawSingleGcode);
	connect(m_printerBox, &PrinterBox::drawSTLFile, m_stlDrawer, &STLDrawer::drawSTLFile);
	connect(m_stlManager, &STLManager::drawSTLPoint, m_stlDrawer, &STLDrawer::drawSTLFile);
	connect(m_printerBox, &PrinterBox::setScanFeedrate, m_scannerBox, &ScannerBox::setScanFeedrate);
	connect(m_printerBox, &PrinterBox::startProfileTrans, m_scannerBox, &ScannerBox::startProfileTrans);
	connect(m_stlMoveDialog, &STLMoveDialog::moveSTL, m_stlManager, &STLManager::setMoveVal);
	connect(m_stlRotateDialog, &STLRotateDialog::rotateSTL, m_stlManager, &STLManager::setRotateVal);
	connect(m_stlScaleDialog, &STLScaleDialog::scaleSTL, m_stlManager, &STLManager::setScaleVal);
	connect(m_printerBox, &PrinterBox::sliceSignal, m_slicer, &Slicer::slice);
	connect(m_printerBox, &PrinterBox::updateColor, this, &MainWindow::updateColor);
	connect(m_glwidget, &GLWidget::updateVisible, this, &MainWindow::updateVisible);
	//to ScandataManager
	connect(m_printerBox, &PrinterBox::openPcdFile, m_scandataManager, &ScandataManager::openPcdFile);
	connect(m_pointCloudBox, &PointCloudBox::filterPointCloud, m_scandataManager, &ScandataManager::filterPointCloud);
	connect(m_pointCloudBox, &PointCloudBox::sacPointCloud, m_scandataManager, &ScandataManager::sacPointCloud);
	connect(m_pointCloudBox, &PointCloudBox::resetPointCloud, m_scandataManager, &ScandataManager::resetPointCloud);

}

MainWindow::~MainWindow()
{
	pclThread.quit();
	pclThread.wait();
}

void MainWindow::updateVTK()
{
	m_qvtkWidget->update();
}

void MainWindow::clearPointData()
{
	m_scandataManager->clear();
}

void MainWindow::createActions()
{
	//创建加载打印文件动作
	m_loadAction = new QAction(QIcon(":/picture/Resources/picture/file.png"),tr("&Load"), this);
	m_loadAction->setShortcut(tr("Ctrl+L"));
	m_loadAction->setStatusTip(tr("Load File"));
	connect(m_loadAction, &QAction::triggered, m_printerBox, &PrinterBox::openFile);

	//创建保存文件动作
	m_saveAction = new QAction(QIcon(":/picture/Resources/picture/save.png"), tr("&Save Point Clouds"), this);
	m_saveAction->setShortcut(QKeySequence::Save);
	m_saveAction->setStatusTip(tr("Save point clouds"));
	connect(m_saveAction, &QAction::triggered, this, &MainWindow::saveProfile);

	//创建清空点云操作
	m_clearPointCloudsAction = new QAction(tr("&Clear point clouds"), this);
	m_clearPointCloudsAction->setStatusTip(tr("Clear point clouds"));
	//connect(m_clearPointCloudsAction, &QAction::triggered, m_scandataManager, &ScandataManager::clear);
	connect(m_clearPointCloudsAction, &QAction::triggered,this, &MainWindow::clearPointData);

	//创建退出程序动作
	m_exitAction = new QAction(tr("&Exit"), this);
	m_exitAction->setShortcut(tr("Ctrl+Q"));
	m_exitAction->setStatusTip(tr("Exit"));
	connect(m_exitAction, &QAction::triggered, this, &MainWindow::close);

	//创建急停动作
	m_emergencyStopAction = new QAction(QIcon(":/picture/Resources/picture/emergencystop.png"), tr("Emergency stop"), this);
	m_emergencyStopAction->setStatusTip(tr("EMERGENCY STOP"));
	connect(m_emergencyStopAction, &QAction::triggered, m_printerBox, &PrinterBox::emergencyStop);

	//创建停止动作
	m_stopPrintingAction = new QAction(QIcon(":/picture/Resources/picture/stop.png"), tr("Stop printing"),this);
	m_stopPrintingAction->setStatusTip(tr("Stop printing"));
	connect(m_stopPrintingAction, &QAction::triggered, m_printerBox, &PrinterBox::stopPrinting);

	//STL相关操作
	//创建移动操作
	m_STLMoveAction = new QAction(QIcon(":/picture/Resources/picture/move.png"), tr("Move STL Model"), this);
	m_STLMoveAction->setStatusTip(tr("Move STL Model"));
	connect(m_STLMoveAction, &QAction::triggered, this, &MainWindow::openMoveDialog);

	//创建旋转操作
	m_STLRotateAction = new QAction(QIcon(":/picture/Resources/picture/rotate.png"), tr("Rotate STL Model"), this);
	m_STLRotateAction->setStatusTip(tr("Rotate STL Model"));
	connect(m_STLRotateAction, &QAction::triggered, this, &MainWindow::openRotateDialog);

	//创建缩放操作
	m_STLScaleAction = new QAction(QIcon(":/picture/Resources/picture/scale.png"), tr("Scale STL Model"), this);
	m_STLScaleAction->setStatusTip(tr("Scale STL Model"));
	connect(m_STLScaleAction, &QAction::triggered, this, &MainWindow::openScaleDialog);

	//创建对中操作
	m_STLCentreAction = new QAction(QIcon(":/picture/Resources/picture/centre.png"), tr("Centre STL Model"), this);
	m_STLCentreAction->setStatusTip(tr("Centre STL Model"));
	connect(m_STLCentreAction, &QAction::triggered, m_stlManager, &STLManager::centreSTL);
}

void MainWindow::createMenus()
{
	m_fileMenu = menuBar()->addMenu(tr("&File"));
	m_fileMenu->addAction(m_loadAction);
	m_fileMenu->addAction(m_saveAction);
	m_fileMenu->addAction(m_clearPointCloudsAction);
	m_fileMenu->addSeparator();
	m_fileMenu->addAction(m_exitAction);

}

void MainWindow::createToolBars()
{
	//上方工具栏：打开文件、停止打印、急停
	m_toolBar = new QToolBar(this);
	m_toolBar->setIconSize(QSize(60, 60));
	QWidget* spacer = new QWidget(this);
	spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	
	m_toolBar->addAction(m_loadAction);
	m_toolBar->addAction(m_stopPrintingAction);
	m_toolBar->addWidget(spacer);
	m_toolBar->addAction(m_emergencyStopAction);
	addToolBar(m_toolBar);

	//STL相关工具栏：移动、旋转、对中STL模型
	m_rightToolBar = new QToolBar(this);
	m_rightToolBar->setIconSize(QSize(30, 30));
	m_rightToolBar->addAction(m_STLMoveAction);
	m_rightToolBar->addAction(m_STLRotateAction);
	m_rightToolBar->addAction(m_STLScaleAction);
	m_rightToolBar->addAction(m_STLCentreAction);
	m_rightToolBar->setStyleSheet("border:none;");
	addToolBar(Qt::RightToolBarArea,m_rightToolBar);
}

void MainWindow::createStatusBar()
{
	m_statusLabel = new QLabel(tr("No Scanner connect"),this);
	
	m_progressBar = new QProgressBar(this);
	m_progressBar->setOrientation(Qt::Horizontal);
	m_progressBar->setMinimum(0);
	m_progressBar->setMaximum(100);
	m_progressBar->setVisible(false);

	statusBar()->addWidget(m_statusLabel);
	statusBar()->addWidget(m_progressBar);
}

void MainWindow::loadProfile()
{

}

void MainWindow::saveProfile()
{
	//若当前没有点云数据，则弹出警告
	if(!m_scandataManager->isEmpty())
		QMessageBox::warning(this, tr("Save scan point data"),
			tr("no point clouds need to be saved."));
	//当前有点云数据
	else {
		QString savePath = QFileDialog::getSaveFileName(this, tr("Save Point"), "", tr("PCD (*.pcd)"));
		if (savePath.isEmpty())
			return;
		m_scandataManager->savePcdFile(savePath);
	}
}

void MainWindow::hideDialog()
{
	m_stlMoveDialog->hide();
	m_stlRotateDialog->hide();
	m_stlScaleDialog->hide();
}

void MainWindow::openMoveDialog()
{
	hideDialog();
	m_stlMoveDialog->setMoveValue(m_stlManager->getMoveValue());
	m_stlMoveDialog->show();
}

void MainWindow::openRotateDialog()
{
	hideDialog();
	m_stlRotateDialog->setRotateValue(m_stlManager->getRotateValue());
	m_stlRotateDialog->show();
}

void MainWindow::openScaleDialog()
{
	hideDialog();
	m_stlScaleDialog->setScaleValue(m_stlManager->getScaleValue());
	m_stlScaleDialog->show();
}

void MainWindow::updateStatusBar(QString & status)
{
	m_statusLabel->setText(status);
}

void MainWindow::updateColor()
{
	m_stlDrawer->updateColor();
	m_slicer->updateColor();
	//m_gcodeDrawer->updateColor();
}

void MainWindow::updateVisible(int id, bool checked)
{
	QVector<ShaderDrawable*> vdrawables;
	vdrawables.push_back(m_scannerDrawer);
	vdrawables.push_back(m_gcodeDrawer);
	vdrawables.push_back(m_stlDrawer);
	vdrawables.push_back(m_slicer);

	vdrawables[id]->setVisible(checked);
}


