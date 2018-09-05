#include "scannerbox.h"
#include "printerbox.h"
#include "mainwindow.h"
#include "glwidget.h"
#include "origindrawer.h"
#include "netdrawer.h"
#include "platformdrawer.h"
#include "scannerdrawer.h"
#include "slicer.h"
#include "gcodemanager.h"
#include "stlmanager.h"
#include "gcodedrawer.h"
#include "stldrawer.h"
#include "stlmovedialog.h"
#include "stlrotatedialog.h"
#include "stlscaledialog.h"

#include <QAction>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QLayout>
#include <QScreen>
#include <QLabel>


MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	m_scannerBox = new ScannerBox(this);
	m_printerBox = new PrinterBox(this);
	m_gcodeManager = new GcodeManager();
	m_stlManager = new STLManager();
	m_printerBox->setGcodeManager(m_gcodeManager);
	m_printerBox->setSTLManager(m_stlManager);
	m_glwidget = new GLWidget(this);

	createActions();
	createMenus();
	createToolBars();
	createStatusBar();

	//布局
	QWidget* widget = new QWidget;
	QVBoxLayout* vlayout = new QVBoxLayout;
	vlayout->addWidget(m_scannerBox);
	vlayout->addWidget(m_printerBox);
	vlayout->addStretch();
	QHBoxLayout* hlayout = new QHBoxLayout;
	hlayout->addLayout(vlayout,0);
	hlayout->addWidget(m_glwidget, 1);
	widget->setLayout(hlayout);
	setCentralWidget(widget);

	//添加需要绘制的drawable
	m_originDrawer = new OriginDrawer();
	m_netDrawer = new NetDrawer();
	m_platformDrawer = new PlatformDrawer();
	m_scannerDrawer = new ScannerDrawer();
	m_gcodeDrawer = new GcodeDrawer();
	m_gcodeDrawer->setGcodeManager(m_gcodeManager);
	m_stlDrawer = new STLDrawer();
	m_stlDrawer->setSTLManager(m_stlManager);
	m_slicer = new Slicer();
	m_slicer->setSTLManager(m_stlManager);


	m_glwidget->addDrawable(m_originDrawer);
	m_glwidget->addDrawable(m_netDrawer);
	m_glwidget->addDrawable(m_platformDrawer);
	m_glwidget->addDrawable(m_scannerDrawer);
	m_glwidget->addDrawable(m_gcodeDrawer);
	m_glwidget->addDrawable(m_stlDrawer);
	m_glwidget->addDrawable(m_slicer);

	//dailog
	m_stlMoveDialog = new STLMoveDialog(this);
	m_stlRotateDialog = new STLRotateDialog(this);
	m_stlScaleDialog = new STLScaleDialog(this);

	connect(m_scannerBox, &ScannerBox::updateStatus, this, &MainWindow::updateStatusBar);
	connect(m_scannerBox, &ScannerBox::updateGraph, m_scannerDrawer, &ScannerDrawer::update);
	connect(m_printerBox, &PrinterBox::updateStatus, this, &MainWindow::updateStatusBar);
	connect(m_scannerDrawer, &ShaderDrawable::updateGraph, m_glwidget, &GLWidget::updateGraph);
	connect(m_gcodeDrawer, &ShaderDrawable::updateGraph, m_glwidget, &GLWidget::updateGraph);
	connect(m_stlDrawer, &ShaderDrawable::updateGraph, m_glwidget, &GLWidget::updateGraph);
	connect(m_slicer, &ShaderDrawable::updateGraph, m_glwidget, &GLWidget::updateGraph);
	connect(m_printerBox, &PrinterBox::drawSingleGcode, m_gcodeDrawer, &GcodeDrawer::drawSingleGcode);
	connect(m_printerBox, &PrinterBox::drawSTLFile, m_stlDrawer, &STLDrawer::drawSTLFile);
	connect(m_stlManager, &STLManager::drawSTLPoint, m_stlDrawer, &STLDrawer::drawSTLFile);
	connect(m_printerBox, &PrinterBox::setScanFeedrate, m_scannerDrawer, &ScannerDrawer::setScanFeedrate);
	connect(m_printerBox, &PrinterBox::startProfileTrans, m_scannerBox, &ScannerBox::startProfileTrans);
	connect(m_stlMoveDialog, &STLMoveDialog::moveSTL, m_stlManager, &STLManager::setMoveVal);
	connect(m_stlRotateDialog, &STLRotateDialog::rotateSTL, m_stlManager, &STLManager::setRotateVal);
	connect(m_stlScaleDialog, &STLScaleDialog::scaleSTL, m_stlManager, &STLManager::setScaleVal);
	connect(m_stlManager, &STLManager::drawPolyLine, m_slicer, &Slicer::drawPolyLine);
}

void MainWindow::createActions()
{
	//创建加载打印文件动作
	m_loadAction = new QAction(QIcon(":/picture/Resources/picture/file.png"),tr("&Load"), this);
	m_loadAction->setShortcut(tr("Ctrl+L"));
	m_loadAction->setStatusTip(tr("Load Gcode"));
	connect(m_loadAction, &QAction::triggered, m_printerBox, &PrinterBox::openFile);

	//创建保存文件动作
	m_saveAction = new QAction(tr("&Save"), this);
	m_saveAction->setShortcut(QKeySequence::Save);
	m_saveAction->setStatusTip(tr("Save profiles"));
	connect(m_saveAction, &QAction::triggered, this, &MainWindow::saveProfile);

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
}

void MainWindow::createMenus()
{
	m_fileMenu = menuBar()->addMenu(tr("&File"));
	m_fileMenu->addAction(m_loadAction);
	m_fileMenu->addAction(m_saveAction);
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
	m_statusLabel = new QLabel(tr("No Scanner connect"));
	statusBar()->addWidget(m_statusLabel);
}

void MainWindow::loadProfile()
{

}

void MainWindow::saveProfile()
{
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


