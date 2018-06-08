#include "scannerbox.h"
#include "printerbox.h"
#include "mainwindow.h"
#include "glwidget.h"
#include "origindrawer.h"
#include "netdrawer.h"
#include "platformdrawer.h"

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
	m_glwidget = new GLWidget(this);

	createActions();
	createMenus();
	createToolBars();
	createStatusBar();

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

	m_glwidget->addDrawable(m_originDrawer);
	m_glwidget->addDrawable(m_netDrawer);
	m_glwidget->addDrawable(m_platformDrawer);

	connect(m_scannerBox, &ScannerBox::updateStatus, this, &MainWindow::updateStatusBar);
	connect(m_scannerBox, &ScannerBox::updateGraph, m_glwidget, &GLWidget::updateGraph);
	connect(m_printerBox, &PrinterBox::updateStatus, this, &MainWindow::updateStatusBar);
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
	//test merge

	//创建急停动作
	m_emergencyStopAction = new QAction(QIcon(":/picture/Resources/picture/emergencystop.png"), tr("Emergency stop"), this);
	m_emergencyStopAction->setStatusTip(tr("EMERGENCY STOP"));
	connect(m_emergencyStopAction, &QAction::triggered, m_printerBox, &PrinterBox::emergencyStop);

	//创建停止动作
	m_stopPrintingAction = new QAction(QIcon(":/picture/Resources/picture/stop.png"), tr("Stop printing"),this);
	m_stopPrintingAction->setStatusTip(tr("Stop printing"));
	connect(m_stopPrintingAction, &QAction::triggered, m_printerBox, &PrinterBox::stopPrinting);
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
	m_toolBar = new QToolBar();
	m_toolBar->setIconSize(QSize(60, 60));
	QWidget* spacer = new QWidget(this);
	spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	
	m_toolBar->addAction(m_loadAction);
	m_toolBar->addAction(m_stopPrintingAction);
	m_toolBar->addWidget(spacer);
	m_toolBar->addAction(m_emergencyStopAction);
	addToolBar(m_toolBar);
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

void MainWindow::updateStatusBar(QString & status)
{
	m_statusLabel->setText(status);
}


