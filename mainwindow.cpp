#include "graphbox.h"
#include "scannerbox.h"
#include "mainwindow.h"
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
	createActions();
	createMenus();
	createStatusBar();

	Q3DSurface* graph = new Q3DSurface();
	QWidget* container = QWidget::createWindowContainer(graph);
	QSize screenSize = graph->screen()->size();
	container->setMinimumSize(QSize(screenSize.width() / 2, screenSize.height() / 1.6));
	container->setMaximumSize(screenSize);
	container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	container->setFocusPolicy(Qt::StrongFocus);

	m_scannerBox = new ScannerBox(this);
	m_graphBox = new GraphBox(graph, this);
	
	QWidget* widget = new QWidget;
	QVBoxLayout* vlayout = new QVBoxLayout;
	vlayout->addWidget(m_scannerBox);
	vlayout->addWidget(m_graphBox);
	vlayout->addStretch();
	QHBoxLayout* hlayout = new QHBoxLayout;
	hlayout->addLayout(vlayout);
	hlayout->addWidget(container, 1);
	widget->setLayout(hlayout);
	setCentralWidget(widget);

	connect(m_scannerBox, &ScannerBox::updateStatus, this, &MainWindow::updateStatusBar);
	connect(m_scannerBox, &ScannerBox::updateGraph, m_graphBox, &GraphBox::updateGraph);
}

void MainWindow::createActions()
{
	//创建加载轮廓文件动作
	m_loadAction = new QAction(tr("&Load"), this);
	m_loadAction->setShortcut(tr("Ctrl+L"));
	m_loadAction->setStatusTip(tr("Load profiles"));
	connect(m_loadAction, &QAction::triggered, this, &MainWindow::loadProfile);

	//创建保存轮廓文件动作
	m_saveAction = new QAction(tr("&Save"), this);
	m_saveAction->setShortcut(QKeySequence::Save);
	m_saveAction->setStatusTip(tr("Save profiles"));
	connect(m_saveAction, &QAction::triggered, this, &MainWindow::saveProfile);

	//创建退出程序动作
	m_exitAction = new QAction(tr("&Exit"), this);
	m_exitAction->setShortcut(tr("Ctrl+Q"));
	m_exitAction->setStatusTip(tr("Exit"));
	connect(m_exitAction, &QAction::triggered, this, &MainWindow::close);

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


