#include "scannerbox.h"
#include "printerbox.h"
#include "mainwindow.h"
#include "glwidget.h"

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

	m_scannerBox = new ScannerBox(this);
	m_printerBox = new PrinterBox(this);
	m_glwidget = new GLWidget(this);

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

	connect(m_scannerBox, &ScannerBox::updateStatus, this, &MainWindow::updateStatusBar);
	connect(m_scannerBox, &ScannerBox::updateGraph, m_glwidget, &GLWidget::updateGraph);
	connect(m_printerBox, &PrinterBox::updateStatus, this, &MainWindow::updateStatusBar);
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
	//test merge
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


