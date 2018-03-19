#include "mainwindow.h"
#include "scanner.h"
#include "scannerbox.h"
#include <QAction>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	createActions();
	createMenus();

	m_scannerBox = new ScannerBox(this);
	setCentralWidget(m_scannerBox);
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
}

void MainWindow::loadProfile()
{
}

void MainWindow::saveProfile()
{
}


