#pragma once

#include <QtWidgets/QMainWindow>

class Scanner;

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = Q_NULLPTR);

private:
	Scanner * m_scanner;
	void createActions();
	void createMenus();
	void createToolBars();
	void createStatusBar();
	void loadProfile();
	void saveProfile();

	QAction* m_loadAction;
	QAction* m_saveAction;
	QAction* m_exitAction;

	QMenu* m_fileMenu;


};
