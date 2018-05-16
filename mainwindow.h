#pragma once
#include <QtWidgets/QMainWindow>

class ScannerBox;
class GLWidget;
class QLabel;
class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = Q_NULLPTR);

private:
	void createActions();
	void createMenus();
	void createToolBars();
	void createStatusBar();
	void loadProfile();
	void saveProfile();
	void updateStatusBar(QString& status);

	QLabel* m_statusLabel;
	QAction* m_loadAction;
	QAction* m_saveAction;
	QAction* m_exitAction;

	QMenu* m_fileMenu;

	ScannerBox* m_scannerBox;
	GLWidget* m_glwidget;
};
