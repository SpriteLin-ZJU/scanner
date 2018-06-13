#pragma once
#include <QtWidgets/QMainWindow>

class OriginDrawer;
class NetDrawer;
class PlatformDrawer;
class ScannerDrawer;
class GcodeDrawer;
class ScannerBox;
class PrinterBox;
class GLWidget;
class QLabel;
class GcodeManager;

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

	OriginDrawer* m_originDrawer;
	NetDrawer* m_netDrawer;
	PlatformDrawer* m_platformDrawer;
	ScannerDrawer* m_scannerDrawer;
	GcodeDrawer* m_gcodeDrawer;

	QLabel* m_statusLabel;
	QAction* m_loadAction;
	QAction* m_saveAction;
	QAction* m_exitAction;
	QAction* m_emergencyStopAction;
	QAction* m_stopPrintingAction;

	QMenu* m_fileMenu;
	QToolBar* m_toolBar;

	GcodeManager* m_gcodeManager;
	ScannerBox* m_scannerBox;
	PrinterBox* m_printerBox;
	GLWidget* m_glwidget;
};
