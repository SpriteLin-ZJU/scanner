#pragma once
#include <QtWidgets/QMainWindow>

class OriginDrawer;
class NetDrawer;
class PlatformDrawer;
class ScannerDrawer;
class GcodeDrawer;
class STLDrawer;
class ScannerBox;
class PrinterBox;
class GLWidget;
class QLabel;
class GcodeManager;
class STLManager;
class STLMoveDialog;

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
	void openMoveDialog();
	void updateStatusBar(QString& status);

	OriginDrawer* m_originDrawer;
	NetDrawer* m_netDrawer;
	PlatformDrawer* m_platformDrawer;
	ScannerDrawer* m_scannerDrawer;
	GcodeDrawer* m_gcodeDrawer;
	STLDrawer* m_stlDrawer;

	QLabel* m_statusLabel;
	QAction* m_loadAction;
	QAction* m_saveAction;
	QAction* m_exitAction;
	QAction* m_emergencyStopAction;
	QAction* m_stopPrintingAction;
	QAction* m_STLMoveAction;
	QAction* m_STLRotateAction;
	QAction* m_STLCentreAction;

	QMenu* m_fileMenu;
	QToolBar* m_toolBar;
	QToolBar* m_rightToolBar;

	GcodeManager* m_gcodeManager;
	STLManager* m_stlManager;
	ScannerBox* m_scannerBox;
	PrinterBox* m_printerBox;
	GLWidget* m_glwidget;
	
	//Dialog
	STLMoveDialog* m_stlMoveDialog;
};
