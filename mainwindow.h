#pragma once
#include <QtWidgets/QMainWindow>
#include <QThread>
class OriginDrawer;
class NetDrawer;
class ScannerDrawer;
class GcodeDrawer;
class STLDrawer;
class ScannerBox;
class PrinterBox;
class GLWidget;
class QLabel;
class GcodeManager;
class STLManager;
class ScandataManager;
class STLMoveDialog;
class STLRotateDialog;
class STLScaleDialog;
class Slicer;
class QProgressBar;
class VTKWidget;
class PointCloudBox;
class PointCloudListWidget;

class MainWindow : public QMainWindow
{
	Q_OBJECT
	QThread pclThread;
public:
	MainWindow(QWidget *parent = Q_NULLPTR);
	~MainWindow();
	
	//槽
	void onPointCloudSelectionChanged();
signals:
	void SignalUpdateSelectedPointClouds(const QStringList&);
private:
	void createActions();
	void createMenus();
	void createToolBars();
	void createStatusBar();
	void loadProfile();
	void saveProfile();
	//STL操作对话框
	void hideDialog();
	void openMoveDialog();
	void openRotateDialog();
	void openScaleDialog();

	void updateStatusBar(const QString& status);
	void updateProgressBar(QString process, int value);
	void updateColor();
	void updateVisible(int id, bool checked);

	OriginDrawer* m_originDrawer;
	NetDrawer* m_netDrawer;
	ScannerDrawer* m_scannerDrawer;
	GcodeDrawer* m_gcodeDrawer;
	STLDrawer* m_stlDrawer;
	Slicer* m_slicer;

	QLabel* m_statusLabel;
	QProgressBar* m_progressBar;

	QAction* m_loadAction;
	QAction* m_saveAction;
	QAction* m_clearPointCloudsAction;
	QAction* m_exitAction;
	QAction* m_emergencyStopAction;
	QAction* m_stopPrintingAction;
	QAction* m_STLMoveAction;
	QAction* m_STLRotateAction;
	QAction* m_STLCentreAction;
	QAction* m_STLScaleAction;

	QMenu* m_fileMenu;
	QToolBar* m_toolBar;
	QToolBar* m_rightToolBar;

	GcodeManager* m_gcodeManager;
	STLManager* m_stlManager;
	ScandataManager* m_scandataManager;

	ScannerBox* m_scannerBox;
	PrinterBox* m_printerBox;
	GLWidget* m_glwidget;
	
	//Dialog
	STLMoveDialog* m_stlMoveDialog;
	STLRotateDialog* m_stlRotateDialog;
	STLScaleDialog* m_stlScaleDialog;

	//PCL相关
	VTKWidget* m_vtkWidget;
	PointCloudBox* m_pointCloudBox;
	PointCloudListWidget* m_pointCloudListWidget;
};
