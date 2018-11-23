#pragma once
#include <QWidget>
class QLabel;
class QSpinBox;
class QDoubleSpinBox;
class QLineEdit;
class QTabWidget;
class QPushButton;
class QGroupBox;

class PointCloudBox : public QWidget
{
	Q_OBJECT
public:
	PointCloudBox(QWidget *parent = Q_NULLPTR);
	~PointCloudBox();

	//²Ûº¯Êý
	void onFilterButtonClicked();
	void onSACButtonClicked();
	void onResetButtonClicked();

signals:
	void filterPointCloud(int meanK, double thresh);
	void sacPointCloud(int maxIterations, double thresh);
	void resetPointCloud();
private:
	QWidget* creatFilterPage();
	QWidget* creatSACPage();

	QGroupBox* m_pointCloudGroupBox;
	QTabWidget* m_tabWidget;
	QPushButton* m_resetButton;
	//Filter
	QLabel* m_filterMeanKLabel;
	QSpinBox* m_filterMeanKSpinBox;
	QLabel* m_filterThreshLabel;
	QDoubleSpinBox* m_filterThreshSpinBox;
	QPushButton* m_filterButton;
	//SAC
	QLabel* m_sacIterLabel;
	QSpinBox* m_sacIterSpinBox;
	QLabel* m_sacThreshLabel;
	QDoubleSpinBox* m_sacThreshSpinBox;
	QPushButton* m_sacButton;

};
