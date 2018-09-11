#pragma once

#include <QDialog>

class QLabel;
class QPushButton;
class QDoubleSpinBox;

class PrintSettingsDialog : public QDialog
{
	Q_OBJECT
public:
	PrintSettingsDialog(QWidget* parent = 0);

	void readSettings();
	void writeSetting();

private:
	QLabel* m_layerHeightLabel;
	QDoubleSpinBox* m_layerHeightSpinBox;
	
	QLabel* m_stlModelColorLabel;
	QLabel* m_stlRLabel;
	QDoubleSpinBox* m_stlRSpinBox;
	QLabel* m_stlGLabel;
	QDoubleSpinBox* m_stlGSpinBox;
	QLabel* m_stlBLabel;
	QDoubleSpinBox* m_stlBSpinBox;

	QLabel* m_sliceColorLabel;
	QLabel* m_sliceRLabel;
	QDoubleSpinBox* m_sliceRSpinBox;
	QLabel* m_sliceGLabel;
	QDoubleSpinBox* m_sliceGSpinBox;
	QLabel* m_sliceBLabel;
	QDoubleSpinBox* m_sliceBSpinBox;

	QLabel* m_gcodeColorLabel;
	QLabel* m_gcodeRLabel;
	QDoubleSpinBox* m_gcodeRSpinBox;
	QLabel* m_gcodeGLabel;
	QDoubleSpinBox* m_gcodeGSpinBox;
	QLabel* m_gcodeBLabel;
	QDoubleSpinBox* m_gcodeBSpinBox;

	QPushButton* m_okButton;
	QPushButton* m_cancelButton;
};