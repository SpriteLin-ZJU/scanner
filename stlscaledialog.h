#pragma once
#include <QDialog>
#include <QVector3D>
#include <QVector>

class QLabel;
class QDoubleSpinBox;

class STLScaleDialog : public QDialog
{
	Q_OBJECT
public:
	STLScaleDialog(QWidget* parent = 0);
	~STLScaleDialog();

	void setScaleValue(QVector3D scale);
	void emitScaleSig();
signals:
	void scaleSTL(QVector3D scale);
private:
	QLabel * m_xScaleLabel;
	QLabel* m_yScaleLabel;
	QLabel* m_zScaleLabel;
	QDoubleSpinBox* m_xScaleSpinBox;
	QDoubleSpinBox* m_yScaleSpinBox;
	QDoubleSpinBox* m_zScaleSpinBox;
};