#pragma once

#include <QDialog>
#include <QVector3D>
#include <QVector>

class QLabel;
class QDoubleSpinBox;

class STLMoveDialog : public QDialog
{
	Q_OBJECT
public:
	STLMoveDialog(QWidget* parent = 0);
	~STLMoveDialog();

	void setMoveValue(QVector3D position);
	void emitMoveSig();
signals:
	void moveSTL(QVector3D position);
private:
	QLabel* m_xMoveLabel;
	QLabel* m_yMoveLabel; 
	QLabel* m_zMoveLabel;
	QDoubleSpinBox* m_xMoveSpinBox;
	QDoubleSpinBox* m_yMoveSpinBox;
	QDoubleSpinBox* m_zMoveSpinBox;
};
