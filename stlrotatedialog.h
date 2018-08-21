#pragma once
#include <QDialog>
#include <QVector3D>
#include <QVector>
class QLabel;
class QDoubleSpinBox;

class STLRotateDialog : public QDialog
{
	Q_OBJECT
public:
	STLRotateDialog(QWidget* parent=0);
	~STLRotateDialog();

	void setRotateValue(QVector3D rotate);
	void emitRotateSig();
signals:
	void rotateSTL(QVector3D rotate);
private:
	QLabel * m_xRotLabel;
	QLabel* m_yRotLabel;
	QLabel* m_zRotLabel;
	QDoubleSpinBox* m_xRotSpinBox;
	QDoubleSpinBox* m_yRotSpinBox;
	QDoubleSpinBox* m_zRotSpinBox;

};