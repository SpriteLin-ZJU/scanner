#pragma once

#include <QDialog>

class QGroupBox;
class QLabel;
class QPushButton;
class QComboBox;
class QLineEdit;
class QCheckBox;
class QTabWidget;

class SettingsDialog : public QDialog
{
	Q_OBJECT
public:
	SettingsDialog(QWidget* parent = 0);

private:
	void readSettings();
	QWidget* createGeneralTabWidget();

	QTabWidget* m_settingTabWidget;
	//����������
	QGroupBox * m_shutterGroupBox;
	QLabel* m_shutterTimeLabel;
	QLabel* m_idleTimeLabel;
	QLineEdit* m_shutterTime;
	QLineEdit* m_idleTime;

	//�������򡢷ֱ���������
	QGroupBox* m_profileGroupBox;
	QLabel* m_measuringFieldLabel;
	QComboBox* m_measuringFieldComBox;
	QLabel* m_resolutionLabel;
	QComboBox* m_resolutionComBox;
	QCheckBox* m_invertX;
	QCheckBox* m_invertZ;

	//��ֵ�����䡢����ǿ��������
	QGroupBox* m_sensorGroupBox;
	QLabel* m_thresholdLabel;
	QComboBox* m_thresholdCombBox;
	QLineEdit* m_thresholdLineEdit;
	QLabel* m_reflectionsLabel;
	QComboBox* m_reflectionsCombBox;
	QLabel* m_laserPowerLabel;
	QComboBox* m_laserPowerCombBox;


};