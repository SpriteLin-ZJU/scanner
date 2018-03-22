#pragma once

#include <QDialog>

class QGroupBox;
class QLabel;
class QPushButton;
class QComboBox;
class QLineEdit;
class QCheckBox;
class QTabWidget;
class QSpinBox;

class SettingsDialog : public QDialog
{
	Q_OBJECT
public:
	SettingsDialog(QWidget* parent = 0);

	void readSettings();
	void writeSettings();
private:
	QWidget* createGeneralTabWidget();
	QWidget* createInterfaceTabWidget();

	QTabWidget* m_settingTabWidget;
	QPushButton* m_okButton;
	QPushButton* m_cancelButton;
	//快门设置组
	QGroupBox * m_shutterGroupBox;
	QLabel* m_shutterTimeLabel;
	QLabel* m_idleTimeLabel;
	QSpinBox* m_shutterTime;
	QSpinBox* m_idleTime;

	//测量区域、分辨率设置组
	QGroupBox* m_profileGroupBox;
	QLabel* m_measuringFieldLabel;
	QComboBox* m_measuringFieldComBox;
	QLabel* m_resolutionLabel;
	QComboBox* m_resolutionComBox;
	QCheckBox* m_invertX;
	QCheckBox* m_invertZ;

	//阈值、反射、激光强度设置组
	QGroupBox* m_sensorGroupBox;
	QLabel* m_thresholdLabel;
	QComboBox* m_thresholdCombBox;
	QLineEdit* m_thresholdLineEdit;
	QLabel* m_reflectionsLabel;
	QComboBox* m_reflectionsCombBox;
	QLabel* m_laserPowerLabel;
	QComboBox* m_laserPowerCombBox;

	//接口RS422设置组
	QGroupBox* m_RS422GroupBox;
	QLabel* m_RS422ModeLabel;
	QComboBox* m_RS422ModeComboBox;
	QLabel* m_serialBaudrateLabel;
	QComboBox* m_serialBaudrateComboBox;
	QCheckBox* m_RS422Termination;

	//digital inputs设置组
	QGroupBox* m_digitalInputsGroupBox;
	QLabel* m_inputsModeLabel;
	QComboBox* m_inputsModeComboBox;
	QLabel* m_inputsLogicLabel;
	QComboBox* m_inputsLogicComboBox;

	//trigger 设置组
	QGroupBox* m_triggerGroupBox;
	QLabel* m_triggerModeLabel;
	QComboBox* m_triggerModeComboBox;
	QLabel* m_triggerSourceLabel;
	QComboBox* m_triggerSourceComboBox;
	QLabel* m_encodeStepLabel;
	QSpinBox* m_encodeStepSpinBox;
	QCheckBox* m_encodeActiveCBox;
	
};