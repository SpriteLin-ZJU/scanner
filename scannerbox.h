#pragma once

#include <QtGui>
#include <QWidget>

class QLabel;
class Scanner;
class QComboBox;
class QGroupBox;
class QPushButton;

class ScannerBox : public QWidget
{
	Q_OBJECT
public:
	ScannerBox(QWidget *parent = Q_NULLPTR);
private:
	void ipSearch();
	void advancedSettings();

	Scanner* m_scanner;
	
	QGroupBox* m_scanGroupBox;
	QLabel* m_ipLabel;
	QComboBox* m_ipComboBox;
	QLabel* m_scanType;

	QPushButton* m_ipSearch;
	QPushButton* m_scanConnect;
	QPushButton* m_advancedSettings;

};