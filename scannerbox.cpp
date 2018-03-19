#include "scannerbox.h"
#include "scanner.h"
#include <QLayout>
#include <QLabel>
#include <QGroupBox>
#include <QPushButton>
#include <QComboBox>

ScannerBox::ScannerBox(QWidget *parent) :
	QWidget(parent)
{
	m_scanner = new Scanner();

	m_ipLabel = new QLabel(tr("IP:"),this);
	m_scanType = new QLabel(tr("Type:"),this);
	m_scanType->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);

	m_ipComboBox = new QComboBox(this);
	m_scanGroupBox = new QGroupBox(tr("Scanner"), this);
	m_ipSearch = new QPushButton(tr("&Search"),this);
	m_scanConnect = new QPushButton(tr("Connect"),this);
	m_advancedSettings = new QPushButton(tr("Advanced"),this);
	
	//²¼¾Ö
	QVBoxLayout *typeLayout = new QVBoxLayout;
	typeLayout->addWidget(m_scanType);
	typeLayout->addStretch();

	QGridLayout *leftLayout = new QGridLayout;
	leftLayout->addWidget(m_ipLabel, 0, 0);
	leftLayout->addWidget(m_ipComboBox, 0, 1);
	leftLayout->addLayout(typeLayout, 1, 0,1,2);
	
	QVBoxLayout *rightLayout = new QVBoxLayout;
	rightLayout->addWidget(m_ipSearch);
	rightLayout->addWidget(m_scanConnect);
	rightLayout->addWidget(m_advancedSettings);

	QHBoxLayout *mainLayout = new QHBoxLayout;
	mainLayout->addLayout(leftLayout);
	mainLayout->addLayout(rightLayout);
	
	m_scanGroupBox->setLayout(mainLayout);

}

void ScannerBox::ipSearch()
{
}

void ScannerBox::advancedSettings()
{
}
