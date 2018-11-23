#include "pointcloudbox.h"
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLayout>
#include <QTabWidget>
#include <QPushButton>
#include <QGroupBox>
PointCloudBox::PointCloudBox(QWidget* parent)
	:QWidget(parent)
{
	//TAB
	m_tabWidget = new QTabWidget(this);
	m_tabWidget->addTab(creatFilterPage(), tr("Filter"));
	m_tabWidget->addTab(creatSACPage(), tr("SAC"));
	//Button
	m_resetButton = new QPushButton(tr("Reset PointCloud"), this);
	//GroupBox
	QVBoxLayout* vLayout = new QVBoxLayout;
	vLayout->addWidget(m_tabWidget);
	vLayout->addWidget(m_resetButton);
	m_pointCloudGroupBox = new QGroupBox(tr("PointCloud"), this);
	m_pointCloudGroupBox->setLayout(vLayout);

	QHBoxLayout* mainLayout = new QHBoxLayout;
	mainLayout->addWidget(m_pointCloudGroupBox);

	setLayout(mainLayout);
	
	//connect
	connect(m_filterButton, &QPushButton::clicked, this, &PointCloudBox::onFilterButtonClicked);
	connect(m_sacButton, &QPushButton::clicked, this, &PointCloudBox::onSACButtonClicked);
	connect(m_resetButton, &QPushButton::clicked, this, &PointCloudBox::onResetButtonClicked);
}

PointCloudBox::~PointCloudBox()
{
}

void PointCloudBox::onFilterButtonClicked()
{
	int meanK = m_filterMeanKSpinBox->value();
	double thresh = m_filterThreshSpinBox->value();
	emit filterPointCloud(meanK, thresh);
}

void PointCloudBox::onSACButtonClicked()
{
	int maxIterations = m_sacIterSpinBox->value();
	double thresh = m_sacThreshSpinBox->value();
	emit sacPointCloud(maxIterations, thresh);
}

void PointCloudBox::onResetButtonClicked()
{
	emit resetPointCloud();
}


QWidget* PointCloudBox::creatFilterPage()
{
	//Filter
	m_filterMeanKLabel = new QLabel(tr("MeanK:"),this);
	m_filterMeanKSpinBox = new QSpinBox(this);
	m_filterMeanKSpinBox->setSingleStep(1);
	m_filterMeanKSpinBox->setValue(5);
	m_filterMeanKLabel->setBuddy(m_filterMeanKSpinBox);

	m_filterThreshLabel = new QLabel(tr("Threshold:"),this);
	m_filterThreshSpinBox = new QDoubleSpinBox(this);
	m_filterThreshSpinBox->setSingleStep(0.1);
	m_filterThreshSpinBox->setValue(1.0);
	m_filterThreshLabel->setBuddy(m_filterThreshSpinBox);

	m_filterButton = new QPushButton(tr("Filtering"), this);

	QGridLayout* filterLayout = new QGridLayout;
	filterLayout->addWidget(m_filterMeanKLabel, 0, 0, 1, 1);
	filterLayout->addWidget(m_filterMeanKSpinBox, 0, 1, 1, 1);
	filterLayout->addWidget(m_filterThreshLabel, 1, 0, 1, 1);
	filterLayout->addWidget(m_filterThreshSpinBox, 1, 1, 1, 1);
	filterLayout->addWidget(m_filterButton, 2, 1, 1, 1);

	QWidget* filterPage = new QWidget(this);
	filterPage->setLayout(filterLayout);

	return filterPage;
}
QWidget * PointCloudBox::creatSACPage()
{
	m_sacIterLabel = new QLabel(tr("Max Iterations:"), this);
	m_sacIterSpinBox = new QSpinBox(this);
	m_sacIterSpinBox->setRange(10, 10000);
	m_sacIterSpinBox->setSingleStep(10);
	m_sacIterSpinBox->setValue(100);

	m_sacThreshLabel = new QLabel(tr("Threshold:"), this);
	m_sacThreshSpinBox = new QDoubleSpinBox(this);
	m_sacThreshSpinBox->setSingleStep(0.05);
	m_sacThreshSpinBox->setValue(0.1);

	m_sacButton = new QPushButton(tr("SAC"), this);

	QGridLayout* sacLayout = new QGridLayout;
	sacLayout->addWidget(m_sacIterLabel, 0, 0, 1, 1);
	sacLayout->addWidget(m_sacIterSpinBox, 0, 1, 1, 1);
	sacLayout->addWidget(m_sacThreshLabel, 1, 0, 1, 1);
	sacLayout->addWidget(m_sacThreshSpinBox, 1, 1, 1, 1);
	sacLayout->addWidget(m_sacButton, 2, 1, 1, 1);

	QWidget* sacPage = new QWidget(this);
	sacPage->setLayout(sacLayout);

	return sacPage;
}
