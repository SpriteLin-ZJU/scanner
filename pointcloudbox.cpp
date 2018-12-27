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
	m_tabWidget->addTab(creatICPPage(), tr("ICP"));
	m_tabWidget->addTab(creatBoundaryPage(), tr("Boundary"));
	//Button
	m_clearButton = new QPushButton(tr("Clear PointCloud"), this);
	//GroupBox
	QVBoxLayout* vLayout = new QVBoxLayout;
	vLayout->addWidget(m_tabWidget,0);
	vLayout->addWidget(m_clearButton);
	m_pointCloudGroupBox = new QGroupBox(tr("PointCloud"), this);
	m_pointCloudGroupBox->setLayout(vLayout);

	QHBoxLayout* mainLayout = new QHBoxLayout;
	mainLayout->addWidget(m_pointCloudGroupBox);

	setLayout(mainLayout);
	
	//connect
	connect(m_filterButton, &QPushButton::clicked, this, &PointCloudBox::onFilterButtonClicked);
	connect(m_sacButton, &QPushButton::clicked, this, &PointCloudBox::onSACButtonClicked);
	connect(m_icpButton, &QPushButton::clicked, this, &PointCloudBox::onICPButtonClicked);
	connect(m_boundaryButton, &QPushButton::clicked, this, &PointCloudBox::onBoundaryButtonClicked);
	connect(m_clearButton, &QPushButton::clicked, this, &PointCloudBox::clearPointCloud);
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

void PointCloudBox::onICPButtonClicked()
{
	int maxIterations = m_icpIterSpinBox->value();
	double eucliEpsilon = m_icpEucliFitEpsSpinBox->value();
	emit icpPointCloud(maxIterations, eucliEpsilon);
}

void PointCloudBox::onBoundaryButtonClicked()
{
	int sacIterations = m_sacIterSpinBox->value();
	double sacThresh = m_sacThreshSpinBox->value();
	int normKSearch = m_boundNormalKSpinBox->value();
	int boundKSearch = m_boundKSearchSpinBox->value();

	emit findPointCloudBoundary(sacIterations, sacThresh, normKSearch, boundKSearch);
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
	m_sacIterLabel = new QLabel(tr("MaxIterations:"), this);
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

QWidget * PointCloudBox::creatICPPage()
{
	
	m_icpEucliFitEpsLable = new QLabel(tr("Eucli.Epsi:"), this);
	m_icpEucliFitEpsSpinBox = new QDoubleSpinBox(this);
	m_icpEucliFitEpsSpinBox->setRange(0, 10);
	m_icpEucliFitEpsSpinBox->setSingleStep(0.01);
	m_icpEucliFitEpsSpinBox->setValue(0.01);

	m_icpIterLabel = new QLabel(tr("MaxIterations:"), this);
	m_icpIterSpinBox = new QSpinBox(this);
	m_icpIterSpinBox->setRange(10, 1000);
	m_icpIterSpinBox->setSingleStep(10);
	m_icpIterSpinBox->setValue(100);

	m_icpButton = new QPushButton(tr("ICP"), this);

	QGridLayout* icpLayout = new QGridLayout;
	icpLayout->addWidget(m_icpIterLabel, 0, 0, 1, 1);
	icpLayout->addWidget(m_icpIterSpinBox, 0, 1, 1, 1);
	icpLayout->addWidget(m_icpEucliFitEpsLable, 1, 0, 1, 1);
	icpLayout->addWidget(m_icpEucliFitEpsSpinBox, 1, 1, 1, 1);
	icpLayout->addWidget(m_icpButton, 2, 1, 1, 1);

	QWidget* icpPage = new QWidget(this);
	icpPage->setLayout(icpLayout);

	return icpPage;
}

QWidget * PointCloudBox::creatBoundaryPage()
{
	//BoundaryEst
	m_boundNormalKSearchLabel = new QLabel(tr("NormalKSearch:"), this);
	m_boundNormalKSpinBox = new QSpinBox(this);
	m_boundNormalKSpinBox->setSingleStep(1);
	m_boundNormalKSpinBox->setValue(15);
	m_boundNormalKSearchLabel->setBuddy(m_boundNormalKSpinBox);

	m_boundKSearchLabel = new QLabel(tr("BoundaryKSearch:"), this);
	m_boundKSearchSpinBox = new QSpinBox(this);
	m_boundKSearchSpinBox->setSingleStep(1);
	m_boundKSearchSpinBox->setValue(30);
	m_boundKSearchLabel->setBuddy(m_boundKSearchSpinBox);

	m_boundaryButton = new QPushButton(tr("Boundary"), this);

	QGridLayout* filterLayout = new QGridLayout;
	filterLayout->addWidget(m_boundNormalKSearchLabel, 0, 0, 1, 1);
	filterLayout->addWidget(m_boundNormalKSpinBox, 0, 1, 1, 1);
	filterLayout->addWidget(m_boundKSearchLabel, 1, 0, 1, 1);
	filterLayout->addWidget(m_boundKSearchSpinBox, 1, 1, 1, 1);
	filterLayout->addWidget(m_boundaryButton, 2, 1, 1, 1);

	QWidget* boundaryPage = new QWidget(this);
	boundaryPage->setLayout(filterLayout);

	return boundaryPage;
}
