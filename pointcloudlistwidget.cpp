#include "pointcloudlistwidget.h"
#include <QtGui>
PointCloudListWidget::PointCloudListWidget(QWidget* parent)
	:QListWidget(parent)
{
	this->setSelectionMode(QAbstractItemView::ExtendedSelection);
	this->setAcceptDrops(true);
}

PointCloudListWidget::~PointCloudListWidget()
{
}

void PointCloudListWidget::onAddPointCloudList(const QString & srcName)
{
	this->addItem(srcName);
}

void PointCloudListWidget::dragEnterEvent(QDragEnterEvent* event)
{
	if (event->mimeData()->hasFormat("text/uri-list"))
		event->acceptProposedAction();
	event->accept();
}

void PointCloudListWidget::dragMoveEvent(QDragMoveEvent * event)
{
}

void PointCloudListWidget::dropEvent(QDropEvent * event)
{
	QList<QUrl> urls = event->mimeData()->urls();
	if (urls.isEmpty())
		return;

	for (auto url : urls) {
		QString path = url.toLocalFile();
		emit signalDragPcdFile(path);
	}
}


