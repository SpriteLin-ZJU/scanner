#pragma once
#include <QListWidget>
class PointCloudListWidget :public QListWidget
{
	Q_OBJECT
public:
	PointCloudListWidget(QWidget* parent = 0);
	~PointCloudListWidget();
	
	//slots
	void onAddPointCloudList(const QString& srcName);
signals:
	void signalDragPcdFile(const QString& url);
protected:
	//拖入打开点云文件
	void dragEnterEvent(QDragEnterEvent* event) override;
	void dragMoveEvent(QDragMoveEvent* event) override;
	void dropEvent(QDropEvent* event) override;
private:

};