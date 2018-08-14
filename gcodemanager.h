#pragma once
#include <QObject>

class GcodeManager : public QObject
{
	Q_OBJECT
public:
	GcodeManager();
	~GcodeManager();

	void clear();
	void addCommand(const QString& command);
	void addDrawerBuffer(const QString& command);
	int fileSize();
	QString takeFirstGcode();
	QString takeFirstBuffer();
	const QString& firstGcode();
	void removeFirstGcode();
	bool fileIsEmpty();
	QList<QString>::iterator fileBegin();
	QList<QString>::iterator fileEnd();
private:
	QList<QString> m_fileGcode;
	QList<QString> m_gcodeDrawerBuffer;
};