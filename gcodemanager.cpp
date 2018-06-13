#include "gcodemanager.h"

GcodeManager::GcodeManager()
{
}

GcodeManager::~GcodeManager()
{
}

void GcodeManager::clear()
{
	m_fileGcode.clear();
	m_gcodeDrawerBuffer.clear();
}

void GcodeManager::addCommand(const QString & command)
{
	m_fileGcode.append(command);
}

void GcodeManager::addDrawerBuffer(const QString & command)
{
	m_gcodeDrawerBuffer.append(command);
}

int GcodeManager::fileSize()
{
	return m_fileGcode.size();
}

QString GcodeManager::takeFirstGcode()
{
	return m_fileGcode.takeFirst();
}

QString GcodeManager::takeFirstBuffer()
{
	return m_gcodeDrawerBuffer.takeFirst();
}

const QString & GcodeManager::firstGcode()
{
	return m_fileGcode.at(0);
}

void GcodeManager::removeFirstGcode()
{
	m_fileGcode.removeFirst();
}

bool GcodeManager::fileIsEmpty()
{
	return m_fileGcode.isEmpty();
}
