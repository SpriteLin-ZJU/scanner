#include "scanner.h"

Scanner::Scanner()
{
	m_pLLT = new CInterfaceLLT("LLT.dll", &m_bLoadError);
}

Scanner::~Scanner()
{
	delete m_pLLT;
}
