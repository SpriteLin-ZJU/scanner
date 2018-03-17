#include "scanner.h"

Scanner::Scanner()
{
	m_pLLT = new CInterfaceLLT("LLT.dll", &bLoadError);
}

Scanner::~Scanner()
{
	delete m_pLLT;
}
