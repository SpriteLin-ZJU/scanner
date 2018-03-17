#pragma once

#include "stdafx.h"
#include <memory>

#include "InterfaceLLT_2.h"


class Scanner
{
public:
	Scanner();
	~Scanner();

private:
	static CInterfaceLLT* m_pLLT;
	static unsigned int m_uiResolution;
	static TScannerType m_tscanCONTROLType;

	std::vector<unsigned int> m_vuiInterfaces;
	std::vector<DWORD> vdwResolutions;
	bool bLoadError;
	int iRetValue;
};