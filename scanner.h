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
	CInterfaceLLT* m_pLLT;
	unsigned int m_uiResolution;
	TScannerType m_tscanCONTROLType;

	std::vector<unsigned int> m_vuiInterfaces;
	std::vector<DWORD> vdwResolutions;
	bool bLoadError;
	int iRetValue;
};