#pragma once

#include "stdafx.h"

#include "InterfaceLLT_2.h"


class Scanner
{
public:
	Scanner();
	~Scanner();

	int search(std::vector<unsigned int> &m_vuiInterfaces);
	int connect(unsigned int uiInterface,std::string &scanType);

private:
	CInterfaceLLT* m_pLLT;
	unsigned int m_uiResolution;
	TScannerType m_tscanCONTROLType;

	bool m_bLoadError;
	int m_iRetValue;

};