#include "scanner.h"

Scanner::Scanner()
{
	//加载DLL
	m_pLLT = new CInterfaceLLT("LLT.dll", &m_bLoadError);
	
	//创建以太网连接设备,若失败则
	if (!m_pLLT->CreateLLTDevice(INTF_TYPE_ETHERNET)) {

	}
}

Scanner::~Scanner()
{
	delete m_pLLT;
}

int Scanner::search(std::vector<unsigned int>& m_vuiInterfaces)
{
	//获取可用的IP接口
	m_iRetValue = m_pLLT->GetDeviceInterfacesFast(m_vuiInterfaces.data(), (unsigned int)m_vuiInterfaces.size());
	return m_iRetValue;
}

int Scanner::connect(unsigned int uiInterface,std::string &scanType)
{
	//设定传感器接口
	m_iRetValue = m_pLLT->SetDeviceInterface(uiInterface, 0);
	if (m_iRetValue < GENERAL_FUNCTION_OK)
		return m_iRetValue;

	//连接传感器
	m_iRetValue = m_pLLT->Connect();
	if (m_iRetValue < GENERAL_FUNCTION_OK)
		return m_iRetValue;

	//获取传感器型号
	m_iRetValue = m_pLLT->GetLLTType(&m_tscanCONTROLType);
	if (m_iRetValue < GENERAL_FUNCTION_OK)
		return m_iRetValue;
	
	if (m_tscanCONTROLType >= scanCONTROL28xx_25 && m_tscanCONTROLType <= scanCONTROL28xx_xxx)
		scanType = "scanCONTROL28xx";
	else if (m_tscanCONTROLType >= scanCONTROL27xx_25 && m_tscanCONTROLType <= scanCONTROL27xx_xxx)
		scanType = "scanCONTROL27xx";
	else if (m_tscanCONTROLType >= scanCONTROL26xx_25 && m_tscanCONTROLType <= scanCONTROL26xx_xxx)
		scanType = "scanCONTROL26xx";
	else if (m_tscanCONTROLType >= scanCONTROL29xx_25 && m_tscanCONTROLType <= scanCONTROL29xx_xxx)
		scanType = "scanCONTROL29xx";
	else
		scanType = "undefined";

	return m_iRetValue;
}


