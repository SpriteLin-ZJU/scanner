#include "scanner.h"

Scanner::Scanner()
{
	//����DLL
	m_pLLT = new CInterfaceLLT("LLT.dll", &m_bLoadError);
	
	//������̫�������豸,��ʧ����
	if (!m_pLLT->CreateLLTDevice(INTF_TYPE_ETHERNET)) {

	}
}

Scanner::~Scanner()
{
	delete m_pLLT;
}

int Scanner::search(std::vector<unsigned int>& m_vuiInterfaces)
{
	//��ȡ���õ�IP�ӿ�
	m_iRetValue = m_pLLT->GetDeviceInterfacesFast(m_vuiInterfaces.data(), (unsigned int)m_vuiInterfaces.size());
	return m_iRetValue;
}

int Scanner::connect(unsigned int uiInterface,std::string &scanType)
{
	//�趨�������ӿ�
	m_iRetValue = m_pLLT->SetDeviceInterface(uiInterface, 0);
	if (m_iRetValue < GENERAL_FUNCTION_OK)
		return m_iRetValue;

	//���Ӵ�����
	m_iRetValue = m_pLLT->Connect();
	if (m_iRetValue < GENERAL_FUNCTION_OK)
		return m_iRetValue;

	//��ȡ�������ͺ�
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


