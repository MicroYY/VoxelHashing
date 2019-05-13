#pragma once
//////////////////////////////////////////////////////////////////////////  
/// COPYRIGHT NOTICE  
/// Copyright (c) 2009, ���пƼ���ѧtickTick Group  ����Ȩ������  
/// All rights reserved.  
///   
/// @file    SerialPort.h    
/// @brief   ����ͨ����ͷ�ļ�  
///  
/// ���ļ���ɴ���ͨ���������  
///  
/// @version 1.0     
/// @author  ¬��   
/// @E-mail��lujun.hust@gmail.com  
/// @date    2010/03/19  
///  
///  �޶�˵����  
//////////////////////////////////////////////////////////////////////////  

#ifndef SERIALPORT_H_  
#define SERIALPORT_H_  
#include <Windows.h>


class CSerialPort
{
public:
	CSerialPort(void);
	~CSerialPort(void);

public:


	bool InitPort(UINT  portNo = 1, UINT  baud = CBR_115200, char  parity = 'N', UINT  databits = 8, UINT  stopsbits = 1, DWORD dwCommEvents = EV_RXCHAR);


	bool InitPort(UINT  portNo, const LPDCB& plDCB);


	bool OpenListenThread();


	bool CloseListenTread();


	bool WriteData(unsigned char* pData, unsigned int length);


	UINT GetBytesInCOM();


	bool ReadChar(char &cRecved);

private:


	bool openPort(UINT  portNo);


	void ClosePort();


	static UINT WINAPI ListenThread(void* pParam);

private:


	HANDLE  m_hComm;


	static bool s_bExit;


	volatile HANDLE    m_hListenThread;


	CRITICAL_SECTION   m_csCommunicationSync;       //!< �����������  

};

#endif //SERIALPORT_H_ 