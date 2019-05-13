#pragma once
//////////////////////////////////////////////////////////////////////////  
/// COPYRIGHT NOTICE  
/// Copyright (c) 2009, 华中科技大学tickTick Group  （版权声明）  
/// All rights reserved.  
///   
/// @file    SerialPort.h    
/// @brief   串口通信类头文件  
///  
/// 本文件完成串口通信类的声明  
///  
/// @version 1.0     
/// @author  卢俊   
/// @E-mail：lujun.hust@gmail.com  
/// @date    2010/03/19  
///  
///  修订说明：  
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


	CRITICAL_SECTION   m_csCommunicationSync;       //!< 互斥操作串口  

};

#endif //SERIALPORT_H_ 