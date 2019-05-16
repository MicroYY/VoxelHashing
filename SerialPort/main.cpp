#include <string>
#include <iostream>
#include <windows.h>

#include "SerialPort.h"

#define FileMapping_NAME "pose"
#define PI 3.1415926

int main()
{
	HANDLE hMapFile = NULL;
	LPVOID lpbase = NULL;
	while (hMapFile == NULL)
	{
		hMapFile = OpenFileMappingA(FILE_MAP_READ, FALSE, FileMapping_NAME);

	}
	while (lpbase == NULL)
		lpbase = MapViewOfFile(hMapFile, FILE_MAP_READ, 0, 0, 0);
	//unsigned char pos[6] = {0,1,2,3,4,5};

	CSerialPort mySerialPort;

	char lpComm[5];
	//std::cout << "请设置端口号（数字）：";
	std::string sData;
	int iComm = 2;
	//std::cin >> iComm;
	sprintf(lpComm, "com%d", iComm);	//将端口号赋给szComm
	std::cout << "请设置模式【读(r)|写(w)】：";
	char cMode = 'w';
	//std::cin >> cMode;
	if (!mySerialPort.InitPort(iComm))
	{
		std::cout << "端口打开失败" << std::endl;
		system("pause");
		return 1;
	}
	if (cMode == 'r')
	{			//读模式
		mySerialPort.OpenListenThread();
		std::cout << "正在侦听端口，如需结束，请按q退出";
		system("pause");
	}

	else if (cMode == 'w')
	{	//写模式
		float* pose = (float*)lpbase;
		float roll, pitch, yaw, quaternion_w, quaternion_x, quaternion_y, quaternion_z, position_x, position_y, position_z;
		short roll_send = 0, pitch_send = 0, yaw_send = 0, x_send = 0, y_send = 0, z_send = 0;
		unsigned char checkSum = 0;

		unsigned char outString[] = "000000000000000";
		while (1)
		{
			//std::cout << pose[0] << " " << pose[1] << " " << pose[2] << " " << pose[3] << std::endl;
			//std::cout << pose[4] << " " << pose[5] << " " << pose[6] << std::endl << std::endl;
			quaternion_w = pose[3]; quaternion_x = pose[0]; quaternion_y = pose[1]; quaternion_z = pose[2];
			position_x = pose[6]; position_y = pose[4]; position_z = pose[5];


			roll = (atan2(2.0f*(quaternion_w*quaternion_z + quaternion_x * quaternion_y), 1 - 2.0f*(quaternion_x*quaternion_x + quaternion_z * quaternion_z))) * 180.0 / PI;
			pitch = asin(2 * (quaternion_w*quaternion_x - quaternion_z * quaternion_y)) * 180.0 / PI;
			yaw = atan2(2 * (quaternion_w*quaternion_y + quaternion_x * quaternion_z), 1 - 2 * (quaternion_y*quaternion_y + quaternion_x * quaternion_x)) * 180.0 / PI;

			std::cout << "rotation: " << roll << " " << pitch << " " << yaw << std::endl;
			std::cout << "position: " << position_x << " " << position_y << " " << position_z << std::endl << std::endl;

			yaw_send = yaw * 10 + 1800;//云台滚转
			roll_send = roll * 10 + 1800;//云台俯仰
			pitch_send = pitch * 10 + 1800;//机旋转
			x_send = position_x * 100 + 300;
			y_send = position_y * 100 + 300;
			z_send = position_z * 100 + 300;

			outString[0] = 0xaa;
			outString[1] = 0xaf;
			outString[2] = 0x01;
			outString[3] = 0x0A;
			outString[4] = (roll_send >> 8) & 0xff;
			outString[5] = (roll_send) & 0xff;
			outString[6] = (x_send >> 8) & 0xff;
			outString[7] = (x_send) & 0xff;
			outString[8] = (y_send >> 8) & 0xff;
			outString[9] = (y_send) & 0xff;
			outString[10] = (yaw_send >> 8) & 0xff;
			outString[11] = (yaw_send) & 0xff;
			outString[12] = (pitch_send >> 8) & 0xff;
			outString[13] = (pitch_send) & 0xff;
			checkSum = 0;
			for (int i = 0; i < 14; i++)
			{
				checkSum += outString[i];
			}
			outString[14] = checkSum;

			//memcpy(outString, lpbase, 15);
			//cin >> sData;
			mySerialPort.WriteData(outString, 15);
			Sleep(100);
			//mySerialPort.WriteData((char *)sData, sData.length);
			//mySerialPort.WriteData(pos, 6);
		}

	}
	return 0;
}