#include <WinSock2.h>
#include <Windows.h>
#include <math.h>
#include <iostream>
#include <thread>


#include <Eigen\Dense>
#include <iomanip>

#pragma comment(lib, "ws2_32.lib")

#define PI M_PI
#define FileMapping_NAME "pose"


float radius = 0.2f;
float breakThres = 0.28f;
float upThres = 0.025f;
float downThres = -0.05f;
bool outOfRange = false;// 记录是否出圈
int timeCount = 0;
Eigen::Vector3f interframeOffset(0.0f, 0.0f, 0.0f);
Eigen::Vector3f lastPos(0.0f, 0.0f, 0.0f);
Eigen::Vector3f accumDis(0.0f, 0.0f, 0.0f);
Eigen::Vector3f UAV_velocity(0.0f, 0.0f, 0.0f);
Eigen::Vector3f userVelocity(0.0f, 0.0f, 0.0f);

char sendData[12];
SOCKET clientSocket;

void getEulerAngleAndPositionFromHMDPose(float* pose, float& roll, float& pitch, float& yaw, float& position_x, float& position_y, float& position_z)
{
	roll = (atan2(2.0f * (pose[3] * pose[2] + pose[0] * pose[1]), 1 - 2.0f  *(pose[0] * pose[0] + pose[2] * pose[2]))) * 180.0f / PI;
	pitch = asin(2 * (pose[3] * pose[0] - pose[2] * pose[1])) * 180.0f / PI;
	yaw = atan2(2 * (pose[3] * pose[1] + pose[0] * pose[2]), 1 - 2 * (pose[1] * pose[1] + pose[0] * pose[0])) * 180.0f / PI;
	//调整坐标系的方向，改为左前上为正方向
	position_x = -pose[6]; position_y = -pose[4]; position_z = pose[5];
	if (roll < 0.0f)
		roll += 360.0f;
	if (pitch < 0.0f)
		pitch += 360.0f;
	if (yaw < 0.0f)
		yaw += 360.0f;
}
//
//void getEulerAngleAndPositionFromPose(float* pose, 
//	float& roll, float& pitch, float& yaw, 
//	float& position_x, float& position_y, float& position_z,
//	float initYaw)
//{
//	roll = (atan2(2.0f * (pose[3] * pose[2] + pose[0] * pose[1]), 1 - 2.0f  *(pose[0] * pose[0] + pose[2] * pose[2]))) * 180.0f / PI;
//	pitch = asin(2 * (pose[3] * pose[0] - pose[2] * pose[1])) * 180.0f / PI;
//	yaw = atan2(2 * (pose[3] * pose[1] + pose[0] * pose[2]), 1 - 2 * (pose[1] * pose[1] + pose[0] * pose[0])) * 180.0f / PI - initYaw;
//	position_x = pose[6]; position_y = pose[4]; position_z = pose[5];
//	if (roll < 0.0f)
//		roll += 360.0f;
//	if (pitch < 0.0f)
//		pitch += 360.0f;
//	if (yaw < 0.0f)
//		yaw += 360.0f;
//}

void getPositionFromHMDPose(float* pose, float& position_x, float& position_y, float& position_z)
{
	position_x = -pose[6]; position_y = -pose[4]; position_z = pose[5];
}

void __send__()
{
	while (1)
	{
		try
		{
			send(clientSocket, (char*)sendData, 12, 0);
			Sleep(30);
		}
		catch (const std::exception&)
		{
			std::cout << "发送控制指令失败！" << std::endl;
		}
	}
}

int main()
{
	TCHAR szName[] = TEXT("pose");
	HANDLE hMapFile = NULL;
	LPVOID lpbase = NULL;
	while (hMapFile == NULL)
	{
		hMapFile = OpenFileMapping(FILE_MAP_READ, FALSE, szName);
	}
	while (lpbase == NULL)
	{
		lpbase = MapViewOfFile(hMapFile, FILE_MAP_READ, 0, 0, 0);
	}

	std::thread thread;
	bool threadStarted = false;

	WSADATA wsaData;
	int iRet = 0;
	iRet = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iRet != 0)
	{
		std::cout << "WSAStartup(MAKEWORD(2, 2), &wsaData) execute failed!" << std::endl;
		return 0;
	}

	if (2 != LOBYTE(wsaData.wVersion) || 2 != HIBYTE(wsaData.wVersion))
	{
		WSACleanup();
		std::cout << "WSADATA version is not correct!" << std::endl;
		return 0;
	}

	clientSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (clientSocket == INVALID_SOCKET)
	{
		std::cout << "clientSocket = socket(AF_INET, SOCK_STREAM, 0) execute failed!" << std::endl;
		return 0;
	}

	SOCKADDR_IN srvAddr;

	srvAddr.sin_addr.S_un.S_addr = inet_addr("192.168.1.100");
	srvAddr.sin_family = AF_INET;
	srvAddr.sin_port = htons(1234);

	iRet = connect(clientSocket, (SOCKADDR*)&srvAddr, sizeof(SOCKADDR));
	if (iRet != 0)
	{
		std::cout << "connect(clientSocket, (SOCKADDR*)&srvAddr, sizeof(SOCKADDR)) execute failed!" << std::endl;
		return 0;
	}
	printf("本地的socket连接已建立");

	float* pose = (float*)lpbase;
	size_t bufSize = sizeof(float) * 7;
	float* tmpPose = (float*)malloc(bufSize);
	float roll, pitch, yaw;
	float quaternion_w, quaternion_x, quaternion_y, quaternion_z, position_x, position_y, position_z;

	//roll = (atan2(2.0f*(quaternion_w*quaternion_z + quaternion_x * quaternion_y), 1 - 2.0f*(quaternion_x*quaternion_x + quaternion_z * quaternion_z))) * 180.0 / PI;
	//pitch = asin(2 * (quaternion_w*quaternion_x - quaternion_z * quaternion_y)) * 180.0 / PI;
	//yaw = atan2(2 * (quaternion_w*quaternion_y + quaternion_x * quaternion_z), 1 - 2 * (quaternion_y*quaternion_y + quaternion_x * quaternion_x)) * 180.0 / PI;
	//position_x = tmpPose[6]; position_y = tmpPose[4]; position_z = tmpPose[5];

	Sleep(5000);

	std::cout << "开始读取盔数据" << std::endl;
	memcpy(tmpPose, pose, bufSize);
	getEulerAngleAndPositionFromHMDPose(tmpPose, roll, pitch, yaw, position_x, position_y, position_z);
	float initYaw = yaw;
	float initPitch = pitch;

	Eigen::Vector3f initPosition(position_x, position_y, position_z);
	lastPos = initPosition;
	std::cout << std::endl << "Initial position:" << std::endl;
	std::cout << initPosition(0) << std::endl << initPosition(1) << std::endl << initPosition(2) << std::endl << std::endl;
	float initHeight = initPosition(2);

	int count = 0;
	while (1)
	{
		memcpy(tmpPose, pose, bufSize);
		getEulerAngleAndPositionFromHMDPose(tmpPose, roll, pitch, yaw, position_x, position_y, position_z);

		yaw -= initYaw;
#ifdef SIMULATION
		if (yaw < 0.0f)
			yaw += 360.0f;

#else
		while (yaw < 0.0f)
		{
			yaw += 360.0f;
		}
		while (yaw > 360.0f)
		{
			yaw -= 360.0f;
		}
#endif // SIMULATION

		//pitch -= initPitch;

		UAV_velocity(0) = 0.2f;
		UAV_velocity(1) = 0.0f;
		UAV_velocity(2) = 0.0f;


		Eigen::Vector3f currentPosition(position_x, position_y, position_z);
		Eigen::Vector3f relativePosition = currentPosition - initPosition;

		float deltaX = relativePosition(0);
		if (deltaX >= radius)
		{
			UAV_velocity(0) = 0.2f + (deltaX - radius) * 5;
		}

		if (deltaX <= -radius / 2)
		{
			UAV_velocity(0) = 0.0f;
		}


		if (pitch > 45.0f)
		{
			pitch = 45.0f;
		}
		if (pitch < -45.0f)
		{
			pitch = -45.0f;
		}
		if (pitch > 0.0f && pitch < 5.0f)
		{
			pitch = 0;
		}
		if (pitch < 0.0f && pitch > -5.0f)
		{
			pitch = 0;
		}

		UAV_velocity(2) = pitch / 90;

		if (roll >= 0.0f && roll < 5.0f)
		{
			roll = 0.0f;
		}
		if (roll > 355.0f)
		{
			roll = 0.0f;
		}
		if (roll > 180.0f)
		{
			roll = 360.0f - roll;
			if (roll > 45.0f)
			{
				roll = 45.0f;
			}
			roll = -roll;
		}
		else
		{
			if (roll > 45.0f)
			{
				roll = 45.0f;
			}
		}



		float yawRate = roll / 5;


		
		count++;
		if (count == 10000000)
		{
			count = 0;
			std::cout << "yaw: " << yaw << std::setw(4) << "        Pitch: " << pitch << std::endl;
			std::cout << "Pitch: " << pitch << std::endl;
			std::cout << "Roll: " << roll << std::endl;
			std::cout << "UAV velocity: " << std::endl;
			std::cout << UAV_velocity << std::endl;
			std::cout << "User velocity: " << std::endl;
			std::cout << userVelocity << std::endl << std::endl;

		}
		//sendData[0] = UAV_velocity(0); sendData[1] = UAV_velocity(1); sendData[2] = UAV_velocity(2);
		//sendData[3] = yaw; sendData[4] = pitch; sendData[5] = roll;
		//sendData[0] = 1; sendData[1] = 2; sendData[2] = 3;

		//send(clientSocket, (char*)sendData, sizeof(float) * 6, 0);
		short yaw0 = short(yawRate * 100.0);
		short pitch0 = short(pitch * 100.0);
		short roll0 = short(roll * 100.0);
		short vx = short(UAV_velocity(0) * 100.0) + 500;
		short vy = short(UAV_velocity(1) * 100.0) + 500;
		short vz = short(UAV_velocity(2) * 100.0) + 500;

		//std::cout << vx << " " << vy << " " << vz << std::endl;

		//sendData[0] = yaw0; sendData[1] = pitch0; sendData[2] = roll0;
		memcpy(sendData + 0, &yaw0, 2);
		memcpy(sendData + 2, &pitch0, 2);
		memcpy(sendData + 4, &roll0, 2);
		memcpy(sendData + 6, &vx, 2);
		memcpy(sendData + 8, &vy, 2);
		memcpy(sendData + 10, &vz, 2);
		//sendData[3] = vx; sendData[4] = vy; sendData[5] = vz;

		//std::cout << (int)sendData[3] << " " << (int)sendData[4] << " " << (int)sendData[3] << std::endl;
		//std::cout << (int)sendData[0] << " " << (int)sendData[1] << " " << (int)sendData[2] << std::endl;
		if (!threadStarted)
		{
			thread = std::thread(__send__);
			threadStarted = true;
		}
	}

	closesocket(clientSocket);
	WSACleanup();
}
