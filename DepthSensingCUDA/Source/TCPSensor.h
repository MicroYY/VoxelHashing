#pragma once
#include <opencv2/opencv.hpp>

#include "GlobalAppState.h"
#include "MatrixConversion.h"

#ifdef TCP_SENSOR

#include <WinSock2.h>

#include "RGBDSensor.h"

#pragma comment(lib,"ws2_32.lib")


class TCPSensor :public RGBDSensor
{
public:
	TCPSensor();

	~TCPSensor();

	HRESULT createFirstConnected();

	HRESULT processDepth();

	HRESULT processColor();

	std::string getSensorName() const {
		return "TCPSensor";
	}

	void imgStreamCap();

	void startThread() {
		start = true;
	}

	bool isStart() {
		return start;
	}

	const unsigned int getFrameNum() {
		return frameNum;
	}

	float* getQuanternion()
	{
		return RT;
	}

	mat4f getRigidTransform()
	{
		return m_rigidTransform;
	}

private:
	void quaternion2Mat(float* quaternion);

	//void recvImg(unsigned int bufSize);

	unsigned char* colorMapUchar;

	unsigned char* depthMapUchar;

	unsigned int colorWidth, colorHeight, depthWidth, depthHeight;

	char* recvImg;

	unsigned int bufSize;

	SOCKET clientSocket;

	unsigned int frameNum;

	bool start;
	std::mutex mtx;

	cv::Mat image;

	mat4f				m_rigidTransform;
	float* RT;
	char* recvPose;

	float initPose[7];
};






#endif // TCP_SENSOR
