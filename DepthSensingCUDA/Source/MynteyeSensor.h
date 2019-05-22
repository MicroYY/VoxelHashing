#pragma once

#include "GlobalAppState.h"
#include "MatrixConversion.h"

#ifdef MYNTEYE

#include "RGBDSensor.h"

#include <mynteyed/camera.h>
#include <mynteyed/utils.h>

#pragma comment(lib, "mynteye_depth.lib")

class MynteyeSensor :public RGBDSensor
{
public:
	MynteyeSensor();

	~MynteyeSensor();

	HRESULT createFirstConnected();

	HRESULT processDepth();

	HRESULT processColor();

	std::string getSensorName() const
	{
		return "MynteyeSensor";
	}

	const unsigned int getFrameNum() {
		return frameNum;
	}


private:
	unsigned int colorWidth, colorHeight, depthWidth, depthHeight;

	mynteyed::Camera cam;
	mynteyed::DeviceInfo dev_info;

	unsigned int frameNum;
};

#endif // MYNTEYE