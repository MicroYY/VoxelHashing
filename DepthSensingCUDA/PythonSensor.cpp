#include "stdafx.h"

#include "PythonSensor.h"

#ifdef PythonSensor


PythonSensor::PythonSensor()
{
	const unsigned int colorWidth = 640;
	const unsigned int colorHeight = 480;

	const unsigned int depthWidth = 640;
	const unsigned int depthHeight = 480;

	RGBDSensor::init(depthWidth, depthHeight, depthWidth, depthHeight, 1);



	initializeDepthIntrinsics(518.52905273437500000, 518.34594726562500000, 319.55499267578125000, 239.06936645507812500);
	initializeDepthExtrinsics(mat4f::identity());

	initializeColorIntrinsics(518.52905273437500000, 518.34594726562500000, 319.55499267578125000, 239.06936645507812500);
	initializeColorExtrinsics(mat4f::identity());

}

HRESULT PythonSensor::processDepth()
{
	HRESULT hr = S_OK;
}

#endif // PythonSensor


