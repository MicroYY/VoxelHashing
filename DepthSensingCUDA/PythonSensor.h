#pragma once

#include "GlobalAppState.h"

#ifdef Python_Sensor

#include "RGBDSensor.h"
#include <Python.h>

class PythonSensor :public RGBDSensor
{
public:
	PythonSensor();


	~PythonSensor();

	HRESULT createfirstconnect();

	HRESULT processDepth();

	HRESULT processColor();


	std::string getSensorName() const {
		return "PythonSensor";
	}






};






#endif // Python_Sensor
