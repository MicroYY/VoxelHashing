#include "stdafx.h"

#include "MynteyeSensor.h"

#ifdef MYNTEYE


MynteyeSensor::MynteyeSensor()
{
	colorWidth = 1280;
	colorHeight = 720;

	depthWidth = 1280;
	depthHeight = 720;

	RGBDSensor::init(depthWidth, depthHeight, depthWidth, depthHeight, 1);

	initializeDepthIntrinsics(1039.90344238281250000, 1039.85351562500000000, 640.06604003906250000, 357.32925415039062500);
	initializeDepthExtrinsics(mat4f::identity());

	initializeColorIntrinsics(1039.90344238281250000, 1039.85351562500000000, 640.06604003906250000, 357.32925415039062500);
	initializeColorExtrinsics(mat4f::identity());


}

MynteyeSensor::~MynteyeSensor()
{
	cam.Close();
}

HRESULT MynteyeSensor::createFirstConnected()
{
	if (!mynteyed::util::select(cam, &dev_info)) {
		return S_FALSE;
	}
	mynteyed::util::print_stream_infos(cam, dev_info.index);

	std::cout << "Open device: " << dev_info.index << ", "
		<< dev_info.name << std::endl << std::endl;

	mynteyed::OpenParams params(dev_info.index);
	params.depth_mode = mynteyed::DepthMode::DEPTH_RAW;
	params.stream_mode = mynteyed::StreamMode::STREAM_1280x720;
	params.ir_intensity = 4;
	params.framerate = 30;

	cam.Open(params);

	std::cout << std::endl;
	if (!cam.IsOpened()) {
		std::cerr << "Error: Open camera failed" << std::endl;
		return S_FALSE;
	}
	std::cout << "Open device success" << std::endl << std::endl;
	return S_OK;
}

HRESULT MynteyeSensor::processDepth()
{
	auto image_depth = cam.GetStreamData(mynteyed::ImageType::IMAGE_DEPTH);
	if (!image_depth.img)
		return S_FALSE;
	float* depth = getDepthFloat();
	const ushort* depth_buffer = (ushort*)image_depth.img->data();
	for (size_t i = 0; i < depthHeight; i++)
	{
		for (size_t j = 0; j < depthWidth; j++)
		{
			unsigned int index = i * depthWidth + j;
			//std::cout << index << std::endl;
			const ushort& d = depth_buffer[index];
			//std::cout << index << " " << d << std::endl;
			if (d == 0)
				depth[index] = -std::numeric_limits<float>::infinity();
			else
			{
				depth[index] = (float)d* 0.001f;
			}
		}
	}
	return S_OK;
}

HRESULT MynteyeSensor::processColor()
{
	auto left_color = cam.GetStreamData(mynteyed::ImageType::IMAGE_LEFT_COLOR);
	if (!left_color.img)
		return S_FALSE;

	const uchar* color_buffer = left_color.img->data();
	for (size_t i = 0; i < colorHeight * colorWidth * 2; i += 4)
	{
		unsigned char y, u, v, r, g, b;
		y = color_buffer[i + 0];
		u = color_buffer[i + 1];
		v = color_buffer[i + 3];
		b = 1.164*(y - 16) + 1.159*(v - 128);
		g = 1.164*(y - 16) - 0.380*(u - 128) + 0.813*(v - 128);
		r = 1.164*(y - 16) + 2.018*(u - 128);
		m_colorRGBX[i / 2].x = b;
		m_colorRGBX[i / 2].y = g;
		m_colorRGBX[i / 2].z = r;
		m_colorRGBX[i / 2].w = 255;

		y = color_buffer[i + 2];
		u = color_buffer[i + 1];
		v = color_buffer[i + 3];
		b = 1.164*(y - 16) + 1.159*(v - 128);
		g = 1.164*(y - 16) - 0.380*(u - 128) + 0.813*(v - 128);
		r = 1.164*(y - 16) + 2.018*(u - 128);
		m_colorRGBX[i / 2 + 1].x = b;
		m_colorRGBX[i / 2 + 1].y = g;
		m_colorRGBX[i / 2 + 1].z = r;
		m_colorRGBX[i / 2 + 1].w = 255;
	}
	return S_OK;
}

#endif // MYNTEYE