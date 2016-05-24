// DragonBoard410c

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ocl/ocl.hpp>

using namespace cv;
using namespace cv::ocl;

void checkOpenCL()
{
	PlatformsInfo platforms;
	getOpenCLPlatforms(platforms);

	for (size_t p = 0; p < platforms.size(); p++)
	{
		DevicesInfo devices;
		getOpenCLDevices(devices, CVCL_DEVICE_TYPE_GPU, platforms[p]);

		for (size_t i = 0; i < devices.size(); i++)
		{
		    	const DeviceInfo *info = devices[i];
		    	printf("%s : %s\n", info->deviceName.c_str(), info->deviceVersion.c_str());
		}
	}
}

int main(int argc, char* argv[])
{
	VideoCapture camera;
	camera.open(0);

	checkOpenCL();
	
	Ptr<FeatureDetector> detector = FeatureDetector::create("STAR");
	//
	std::vector<KeyPoint> keypoint;

	Mat image;
	Mat buffer(640, 480, CV_8UC3, NULL);
	for (bool loop = true; loop; )
	{
		switch (waitKey(10))
		{
		case 'q':
			loop = false;
			break;
		}
		camera >> image;
		if (image.empty())
			break;

		// Detect feature points every second
		// and interpolate with IMU information
		detector->detect(image, keypoint);

		printf("%d\n", (int)keypoint.size());
		for (unsigned int i = 0; i < keypoint.size(); i++)
		{
			Point2f pt = keypoint[i].pt;
			circle(image, Point(pt.x, pt.y), 3, Scalar(0, 0, 255));
		}
		imshow("image", image);
	}
	return 0;
}
