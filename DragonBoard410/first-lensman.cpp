// DragonBoard410c

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ocl/ocl.hpp>
#include <pthread.h>

using namespace cv;
using namespace cv::ocl;

#define WIDTH 640
#define HEIGHT 480

Mat image;
Mat buffer(HEIGHT, WIDTH, CV_8UC3);
Ptr<FeatureDetector> detector = FeatureDetector::create("STAR");
std::vector<KeyPoint> keypoint;


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


void thread_feature(void *arg)
{
	wait(1000);
}

int main(int argc, char* argv[])
{
	VideoCapture camera;

	camera.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
	camera.open(0);

	checkOpenCL();
	
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

