// DragonBoard410c

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ocl/ocl.hpp>
#include <pthread.h>
#include "I2C.h"
#include "KalmanFilter/Kalman.h"

using namespace cv;
using namespace cv::ocl;

#define WIDTH 640
#define HEIGHT 480

Mat image;
Mat buffer(HEIGHT, WIDTH, CV_8UC3);
Ptr<FeatureDetector> detector = FeatureDetector::create("STAR");
std::vector<KeyPoint> keypoint;

static pthread_mutex_t	mutex;
static MPU_6050 mpu6050;
static AXDL345  axdl345;

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


void *thread_feature(void *arg)
{
	usleep(1000 * 100);
	return NULL;
}

void *thread_sensor(void* arg)
{
	mpu6050.Init();
	while (1)
	{
		double x, y, z;
		x = mpu6050.AccelX();
		y = mpu6050.AccelY();
		z = mpu6050.AccelZ();
		printf("X:%8.3f Y:%8.3f Z:%8.3f \n", x, y, z);
		usleep(1000 * 100);
	}
	return NULL;
}


int main(int argc, char* argv[])
{
	VideoCapture camera;

	camera.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
	camera.open(0);

	checkOpenCL();
	
	pthread_t thread;
	pthread_create(&thread, NULL, thread_sensor, NULL);

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

