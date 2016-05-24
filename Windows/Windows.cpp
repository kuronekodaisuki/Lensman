// Windows.cpp : 
//

#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <opencv2/ocl/ocl.hpp>

#pragma comment(lib, "opencv_core2410.lib")
#pragma comment(lib, "opencv_features2d2410.lib")
#pragma comment(lib, "opencv_flann2410.lib")
#pragma comment(lib, "opencv_highgui2410.lib")
#pragma comment(lib, "opencv_imgproc2410.lib")
#pragma comment(lib, "opencv_ml2410.lib")
#pragma comment(lib, "opencv_objdetect2410.lib")
#pragma comment(lib, "opencv_ocl2410.lib")
#pragma comment(lib, "opencv_video2410.lib")

using namespace cv;
using namespace cv::ocl;

void checkOpenCL()
{
	DevicesInfo devices;
	getOpenCLDevices(devices);

	for (size_t i = 0; i < devices.size(); i++)
	{
		const DeviceInfo *info = devices[i];
		printf("%s : %s\nMax Compute unit:%d\n%s\n%s", 
			info->deviceName.c_str(), info->deviceVersion.c_str(), info->maxComputeUnits, 
			info->compilationExtraOptions.c_str(),
			info->deviceExtensions.c_str());
	}
}

int main(int argc, char* argv[])
{
	VideoCapture camera;
	camera.open(0);
	
	checkOpenCL();

	Ptr<FeatureDetector> detector = FeatureDetector::create("STAR");
	GoodFeaturesToTrackDetector_OCL ocl;

	// features
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

		// detect features
		detector->detect(image, keypoint);
		
		// OpenCL
		//oclMat oclImage;
		//oclMat oclCorners;
		//ocl(oclImage, oclCorners);

		printf("%d\n", keypoint.size());
		for (int i = 0; i < keypoint.size(); i++)
		{
			Point2f pt = keypoint[i].pt;
			circle(image, Point(pt.x, pt.y), 3, Scalar(0, 0, 255));
		}
		imshow("image", image);
	}
	return 0;
}

