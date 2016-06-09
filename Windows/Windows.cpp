// Windows.cpp : 
//

#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <opencv2/ocl/ocl.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

#ifdef _DEBUG
#pragma comment(lib, "opencv_core2410d.lib")
#pragma comment(lib, "opencv_features2d2410d.lib")
#pragma comment(lib, "opencv_flann2410d.lib")
#pragma comment(lib, "opencv_highgui2410d.lib")
#pragma comment(lib, "opencv_imgproc2410d.lib")
#pragma comment(lib, "opencv_ml2410d.lib")
#pragma comment(lib, "opencv_nonfree2410d.lib")
#pragma comment(lib, "opencv_objdetect2410d.lib")
#pragma comment(lib, "opencv_ocl2410d.lib")
#pragma comment(lib, "opencv_video2410d.lib")
#else
#pragma comment(lib, "opencv_core2410.lib")
#pragma comment(lib, "opencv_features2d2410.lib")
#pragma comment(lib, "opencv_flann2410.lib")
#pragma comment(lib, "opencv_highgui2410.lib")
#pragma comment(lib, "opencv_imgproc2410.lib")
#pragma comment(lib, "opencv_ml2410.lib")
#pragma comment(lib, "opencv_nonfree2410.lib")
#pragma comment(lib, "opencv_objdetect2410.lib")
#pragma comment(lib, "opencv_ocl2410.lib")
#pragma comment(lib, "opencv_video2410.lib")
#endif

using namespace cv;
using namespace cv::ocl;

#define WIDTH 640
#define HEIGHT 480

Mat image;
Mat buffer(640, 480, CV_8UC3);
// features
std::vector<KeyPoint> keypoint[2];

//GoodFeaturesToTrackDetector_OCL ocl;

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
	camera.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
	camera.open(0);
	
	//checkOpenCL();
	Ptr<FeatureDetector> detector = FeatureDetector::create("STAR");
	//Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("FREAK");
	BriefDescriptorExtractor extractor;
	FlannBasedMatcher matcher;
	std::vector< DMatch > matches;
	Mat descriptor[2];

	int k = 0;
	camera >> image;
	detector->detect(image, keypoint[1]);
	extractor.compute(image, keypoint[1], descriptor[1]);
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
		detector->detect(image, keypoint[k % 2]);
		extractor.compute(image, keypoint[k % 2], descriptor[k % 2]);
		try {
			matcher.match(descriptor[0], descriptor[1], matches);
		}
		catch (Exception ex)
		{
			printf("%s", ex.msg);
		}
		printf("%d\n", keypoint[k % 2].size());
		for (int i = 0; i < keypoint[k % 2].size(); i++)
		{
			Point2f pt = keypoint[k % 2][i].pt;
			circle(image, Point(pt.x, pt.y), 3, Scalar(0, 0, 255));
		}
		k++;
		imshow("image", image);
	}
	return 0;
}

