// Windows.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include <opencv2/opencv.hpp>

#pragma comment(lib, "opencv_core2410.lib")
#pragma comment(lib, "opencv_features2d2410.lib")
#pragma comment(lib, "opencv_flann2410.lib")
#pragma comment(lib, "opencv_highgui2410.lib")
#pragma comment(lib, "opencv_imgproc2410.lib")
#pragma comment(lib, "opencv_objdetect2410.lib")
#pragma comment(lib, "opencv_video2410.lib")

using namespace cv;

int main(int argc, char* argv[])
{
	VideoCapture camera;
	camera.open(0);
	
	Ptr<FeatureDetector> detector = FeatureDetector::create("STAR");
	// 特徴点情報を格納するための変数
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

		// 特徴点抽出の実行
		detector->detect(image, keypoint);
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

