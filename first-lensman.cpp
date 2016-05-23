// DragonBoard410c

#include <stdio.h>
#include <opencv2/opencv.hpp>


using namespace cv;

int main(int argc, char* argv[])
{
	VideoCapture camera;
	camera.open(0);
	
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

		//
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

