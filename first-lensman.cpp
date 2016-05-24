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
<<<<<<< HEAD
	SDL_Rect srcRect = {0, 0, WIDTH, HEIGHT};
	SDL_Rect dstRect = {0, 0};
	SDL_BlitSurface(frame, &srcRect, screen, &dstRect);
	SDL_Flip(screen);
	memcpy(image.data, frame->pixels, 3 * WIDTH * HEIGHT);
}
=======
	VideoCapture camera;
	camera.open(0);
>>>>>>> fb00f1b4c5ef512077df1b0ccaac9e5ec80a48d9

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

<<<<<<< HEAD
int main (int argc, char *argv[])
{
        int quit = 0;
	SDL_Init(SDL_INIT_EVERYTHING);
        screen = SDL_SetVideoMode(WIDTH, HEIGHT, 24, SDL_HWSURFACE | SDL_DOUBLEBUF);
    	 
	frame = SDL_CreateRGBSurface(SDL_SWSURFACE, WIDTH, HEIGHT, 24,   
			0x000000ff, 0x0000ff00, 0x00ff0000, 0);
	image.create(HEIGHT, WIDTH, CV_8UC3);

        SDL_Flip(screen);
	//The settings of the image capture
	omxcam_video_settings_t	settings;
  
  	//Initialize the settings
	omxcam_video_init(&settings);
  	settings.camera.width = WIDTH;
  	settings.camera.height = HEIGHT;
  	settings.format = OMXCAM_FORMAT_RGB888;
  
  	//Set the buffer callback, this is mandatory
  	settings.on_data = &on_data;

	//image.create(WIDTH, HEIGHT, CV_8UC3);
	detector = FeatureDetector::create("STAR");
  
  	//Start the image streaming
	omxcam_video_start(&settings, OMXCAM_CAPTURE_FOREVER);

        //int quit = 0;
        while (!quit)
        {
                SDL_Event event;
                if (SDL_PollEvent(&event))
                {
                        switch (event.type)
			{
			case SDL_QUIT:
				omxcam_video_stop();
                                quit = 1;
				break;
			case SDL_KEYDOWN:
				if (event.key.which == 'q')
					quit = 1;
				break;
			}
                }
        }
  
	SDL_Quit();
=======
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
>>>>>>> fb00f1b4c5ef512077df1b0ccaac9e5ec80a48d9
	return 0;
}

