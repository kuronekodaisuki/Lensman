// RaspberryPi

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <SDL/SDL.h>
#include <pthread.h>
#include "omxcam.h"
#include "I2C.h"
#include "KalmanFilter/Kalman.h"

using namespace cv;

#define WIDTH 640
#define HEIGHT 480
#define SIZE_OF_FRAME (WIDTH * HEIGHT * 3)

static SDL_Surface *screen;
static SDL_Surface *frame;
static int current = 0;

static pthread_mutex_t	mutex;

// OpenCV detect features
static Mat image(HEIGHT, WIDTH, CV_8UC3);
static Ptr<FeatureDetector> detector;
static std::vector<KeyPoint> keypoints;

static MPU_6050 mpu6050;
static AXDL345  axdl345;

inline Uint8 *scanLine(SDL_Surface *surface, int y, int x)
{
	return (Uint8 *)(surface->pixels) + (y * surface->pitch) + (surface->format->BytesPerPixel * x);
}

void *thread_feature(void *arg)
{
	while (1)
	{
		pthread_mutex_lock(&mutex);
		memcpy(image.ptr(), frame->pixels, SIZE_OF_FRAME);
		pthread_mutex_unlock(&mutex);
		detector->detect(image, keypoints);		

		//putchar('*');
		printf("%d ", keypoints.size());
		usleep(1000 * 100);
	}
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

// Show result
void show()
{
	SDL_Rect srcRect = {0, 0, WIDTH, HEIGHT};
	SDL_Rect dstRect = {0, 0};
 
	Mat surface(HEIGHT, WIDTH, CV_8UC3, frame->pixels);
	for (size_t i = 0; i < keypoints.size(); i++)
	{
		Point2f pt = keypoints[i].pt;
		circle(surface, Point(pt.x, pt.y), 3, Scalar(0, 0, 255));
	}
	//printf("%d ", keypoints.size());

	SDL_BlitSurface(frame, &srcRect, screen, &dstRect);
	SDL_Flip(screen);
}

// Get image date from camera
extern "C" void on_data (omxcam_buffer_t buffer){
        //printf("%d %d ", current, buffer.length);

	memcpy((char *)(frame->pixels) + current, buffer.data, buffer.length);
	current += buffer.length;
	if (SIZE_OF_FRAME <= current)
	{
		show();
		//printf("%d ", current);
		current = 0; 
	}
}

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
	omxcam_video_start(&settings, 1000 * 10); //OMXCAM_CAPTURE_FOREVER);

	pthread_mutex_init(&mutex, NULL);

	pthread_t thread;
	pthread_create(&thread, NULL, thread_sensor, NULL);

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
	return 0;
}

