// RaspberryPi

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <SDL/SDL.h>
#include <pthread.h>
#include "omxcam.h"

using namespace cv;

#define WIDTH 640
#define HEIGHT 480
#define SIZE_OF_FRAME (WIDTH * HEIGHT * 3)

static SDL_Surface *screen;
static SDL_Surface *frame;
static int current = 0;

// OpenCV detect features
static Mat image(HEIGHT, WIDTH, CV_8UC3);
static Ptr<FeatureDetector> detector;
static std::vector<KeyPoint> keyPoints;

inline Uint8 *scanLine(SDL_Surface *surface, int y, int x)
{
	return (Uint8 *)(surface->pixels) + (y * surface->pitch) + (surface->format->BytesPerPixel * x);
}

void *thread_feature(void *arg)
{
	printf(".");
	usleep(1000 * 1000);
	return arg;
}

// Show result
void show()
{
	SDL_Rect srcRect = {0, 0, WIDTH, HEIGHT};
	SDL_Rect dstRect = {0, 0};
	SDL_BlitSurface(frame, &srcRect, screen, &dstRect);
	// Copy to Mat
	//memcpy(image.ptr(), frame->pixels, SIZE_OF_FRAME);
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

	pthread_t thread;
	pthread_create(&thread, NULL, thread_feature, NULL);

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

