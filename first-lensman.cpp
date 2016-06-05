// RaspberryPi

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>	// Kalman filter defined here
#include <SDL/SDL.h>
#include <pthread.h>
#include "omxcam.h"
//#include "I2C.h"
//#include "Quaternion.h"
#include "MPU6050_6Axis_MotionApps20.h"

using namespace cv;

#define WIDTH 640
#define HEIGHT 480
#define SIZE_OF_FRAME (WIDTH * HEIGHT * 3)
#define INTERVAL	10 // msec

static SDL_Surface *screen;
static SDL_Surface *frame;
static int current = 0;

static pthread_mutex_t	mutex;

// OpenCV detect features
static Mat image(HEIGHT, WIDTH, CV_8UC3);
static Ptr<FeatureDetector> detector;
static std::vector<KeyPoint> keypoints;
static KalmanFilter kalman(4, 3, 0);	// measure 3 dimensional position
static Mat_<float> measurement(3, 1); // measurement.setTo(Scalar(0));
static Mat estimated;
//static double x, y, z;

static MPU6050 mpu;
//static MPU_6050 mpu6050;
//static AXDL345  axdl345;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int16_t accel[3];      // [x, y, z]            accel sensor measurements

VectorInt16 accelRaw;		// [x, y, z]			raw accel sensor measurements
VectorInt16 accelReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 accelWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector


inline Uint8 *scanLine(SDL_Surface *surface, int y, int x)
{
	return (Uint8 *)(surface->pixels) + (y * surface->pitch) + (surface->format->BytesPerPixel * x);
}

// Read fifo 
static void readFIFO()
{
	int pkts = 0;

	// Get data from FIFO
	fifoCount = mpu.getFIFOCount();
	if (fifoCount > 900) {
		// Full is 1024, so 900 probably means things have gone bad
		printf("Oops, DMP FIFO has %d bytes, aborting\n", fifoCount);
		exit(1);
	}
	while ((fifoCount = mpu.getFIFOCount()) >= 42) {
		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		pkts++;
	}
	if (pkts > 5)
		printf("Found %d packets, running slowly\n", pkts);
}

static void setup() {
	// initialize device
	printf("Initializing I2C devices...\n");
	mpu.initialize();

	// verify connection
	printf("Testing device connections...\n");
	printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

	// load and configure the DMP
	printf("Initializing DMP...\n");
	devStatus = mpu.dmpInitialize();

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		printf("Enabling DMP...\n");
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		//Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		//attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		printf("DMP ready! Waiting for first interrupt...\n");
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		printf("DMP Initialization failed (code %d)\n", devStatus);
	}

	/*
	adjAccel[0] = adjAccel[1] = adjAccel[2] = 0;
	adjGyro[0] = adjGyro[1] = adjGyro[2] = 0;
	for (int i = 0; i < 20; i++)
	{
		readFIFO();
		mpu.dmpGetAccel(accel, fifoBuffer);
		mpu.dmpGetGyro(gyro, fifoBuffer);
		adjAccel[0] += accel[0];
		adjAccel[1] += accel[1];
		adjAccel[2] += accel[2];
		adjGyro[0] += gyro[0];
		adjGyro[1] += gyro[1];
		adjGyro[2] += gyro[2];
	}
	adjAccel[0] /= 20;
	adjAccel[1] /= 20;
	adjAccel[2] /= 20;
	adjGyro[0] /= 20;
	adjGyro[1] /= 20;
	adjGyro[2] /= 20;
	printf("ADJUST: %d, %d, %d\n", adjAccel[0], adjAccel[1], adjAccel[2]);
	*/

	measurement.setTo(cv::Scalar(0));
	kalman.transitionMatrix =
		*(cv::Mat_<float>(4, 4) <<
		1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1);
	readFIFO();
	mpu.dmpGetAccel(accel, fifoBuffer);
	kalman.statePre.at<float>(0) = accel[0];
	kalman.statePre.at<float>(1) = accel[1];
	kalman.statePre.at<float>(2) = accel[2];
	kalman.statePre.at<float>(3) = 0.0;
	setIdentity(kalman.measurementMatrix);
	setIdentity(kalman.processNoiseCov, cv::Scalar::all(1e-4));
	setIdentity(kalman.measurementNoiseCov, cv::Scalar::all(10));
	setIdentity(kalman.errorCovPost, cv::Scalar::all(.1));
}

void *thread_feature(void *arg)
{
	while (1)
	{
		pthread_mutex_lock(&mutex);
		memcpy(image.ptr(), frame->pixels, SIZE_OF_FRAME);
		pthread_mutex_unlock(&mutex);
		detector->detect(image, keypoints);		

		Mat surface(HEIGHT, WIDTH, CV_8UC3, frame->pixels);
		for (size_t i = 0; i < keypoints.size(); i++)
		{
			Point2f pt = keypoints[i].pt;
			circle(surface, Point(pt.x, pt.y), 3, Scalar(0, 0, 255));
		}
		imshow("Feature", surface);

		//putchar('*');
		printf("%d ", keypoints.size());
		usleep(1000 * 100);	// 100msec
	}
	return NULL;
}

void *thread_sensor(void* arg)
{
	//setup();
	/*
	mpu6050.Init();

	measurement.setTo(Scalar(0));
	kalman.transitionMatrix = 
		*(Mat_<float>(4, 4) << 
		1, 0, 1, 0, 
		0, 1, 0, 1, 
		0, 0, 1, 0, 
		0, 0, 0, 1);
	kalman.statePre.at<float>(0) = mpu6050.accelX();
	kalman.statePre.at<float>(1) = mpu6050.accelY();
	kalman.statePre.at<float>(2) = mpu6050.accelZ();
	kalman.statePre.at<float>(3) = 0.0;
	setIdentity(kalman.measurementMatrix);
	setIdentity(kalman.processNoiseCov, Scalar::all(1e-4));
	setIdentity(kalman.measurementNoiseCov, Scalar::all(10));
	setIdentity(kalman.errorCovPost, Scalar::all(.1));
	*/

	//double x, y, z;
	//double xSpeed = 0, ySpeed = 0, zSpeed = 0;
	//double X = 0, Y = 0, Z = 0;

	Quaternion q;           // [w, x, y, z]         quaternion container

	while (1)
	{
		kalman.predict();

		/*
		x = mpu6050.accelX();
		y = mpu6050.accelY();
		z = mpu6050.accelZ();

		measurement(0) = x;
		measurement(1) = y;
		measurement(2) = z;
		*/

		readFIFO();

		// Calcurate Gravity and Yaw, Pitch, Roll
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&accelRaw, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&accelReal, &accelRaw, &gravity);
		mpu.dmpGetLinearAccelInWorld(&accelWorld, &accelReal, &q);

		measurement(0) = accelWorld.x;
		measurement(1) = accelWorld.y;
		measurement(2) = accelWorld.z;
		estimated = kalman.correct(measurement);
		
		/*
		xSpeed += estimated.at<float>(0) * INTERVAL / 1000; // speed m/s
		ySpeed += estimated.at<float>(1) * INTERVAL / 1000;
		zSpeed += estimated.at<float>(2) * INTERVAL / 1000;
		X += xSpeed * INTERVAL / 1000; // speed * INTERVAL / 1000 * 1000 displacement in mm
		Y += ySpeed * INTERVAL / 1000;
		Z += zSpeed * INTERVAL / 1000;
		Quaternion q(0.0, x, y, z);
		VectorFloat vector[3];
		GetGravity(vector, &q);
		float ypr[3];
		GetYawPitchRoll(ypr, &q, vector);
		//mpu6050.Next();
		*/

		printf("%8.5f, %8.5f, %8.5f\n", 
			estimated.at<float>(0), estimated.at<float>(1), estimated.at<float>(2));
		usleep(1000 * INTERVAL);
	}
	return NULL;
}

// Show result
void show()
{
	SDL_Rect srcRect = {0, 0, WIDTH, HEIGHT};
	SDL_Rect dstRect = {0, 0};
 
	Point center(WIDTH / 2, HEIGHT / 2);
	Point vector(center.x + (int)(estimated.at<float>(0)), center.y + (int)(estimated.at<float>(1)));
	//Point vector(center.x + (int)(x * 100), center.y + (int)(y * 100));

	Mat surface(HEIGHT, WIDTH, CV_8UC3, frame->pixels);
	for (size_t i = 0; i < keypoints.size(); i++)
	{
		Point2f pt = keypoints[i].pt;
		circle(surface, Point(pt.x, pt.y), 3, Scalar(0, 0, 255));
	}
	// Center
	circle(surface, center, 3, Scalar(255, 255, 255));
	line(surface, center, vector, Scalar(255, 255, 255));
	vector = Point(center.x + (int)(accelWorld.x), center.y + (int)(accelWorld.y));
	line(surface, center, vector, Scalar(255, 0, 0));

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

	setup();
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

	pthread_mutex_init(&mutex, NULL);

	pthread_t thread;
	pthread_create(&thread, NULL, thread_sensor, NULL);

        //Start the image streaming
        omxcam_video_start(&settings, 1000 * 30); //OMXCAM_CAPTURE_FOREVER);

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

