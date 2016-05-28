APP = first-lensman
CC = g++
CFLAGS = -Werror -g -Wall -O2 -lc++ -x c++
OPENCV_LIB = -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_ml -lopencv_objdetect -lopencv_ocl -lopencv_video

LDFLAGS = -lstdc++ -L/usr/lib/aarch64-linux-gnu/ $(OPENCV_LIB) -lpthread  \
		 
INCLUDES = -I/opt/vc/include -I/opt/vc/include/interface/vcos/pthreads \
		-I/opt/vc/include/interface/vmcs_host/linux -I../KalmanFilter -I../

SRC := $(SRC) $(APP).cpp ../I2C.cpp ../KalmanFilter/Kalman.cpp
OBJS := $(OBJS) $(APP).o I2C.o Kalman.o

all: $(APP) $(SRC)

%.o: %.cpp
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@ -Wno-deprecated-declarations

$(APP): $(OBJS)
	$(CC) -o $@  $(OBJS) $(LDFLAGS) -rdynamic

.PHONY: clean rebuild

clean:
	rm -f $(APP) $(APP).o $(CLEAN)

rebuild:
	make -f Makefile-shared clean && make -f Makefile-shared
