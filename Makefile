APP = first-lensman
CC = gcc
CFLAGS = -Werror -g -Wall -O2 -lc++ -x c++
OPENCV_LIB = -lopencv_core -lopencv_features2d -lopencv_objdetect -lopencv_flann
LDFLAGS = -lstdc++ -L/opt/vc/lib $(OPENCV_LIB) -lopenmaxil -lbcm_host -lvchiq_arm -lpthread \
		-L./$(OMXCAM_HOME)/lib -lomxcam -lSDL -Wl,-rpath=$(OMXCAM_HOME)/lib
INCLUDES = -I/opt/vc/include -I/opt/vc/include/interface/vcos/pthreads \
		-I/opt/vc/include/interface/vmcs_host/linux -I./$(OMXCAM_HOME)/include

SRC := $(SRC) $(APP).cpp
OBJS := $(OBJS) $(APP).o

all: $(APP) $(SRC)

%.o: %.cpp
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@ -Wno-deprecated-declarations

$(APP): $(OBJS)
	$(CC) -o $@ -Wl,--whole-archive $(OBJS) $(LDFLAGS) -Wl,--no-whole-archive -rdynamic

.PHONY: clean rebuild

clean:
	rm -f $(APP) $(APP).o $(CLEAN)

rebuild:
	make -f Makefile-shared clean && make -f Makefile-shared
