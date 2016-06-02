# You need to "sudo apt-get install libgtkmm-3.0-dev" to build the demo_3d binary

all: test

HDRS = helper_3dmath.h I2Cdev.h MPU6050_6Axis_MotionApps20.h MPU6050.h
CMN_OBJS = I2Cdev.o MPU6050.o
D3D_OBJS = test.o

# Set DMP FIFO rate to 20Hz to avoid overflows on 3d demo.  See comments in
# MPU6050_6Axis_MotionApps20.h for details.

CXXFLAGS = -DDMP_FIFO_RATE=9 -Wall -g -O2 `pkg-config gtkmm-3.0 --cflags --libs`
GTKFLAGS = `pkg-config gtkglextmm-1.2 --cflags --libs`

$(CMN_OBJS) $(DMP_OBJS) $(RAW_OBJS) : $(HDRS)

test: $(D3D_OBJS) $(CMN_OBJS)
	$(CXX) -o $@ $^ -lm `pkg-config gtkmm-3.0 --cflags --libs`

gtk: GTK_sample.cpp
	$(CXX) $(CXXFLAGS) $(GTKFLAGS) -DOFFLINE_TEST -o test_3d GTK_sample.cpp

clean:
	rm -f $(CMN_OBJS) $(D3D_OBJS)  test

