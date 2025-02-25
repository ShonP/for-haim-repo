PROG1	= opencv_app
OBJS1	= example.cpp

PROGS	= $(PROG1)

PKGS = gio-2.0 gio-unix-2.0 vdostream libmodbus

CFLAGS += -Os -pipe
CXXFLAGS += -Os -pipe -std=c++11

CFLAGS += $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) pkg-config --cflags $(PKGS))
CXXFLAGS += $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) pkg-config --cflags $(PKGS))
LDLIBS += $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) pkg-config --libs $(PKGS))

# Add OpenCV flags and libs
CXXFLAGS += -I$(SDKTARGETSYSROOT)/usr/include/opencv4
LDFLAGS = -L./lib -Wl,--no-as-needed,-rpath,'$$ORIGIN/lib'
LDLIBS += -lm -lopencv_video -lopencv_imgproc -lopencv_core -lpthread -lmodbus

CFLAGS += -W -Wall

all: $(PROGS)

$(PROG1): $(OBJS1)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

clean:
	rm -f $(PROGS) *.o *~ 