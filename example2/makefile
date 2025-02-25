# Compiler and Linker
ARCH = aarch64
CROSS_COMPILE = $(ARCH)-linux-gnu-
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
LD = $(CC)

export C_INCLUDE_PATH=/opt/axis/acapsdk/sysroots/aarch64/usr/include
export LIBRARY_PATH=/opt/axis/acapsdk/sysroots/aarch64/usr/lib
export LD_LIBRARY_PATH=/opt/axis/acapsdk/sysroots/aarch64/usr/lib:/usr/lib/aarch64-linux-gnu

# Directories
INCLUDE_DIRS = -I/usr/include/modbus -I/opt/axis/acapsdk/sysroots/aarch64/usr/include/curl
CFLAGS = -Wall $(INCLUDE_DIRS)
LDFLAGS = -L/opt/axis/acapsdk/sysroots/aarch64/usr/lib -L/usr/lib/aarch64-linux-gnu -lcurl -lmodbus




# Flags
CFLAGS = -Wall $(INCLUDE_DIRS)
LDFLAGS = -lcurl -lmodbus

# Targets and Objects
TARGET = modbus_event_handler
OBJS = modbus_event_handler.o modbus_handler.o

# Build Target
all: $(TARGET)

$(TARGET): $(OBJS)
	$(LD) $(CFLAGS) $(OBJS) -o $@ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) $(OBJS)
