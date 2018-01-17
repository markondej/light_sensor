CFLAGS += -Wall -fexceptions -pthread -lm -O3 -fpermissive -fno-strict-aliasing
TARGET = light_sensor

CPP=$(CCPREFIX)g++

all:
	$(CPP) $(CFLAGS) -o $(TARGET) main.cpp

clean:
	rm light_sensor
