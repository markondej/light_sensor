CFLAGS += -Wall -fexceptions -pthread -lm -O3 -fpermissive -fno-strict-aliasing
TARGET = light_sensor

CPP=$(CCPREFIX)g++

all: main.o
	$(CPP) $(CFLAGS) -o $(TARGET) main.o

main.o: main.cpp
	$(CPP) $(CFLAGS) -c main.cpp

clean:
	rm *.o
