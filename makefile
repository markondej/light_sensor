EXECUTABLE = gpio_service
FLAGS = -Wall -O3 -std=c++11

all: gpio_service.o mailbox.o
	g++ -L/opt/vc/lib -o $(EXECUTABLE) gpio_service.o mailbox.o -lpthread -lbcm_host

mailbox.o: mailbox.c mailbox.h
	g++ $(FLAGS) -c mailbox.c
	
gpio_service.o: gpio_service.cpp
	g++ $(FLAGS) -I/opt/vc/include -c gpio_service.cpp

clean:
	rm *.o
