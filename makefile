EXECUTABLE = gpio_service
FLAGS = -Wall -O3 -std=c++11

all: service.o mailbox.o
	g++ -L/opt/vc/lib -lbcm_host -o $(EXECUTABLE) service.o mailbox.o

mailbox.o: mailbox.c mailbox.h
	g++ $(FLAGS) -c mailbox.c
	
service.o: service.cpp
	g++ $(FLAGS) -I/opt/vc/include -c service.cpp

clean:
	rm *.o
