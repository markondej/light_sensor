EXECUTABLE = gpio_control
FLAGS = -Wall -O3

all: main.o mailbox.o
	g++ -L/opt/vc/lib -lpthread -lbcm_host -o $(EXECUTABLE) main.o mailbox.o

mailbox.o: mailbox.c mailbox.h
	g++ $(FLAGS) -c mailbox.c
	
main.o: main.cpp
	g++ $(FLAGS) -I/opt/vc/include -c main.cpp

clean:
	rm *.o