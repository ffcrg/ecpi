#for cross compiling call "make -f <file> CROSS=1

INC= -I . -I ../src/libmodbus/  -I ../src/wiringPi/wiringPi/   -I ../src/energycam 

ifeq "$(CROSS)" "1"
    CC     = arm-linux-gnueabihf-gcc
    CPP    = arm-linux-gnueabihf-g++
else
    CC     = gcc
    CPP    = g++
endif

all:				ecpi 

		
ecpi: 		 		energycam.o energycammodbus.o wiringPi.o  modbus.o modbus-data.o modbus-rtu.o 
					$(CC) -o ecpi energycam.o energycammodbus.o wiringPi.o modbus.o modbus-data.o modbus-rtu.o 
				
energycam.o:		../src/energycam/energycam.c ../src/energycam/energycampi.h 
					$(CC) $(INC) -c ../src/energycam/energycam.c

energycammodbus.o:	../src/energycam/energycammodbus.c ../src/energycam/energycammodbus.h 
					$(CC) $(INC) -c ../src/energycam/energycammodbus.c
			
wiringPi.o:	    	../src/wiringPi/wiringPi/wiringPi.c 
					$(CC) $(INC) -c ../src/wiringPi/wiringPi/wiringPi.c
				
modbus.o:			../src/libmodbus/modbus.c 
					$(CC) $(INC) -c ../src/libmodbus/modbus.c

modbus-data.o:		../src/libmodbus/modbus-data.c 
					$(CC) $(INC) -c ../src/libmodbus/modbus-data.c
				
modbus-rtu.o:		../src/libmodbus/modbus-rtu.c 
					$(CC) $(INC) -c ../src/libmodbus/modbus-rtu.c
	  
clean: 			
				@rm -f ec energycam.o energycammodbus.o wiringPi.o modbus.o modbus-data.o modbus-rtu.o
				@echo Clean done
