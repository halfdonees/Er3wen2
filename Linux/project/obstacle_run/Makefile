###############################################################
#
# Purpose: Makefile for "demo"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = obstacle_run
PRO = main.cpp
LIB = `pkg-config --cflags opencv` `pkg-config --libs opencv`
OFLAGS += -lm -ggdb


INCLUDE_DIRS = -I../../include -I../../../Framework/include 


CXX = g++ -std=c++98
CXXFLAGS += -O2 -DLINUX -Wall $(INCLUDE_DIRS) $(LIB)
#CXXFLAGS += -O2 -DDEBUG -DLINUX -Wall $(INCLUDE_DIRS)
LFLAGS += -lpthread -ljpeg -lrt 

OBJECTS = VisionMode.o StatusCheck.o NearPixel.o main.o $(OBJECTS2)

all: $(TARGET)


$(OBJECTS2): $(PRO)
	@$(CXX) -c $(PRO) $(OFLAGS) $(LIB)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

libclean:
	make -C ../../build clean

distclean: clean libclean

darwin.a:
	make -C ../../build

$(TARGET): darwin.a $(OBJECTS)
	$(CXX) $(CFLAGS) $(OBJECTS) ../../lib/darwin.a -o $(TARGET) $(OFLAGS) $(LFLAGS)  $(LIB)
	chmod 755 $(TARGET)

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/DARwIn_demo_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
