TARGET = main
LIBS = -lm -pthread
CPP = g++-7
CPPFLAGS = -g -std=c++17 -Wall 


.PHONY: default all clean

default: $(TARGET)
all: default

OBJECTS = $(patsubst %.cpp, %.o, $(wildcard *.cpp))
HEADERS = $(wildcard *.h )

%.o: %.c $(HEADERS) 
	$(CPP) $(CPPFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(CPP) $(OBJECTS) -Wall $(LIBS) -o $@

clean:
	-rm -f *.o
	-rm -f $(TARGET)
