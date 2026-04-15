CXX = g++
CXXFLAGS = -O2 -std=c++17 -Wall -Wextra -pedantic
LDFLAGS = -lcurl

TARGET = rds_control
SRC = src/rds_control.cpp

all:
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC) $(LDFLAGS)

clean:
	rm -f $(TARGET)

install:
	cp $(TARGET) /usr/local/bin/

.PHONY: all clean install
