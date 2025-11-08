# Compiler
CXX = g++
CXXFLAGS = -std=c++17 -Wall -I -O3.

# Find all .cpp files in this directory
SOURCES := $(wildcard *.cpp)
OBJECTS := $(SOURCES:.cpp=.o)

# Output program name
TARGET = raytracer

# Default rule
all: $(TARGET)

# Linking step
$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(TARGET)

# Compilation step
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up build files
clean:
	rm -f $(OBJECTS) $(TARGET)
