# ===== Compiler =====
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 \
	-Iinclude \
	-Iinclude/simulation \
	-Iinclude/rendering \
	-Iinclude/glad \
	-Iinclude/GLFW

# ===== Libraries =====
LDFLAGS = \
	-Llibraries/GLFW/lib \
	-lglfw3 \
	-lopengl32 \
	-lgdi32 \
	-luser32

# ===== Files =====
SRC := $(wildcard src/*.cpp src/simulation/*.cpp src/rendering/*.cpp libraries/glad/src/*.c)
OBJ := $(SRC:%.cpp=build/%.o)
OBJ := $(OBJ:%.c=build/%.o)

TARGET = main.exe

# ===== Rules =====
all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $^ -o $@ $(LDFLAGS)

build/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

build/%.o: %.c
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf build $(TARGET)

.PHONY: all clean
