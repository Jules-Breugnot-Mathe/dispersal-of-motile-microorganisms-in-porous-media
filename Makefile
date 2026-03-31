# ===== OS Detection =====
ifeq ($(OS),Windows_NT)
    PLATFORM = WINDOWS
else
    PLATFORM = LINUX
endif

# ===== Compilers =====
CXX = g++
CC = gcc

# ===== Flags =====
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 \
    -Iinclude \
    -Iinclude/simulation \
    -Iinclude/rendering \
    -Iinclude/glad

# ===== Sources =====
SRC_CPP := $(wildcard src/simulation/*.cpp src/rendering/*.cpp)
SRC_C := Libraries/glad/src/glad.c

# ===== Objects =====
OBJ := $(SRC_CPP:%.cpp=build/%.o)
OBJ += $(SRC_C:%.c=build/%.o)

# ===== Output =====
ifeq ($(PLATFORM),WINDOWS)
    TARGET = main.exe
else
    TARGET = main
endif

# ===== Platform-specific libs =====
ifeq ($(PLATFORM),WINDOWS)
    LDFLAGS = \
        -LLibraries/glfw/lib \
        -lglfw3 \
        -lopengl32 \
        -lgdi32 \
        -luser32
else
    LDFLAGS = \
        -lglfw \
        -lGL \
        -ldl \
        -lX11 \
        -lpthread
endif

# ===== Rules =====
all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(OBJ) -o $@ $(LDFLAGS)

# ===== Compile C++ =====
build/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# ===== Compile C (GLAD) =====
build/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) -Iinclude -c $< -o $@
# ===== Clean =====
clean:
	rm -rf build $(TARGET)

.PHONY: all clean
