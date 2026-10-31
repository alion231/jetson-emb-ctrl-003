# Target executable name
TARGET = robotest

# Compiler and optimization flags
CC = gcc

GTK4_CFLAGS = $(shell pkg-config --cflags gtk4)
GTK4_LIBS = $(shell pkg-config --libs gtk4)

CFLAGS = -Wall -Wextra -O3 $(GTK4_CFLAGS)
LIBS = -lgpiod -lpthread -lm $(GTK4_LIBS)

# Automatically find all .c files in the current directory
SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

# Default rule to build the application
all: $(TARGET)

# Link object files into the final executable
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LIBS)

# Compile .c files to .o files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up build artifacts
clean:
	rm -f *.o $(TARGET)

.PHONY: all clean
