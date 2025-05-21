# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -O2

# Program name
PROG = gmc

# Source files
SRCS = gmc.c cJSON.c
OBJS = $(SRCS:.c=.o)

# Default target
all: $(PROG)

# Compile and link
$(PROG): $(OBJS)
	$(CC) $(OBJS) -o $@

# Compile source files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean build files
clean:
	rm -f $(PROG) $(OBJS)

.PHONY: all clean 