CC=gcc
CFLAGS=-Wall -Wextra
LIBS=-lSDL2
CFILES=main.c cpu.c ppu.c apu.c
BINARY=gp

all: $(BINARY)

$(BINARY): $(CFILES)
	$(CC) $(CFLAGS) $(CFILES) $(LIBS) -o $(BINARY)

clean:
	rm $(BINARY)
