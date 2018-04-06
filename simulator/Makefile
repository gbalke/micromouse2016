CC=g++
CFLAGS=-g --std=c++11 -Wall -pedantic
DEPS = Maze.h Solver.h
OBJ = Maze.o Solver.o main.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

main.out: $(OBJ)
	g++ -o $@ $^ $(CFLAGS)

clean:
	rm -f *.o
