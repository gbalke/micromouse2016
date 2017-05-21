#include <iostream>
#include <cstdlib>  // atoi

#include "Maze.h"
#include "MazeDefinitions.h"
#include "Solver.h"

/**
 * Demo of a PathFinder implementation.
 *
 * Do not use a left/right wall following algorithm, as most
 * Micromouse mazes are designed for such algorithms to fail.
 */

Solver::DIRECTION convert (Dir d)
{
	switch(d) {
        case NORTH:
            return Solver::NORTH;
        case SOUTH:
            return Solver::SOUTH;
		case EAST:
            return Solver::EAST;
        case WEST:
            return Solver::WEST;
    }
}

class Translation : public PathFinder {
public:
    Translation(bool shouldPause = false) : pause(shouldPause) {
        shouldGoForward = false;
        visitedStart = false;
		finishedUpdate = true;
		solver = new Solver(MazeDefinitions::MAZE_LEN, MazeDefinitions::MAZE_LEN);
        count = 0;
    }

    MouseMovement nextMovement(unsigned x, unsigned y, const Maze &maze) {
 
		if (finishedUpdate)
		{       
            count++;
			Solver::Cell newCell;

			newCell.N = maze.wallToNorth();
			newCell.S = maze.wallToSouth();
			newCell.E = maze.wallToEast();
			newCell.W = maze.wallToWest();
	
			Solver::Position pos(x,y);

			directionToHead = solver->update(pos, convert(maze.getHeading()), newCell);
			finishedUpdate = false;

            std::cout << print() << std::endl << std::endl;

		}

        // Pause at each cell if the user requests it.
        // It allows for better viewing on command line.
        if(pause) {
            std::cout << "Hit enter to continue..." << std::endl;
            std::cin.ignore(10000, '\n');
            std::cin.clear();
        }

        std::cout << maze.draw(5) << std::endl << std::endl;

        // If we somehow miraculously hit the center
        // of the maze, just terminate and celebrate!
        if(isAtCenter(x, y)) {
            std::cout << std::endl << count << std::endl << std::endl;
            std::cout << "Found center! Good enough for the demo, won't try to get back." << std::endl;
            return Finish;
        }

        // If we hit the start of the maze a second time, then
        // we couldn't find the center and never will...
        if(x == 0 && y == 0) {
            if(visitedStart) {
                std::cout << "Unable to find center, giving up." << std::endl;
                return Finish;
            } else {
                visitedStart = true;
            }
        }

		if (convert(maze.getHeading()) == directionToHead)
		{
			shouldGoForward = true;
			finishedUpdate = true;
			return MoveForward;	
		}	
		else
		{
			shouldGoForward = false;
			return TurnClockwise;
		}

        // If we get stuck somehow, just terminate.
        std::cout << "Got stuck..." << std::endl;
        return Finish;
    }

    std::string print()
    {
        std::string info = "";
        for (int i = 0; i < solver->getNumCells(); i++)
        {
            if (!(i % MazeDefinitions::MAZE_LEN))
                info += "\n";
            if (solver->getDead(i))
                info += "x";
            else
                info += "-";
        }
        return info;
    }

protected:
    // Count of number of steps.
    int count;

    // Helps us determine that we should go forward if we have just turned left.
    bool shouldGoForward;

	// Solver object
	Solver* solver;

    // Helps us determine if we've made a loop around the maze without finding the center.
    bool visitedStart;    

	// Direction to keep track of between steps.
	Solver::DIRECTION directionToHead;
	bool finishedUpdate;

    // Indicates we should pause before moving to next cell.
    // Useful for command line usage.
    const bool pause;

    bool isAtCenter(unsigned x, unsigned y) const {
        unsigned midpoint = MazeDefinitions::MAZE_LEN / 2;

        if(MazeDefinitions::MAZE_LEN % 2 != 0) {
            return x == midpoint && y == midpoint;
        }

        return  (x == midpoint     && y == midpoint    ) ||
        (x == midpoint - 1 && y == midpoint    ) ||
        (x == midpoint     && y == midpoint - 1) ||
        (x == midpoint - 1 && y == midpoint - 1);
    }
};

int main(int argc, char * argv[]) {
    MazeDefinitions::MazeEncodingName mazeName = MazeDefinitions::MAZE_ALL_JAPAN_2012;
    bool pause = false;

    // Since Windows does not support getopt directly, we will
    // have to parse the command line arguments ourselves.

    // Skip the program name, start with argument index 1
    for(int i = 1; i < argc; i++) {
        if(strcmp(argv[i], "-m") == 0 && i+1 < argc) {
            int mazeOption = atoi(argv[++i]);
            if(mazeOption < MazeDefinitions::MAZE_NAME_MAX && mazeOption > 0) {
                    mazeName = (MazeDefinitions::MazeEncodingName)mazeOption;
            }
        } else if(strcmp(argv[i], "-p") == 0) {
            pause = true;
        } else {
            std::cout << "Usage: " << argv[0] << " [-m N] [-p]" << std::endl;
            std::cout << "\t-m N will load the maze corresponding to N, or 0 if invalid N or missing option" << std::endl;
            std::cout << "\t-p will wait for a newline in between cell traversals" << std::endl;
            return -1;
        }
    }

    Translation guide(pause);
    Maze maze(mazeName, &guide);
    std::cout << maze.draw(5) << std::endl << std::endl;

    maze.start();
}
