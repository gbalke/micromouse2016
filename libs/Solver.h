// File: maze.h
// Purpose: This class contains the maze data structure and cell struct.
// 		The functions of the maze class will solve the maze.

#pragma once

#include <stdint.h>

class Solver
{
public:
	enum DIRECTION {
		NORTH = 0,
		SOUTH,
		EAST,
		WEST
	};
	
	// Common data struct for position.
	struct Position
	{
		int8_t x, y;
	};
	
	// Struct for cell properties.
	struct Cell
	{
		bool n:1, 		// North Wall
			 s:1, 		// South Wall
			 e:1, 		// East Wall
			 w:1, 		// West Wall
			 ent:1;		// Discovered?

		Position pos;		

		// Manhattan Distance 
	    uint8_t dist;
        Cell()
        {
			ent = false;
            dist = 0;
			n = s = e = w = false;
        }
	};

	Solver (uint8_t max_X, uint8_t max_Y);

	// Car passes info related to current cell here and the function 
	// 	returns the next direction to move:
	DIRECTION update (const Position &pos, const DIRECTION &currDir, const Cell &newCell);

    int getNumCells();

    uint8_t getDist (int pos);

private:
	// Dimensions of the maze and center point.
	uint8_t size_X, size_Y;

	// Upper and lower bounds of the center box.
	Position lowerLeft, upperRight;
	
	// The number of walls in the maze.	
	int numCells;

	// Maze cell storage array.
	Cell * cells;

	// Finds the direction that heads towards the center.
	DIRECTION findSmallest (const Position &pos);

	// Set the walls of 'toSet' to the walls in 'data'.
	void setWalls (Cell &toSet, const Cell &data);

	// FLood fill algorithm function.
	void floodFill (const Position &pos);

	// Returns the cell in the given direction.
	Cell findCell (Position pos, const DIRECTION dir);
};
