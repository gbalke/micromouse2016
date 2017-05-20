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

		Position (int8_t x_axis, int8_t y_axis):x(x_axis),y(y_axis){}
	};
	
	// Struct for cell properties.
	struct Cell
	{
		bool N:1, 		// North Wall
			 S:1, 		// South Wall
			 E:1, 		// East Wall
			 W:1, 		// West Wall
			 deadEnd:1,	// Set if this cell leads to a dead end.
			 hasEnt:1;	// Set if this cell has been entered.
	};

	Solver (uint8_t max_X, uint8_t max_Y);
	// Car passes info related to current cell here and the function 
	// 	returns the next direction to move:
	DIRECTION update(const Position &pos, const DIRECTION &currDir, const Cell &newCell);

private:
	// Dimensions of the maze and center point.
	uint8_t size_X, size_Y,
			center_X, center_Y;

	// Maze cell storage array.
	Cell * cells;

	// Checks the position passed to see if it is a valid path.
	bool checkCell (Position pos, const DIRECTION dir);
	
	// Finds the direction that heads towards the center.
	DIRECTION findClosest (const Position &pos);

	// Get inverse of the passed direction.
	DIRECTION getOppositeDir (const DIRECTION &dir);

	void setWalls (Cell &toSet, const Cell &data);
};
