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

    enum CHECKS {
        CHECK_DEAD_END = 0,
        CHECK_HAS_ENTERED,
        CHECK_TIMED_OUT
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
             hasEnt:1,	// Set if this cell has been entered.
             timedOut:1;// Timed out means the cell was killed after too many visits.
        uint8_t visits;
        Cell()
        {
            deadEnd = 0;
            hasEnt = 0;
            timedOut = 0;
            visits = 0;
        }
	};

	Solver (uint8_t max_X, uint8_t max_Y);
	// Car passes info related to current cell here and the function 
	// 	returns the next direction to move:
	DIRECTION update(const Position &pos, const DIRECTION &currDir, const Cell &newCell);

    int getNumCells();

    bool Solver::getDead(int pos);

private:
	// Dimensions of the maze and center point.
	uint8_t size_X, size_Y,
			center_X, center_Y;
    int numCells;

	// Maze cell storage array.
	Cell * cells;

	// Checks the position passed to see if it is a valid path.
	bool checkCell (Position pos, const DIRECTION dir, const CHECKS type);
	
	// Finds the direction that heads towards the center.
	DIRECTION findClosest (const Position &pos, const DIRECTION dir);

	// Get inverse of the passed direction.
	DIRECTION getOppositeDir (const DIRECTION &dir);

    // Find an open wall.
    DIRECTION findOpen(Position pos);

	void setWalls (Cell &toSet, const Cell &data);
};
