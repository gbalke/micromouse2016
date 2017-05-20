#include "Solver.h"
#include <cmath>

Solver::Solver (uint8_t max_X, uint8_t max_Y)
{
	// Allocate memory for grid.
	cells = new Cell[max_X*max_Y]();
	size_X = max_X;
	size_Y = max_Y;
	center_X = size_X / 2;
	center_Y = size_Y / 2;
}

// How this works:
// First the function checks if all directions are walls or dead ends. If so, make
// 	cell dead end and head towards the only non-dead end.
// If not a dead end,  
Solver::DIRECTION Solver::update (const Position &pos, const DIRECTION &currDir, const Cell &newCell)
{
	// Add new cell.
	if (!cells[pos.x + (pos.y * size_X)].hasEnt)	
	{
		setWalls(cells[pos.x + (pos.y * size_X)], newCell);
		cells[pos.x + (pos.y * size_X)].hasEnt = true;
	}

	// Get the direction towards the center.	
	DIRECTION origClosest = findClosest (pos);
	DIRECTION closest = origClosest,
			  oppCurrDir = getOppositeDir(currDir);

	// Check if possible to move in that direction
	Position toCheck = pos;

	// Check if all sides are walls and/or dead ends.
	if (((newCell.N | !checkCell(pos, NORTH)) + 
		 (newCell.S | !checkCell(pos, SOUTH)) + 
		 (newCell.E | !checkCell(pos, EAST)) + 
		 (newCell.W | !checkCell(pos, WEST))) >= 3)
	{
		// If so, set Cell to deadEnd and return direction that is not dead end or wall.
		cells[pos.x + (pos.y * size_X)].deadEnd = true;
		if (!newCell.N | checkCell(pos, NORTH))
			return NORTH;
		if (!newCell.S | checkCell(pos, SOUTH))
			return SOUTH;
		if (!newCell.E | checkCell(pos, EAST))
			return EAST;
		if (!newCell.W | checkCell(pos, WEST))
			return WEST;	
	}
	else
	{
		// Change position to the cell to check.
		uint8_t numChecked = 0;
		// This loop will determine the correct direction to head next.
		while (true)
		{	
			switch (closest)
			{
				case NORTH:
					// Check for wall or if the path is the path we came down.
					if (newCell.N || (closest == oppCurrDir) || !checkCell(toCheck, closest))
					{ 
						// If this is the second check, the opposite of the last direction is the correct dir.
						if (numChecked == 2)
							closest = getOppositeDir(closest);
						else
							closest = findClosest(Position(pos.x, size_X + 1)); 
						break;
					}
					return closest;		
				case SOUTH:
					toCheck.y -= 1;
					if (newCell.S || (closest == oppCurrDir) || !checkCell(toCheck, closest))
					{ 
						// If this is the second check, the opposite of the last direction is the correct dir.
						if (numChecked == 2)
							return getOppositeDir(closest);
						else
							closest = findClosest(Position(pos.x, -1));
						break;
					}
					return closest;
				case EAST:	
					// Check for wall
					if (newCell.E || (closest == oppCurrDir) || !checkCell(toCheck, closest))
					{ 
						// If this is the second check, check opposite direction of last situation.
						if (numChecked == 2)
							closest = getOppositeDir(closest);
						else
							closest = findClosest(Position(size_Y + 1, pos.y)); 
						break;
					}
					return closest;
				case WEST:				
					// Check for wall
					if (newCell.W || (closest == oppCurrDir) || !checkCell(toCheck, closest))
					{ 
						// If this is the second check, check opposite direction of last situation.
						if (numChecked == 2)
							closest = getOppositeDir(closest);
						else		
							closest = findClosest(Position(-1, pos.y)); 
						break;
					}
					return closest;	
			}

			if(numChecked == 2)
				return closest;

			// Reset toCheck position.
			toCheck = pos;
				
			// Increment number of checked walls.
			numChecked++;
		}	
	}
}

bool Solver::checkCell (Position pos, const DIRECTION dir)
{
	// Change position to the cell next in the given direction.
	switch (dir)
	{
		case NORTH:
			pos.y += 1;
			break;
		case SOUTH:
			pos.y -= 1;
			break;
		case EAST:
			pos.x += 1;			
			break;
		case WEST:
			pos.x -= 1;
			break;			
	}

	// If the cell is a dead end then do not enter.
	if (cells[pos.x + (pos.y * size_X)].deadEnd)
		return false;
	// If the cell has been entered and is not a dead end then it's still OK to enter.
	// (Might be a junction)
	// If the cell has not been entered then it's free to be checked.
	return true;
}

Solver::DIRECTION Solver::findClosest (const Position &pos)
{
	int8_t x_dif = center_X - pos.x;
	int8_t y_dif = center_Y = pos.y;

	// X axis has the closer distance (East/West)
	if (std::abs(x_dif) < std::abs(y_dif))
	{
		if (x_dif < 0)
			return WEST;
		else
			return EAST;
	}
	// Y axis has the closer distance (North/South)
	else if (std::abs(x_dif) > std::abs(y_dif))
	{
		if (y_dif < 0)
			return SOUTH;
		else
			return NORTH;
	}
	// Equidistant (choose along the diagonal)
	// This solution is hard coded to choose direction along the x axis
	//  in this scenario.
	else
	{
		if (y_dif < 0)
			return SOUTH;
		else
			return NORTH;
	}
}

Solver::DIRECTION Solver::getOppositeDir (const DIRECTION &dir)
{
	switch (dir)
	{
		case NORTH:
			return SOUTH;
		case SOUTH:
			return NORTH;
		case EAST:
			return WEST;
		case WEST:
			return EAST;
	}
}

void Solver::setWalls (Cell &toSet, const Cell & data)
{
	toSet.N = data.N;
	toSet.S = data.S;
	toSet.E = data.E;
	toSet.W = data.W;
}
