#include "Solver.h"
#include <cmath>

Solver::Solver (uint8_t max_X, uint8_t max_Y)
{
	// Allocate memory for grid.
    numCells = max_X*max_Y;
	cells = new Cell[numCells]();
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

    // Get the direction towards the center.	
    DIRECTION origClosest = findClosest(pos, currDir);
    DIRECTION closest = origClosest,
        oppCurrDir = getOppositeDir(currDir);

    // Update number of times this cell has been visited (kill if 3)
    if (5 == cells[pos.x + (pos.y * size_X)].visits++)
    {
        cells[pos.x + (pos.y * size_X)].deadEnd = true;
        cells[pos.x + (pos.y * size_X)].timedOut = true;
        return oppCurrDir;
    }

    // Drive straight until a wall is hit or entered a previously entered cell.
	if (!cells[pos.x + (pos.y * size_X)].hasEnt)	
	{
        // Add new cell.
		setWalls(cells[pos.x + (pos.y * size_X)], newCell);
		cells[pos.x + (pos.y * size_X)].hasEnt = true;
        switch (currDir)
        {
            case NORTH:
                if (!newCell.N)
                    return NORTH;
                break;
            case SOUTH:
                if (!newCell.S)
                    return SOUTH;
                break;
            case EAST:
                if (!newCell.E)
                    return EAST;
                break;
            case WEST:
                if(!newCell.W)
                    return WEST;
                break;
        }
	}

	// Check if all sides are walls and/or dead ends.
    if ((((newCell.N || !checkCell(pos, NORTH, CHECK_DEAD_END)) +
        (newCell.S || !checkCell(pos, SOUTH, CHECK_DEAD_END)) +
        (newCell.E || !checkCell(pos, EAST, CHECK_DEAD_END)) +
        (newCell.W || !checkCell(pos, WEST, CHECK_DEAD_END))) >= 3) && !((pos.x == 0) && (pos.y == 0)))
    {
        // If the cell is claimed as a dead end using a cell that was killed, keep track of that (the cell may be part of the solution).
        if (!(checkCell(pos, NORTH, CHECK_TIMED_OUT) +
            checkCell(pos, SOUTH, CHECK_TIMED_OUT) +
            checkCell(pos, EAST, CHECK_TIMED_OUT) +
            checkCell(pos, WEST, CHECK_TIMED_OUT) >= 4))
            cells[pos.x + (pos.y * size_X)].timedOut = true;
        // If so, set Cell to deadEnd and return direction that is not dead end or wall.
        cells[pos.x + (pos.y * size_X)].deadEnd = true;
        return findOpen(pos);
    }
	else
	{
		// Change position to the cell to check.
		uint8_t numChecked = 0;

		// This loop will determine the correct direction to head next.
        bool finished = false;
		while (!finished)
		{	
			switch (closest)
			{
				case NORTH:
					// Check for wall or if the path is the path we came down.
					if (newCell.N
                        || (closest == oppCurrDir) 
                        || !checkCell(pos, closest, CHECK_DEAD_END)  
                        //|| cells[pos.x + (pos.y * size_X)].hasEnt
                        )
					{ 
						// If this is the second check, the opposite of the last direction is the correct dir.
                        if (numChecked == 1)
                            closest = getOppositeDir(closest);
						else if (numChecked == 2)
                            closest = findOpen(pos);
						else
							closest = findClosest(Position(pos.x, size_X + 1), currDir); 
						break;
					}
					finished = true;
                    break;
				case SOUTH:
					if (newCell.S 
                        || (closest == oppCurrDir)
                        || !checkCell(pos, closest, CHECK_DEAD_END)
                        //|| cells[pos.x + (pos.y * size_X)].hasEnt
                        )
					{ 
						// If this is the second check, the opposite of the last direction is the correct dir.
                        if (numChecked == 1)
                            closest = getOppositeDir(closest);
                        else if (numChecked == 2)
                            closest = findOpen(pos);
						else
							closest = findClosest(Position(pos.x, -1), currDir);
						break;
					}
                    finished = true;
                    break;
				case EAST:	
					// Check for wall
					if (newCell.E
                        || (closest == oppCurrDir) 
                        || !checkCell(pos, closest, CHECK_DEAD_END) 
                        //|| cells[pos.x + (pos.y * size_X)].hasEnt
                        )
					{ 
						// If this is the second check, check opposite direction of last situation.
                        if (numChecked == 1)
                            closest = getOppositeDir(closest);
                        else if (numChecked == 2)
                            closest = findOpen(pos);
						else
							closest = findClosest(Position(size_Y + 1, pos.y), currDir); 
						break;
					}
                    finished = true;
                    break;
				case WEST:				
					// Check for wall
					if (newCell.W 
                        || (closest == oppCurrDir) 
                        || !checkCell(pos, closest, CHECK_DEAD_END) 
                        || cells[pos.x + (pos.y * size_X)].hasEnt
                        )
					{ 
						// If this is the second check, check opposite direction of last situation.
                        if (numChecked == 1)
                            closest = getOppositeDir(closest);
                        else if (numChecked == 2)
                            closest = findOpen(pos);
						else		
							closest = findClosest(Position(-1, pos.y), currDir); 
						break;
					}
                    finished = true;
                    break;
			}
            
            if (numChecked == 2)
            {
                finished = true;
            }
				
			// Increment number of checked walls.
			numChecked++;
		}	

        // Check if the chosen closest is a path already traveled; 
        //      if so, take a different path if one exists.
        if (checkCell(pos, NORTH, CHECK_HAS_ENTERED) && !newCell.N && checkCell(pos, NORTH, CHECK_DEAD_END))
        {
            closest = NORTH;
        }
        if (checkCell(pos, SOUTH, CHECK_HAS_ENTERED) && !newCell.S && checkCell(pos, SOUTH, CHECK_DEAD_END))
        {
            closest = SOUTH;
        }
        if (checkCell(pos, EAST, CHECK_HAS_ENTERED) && !newCell.E && checkCell(pos, EAST, CHECK_DEAD_END))
        {
            closest = EAST;
        }
        if (checkCell(pos, WEST, CHECK_HAS_ENTERED) && !newCell.W && checkCell(pos, WEST, CHECK_DEAD_END))
        {
            closest = WEST;
        }

        return closest;
	}
}

bool Solver::checkCell (Position pos, const DIRECTION dir, const CHECKS type)
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
	if (cells[pos.x + (pos.y * size_X)].deadEnd && type == CHECK_DEAD_END)
		return false;
    // If checking if the cell has already been entered return false.
    if (cells[pos.x + (pos.y * size_X)].hasEnt && type == CHECK_HAS_ENTERED)
        return false;
    // If checking timeout, return false if the cell has been killed.
    if (cells[pos.x + (pos.y * size_X)].hasEnt && type == CHECK_TIMED_OUT)
        return false;
	// If the cell has been entered and is not a dead end then it's still OK to enter.
	// (Might be a junction)
	// If the cell has not been entered then it's free to be checked.
	return true;
}

Solver::DIRECTION Solver::findClosest (const Position &pos, const DIRECTION dir)
{
	int8_t x_dif = center_X - pos.x;
	int8_t y_dif = center_Y - pos.y;

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
        if (dir == EAST || dir == WEST)
		    if (y_dif < 0)
			    return SOUTH;
		    else
			    return NORTH;
        if (dir == NORTH || dir == SOUTH)
            if (x_dif < 0)
                return EAST;
            else
                return WEST;
	}
}

Solver::DIRECTION Solver::findOpen(Position pos)
{
    if (!cells[pos.x + (pos.y * size_X)].N && checkCell(pos, NORTH, CHECK_DEAD_END))
        return NORTH;
    if (!cells[pos.x + (pos.y * size_X)].S && checkCell(pos, SOUTH, CHECK_DEAD_END))
        return SOUTH;
    if (!cells[pos.x + (pos.y * size_X)].E && checkCell(pos, EAST, CHECK_DEAD_END))
        return EAST;
    if (!cells[pos.x + (pos.y * size_X)].W && checkCell(pos, WEST, CHECK_DEAD_END))
        return WEST;
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

int Solver::getNumCells()
{
    return numCells;
}

bool Solver::getDead(int pos)
{
    int x = pos%size_X,
        y = size_Y - pos/size_X;

    return cells[x + (y * size_X)].deadEnd;
}