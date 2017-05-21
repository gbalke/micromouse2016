#include "Solver.h"
#include <cmath>
#include <stack>
#include <iostream>

Solver::Solver (uint8_t max_X, uint8_t max_Y) : size_X(max_X), size_Y(max_Y)
{
	// Allocate memory for grid.
    numCells = max_X*max_Y;
	cells = new Cell[numCells];
	size_X = max_X;
	size_Y = max_Y;
	lowerLeft.x = upperRight.x = max_X/2;
	lowerLeft.y = upperRight.y = max_Y/2;

	lowerLeft.y--;
	lowerLeft.x--; 

	// Calculate Manhattan distances.
	for (int row = 0; row < max_Y; row++)
	{
		for (int col = 0; col < max_X; col++)
		{
			int x_dif_lower = std::abs(lowerLeft.x - col),
				x_dif_upper = std::abs(upperRight.x - col);

			int y_dif_lower = std::abs(lowerLeft.y - row),
				y_dif_upper = std::abs(upperRight.y - row);

			cells[col + (row * size_X)].dist = x_dif_upper + y_dif_upper;
												// ((x_dif_lower<x_dif_upper)?x_dif_lower:x_dif_upper) +
			   									//	((y_dif_lower<y_dif_upper)?y_dif_lower:y_dif_upper);
			cells[col + (row * size_X)].pos.x = col;
			cells[col + (row * size_X)].pos.y = row;
		}
	}
}

// How this works:
// First the function checks if all directions are walls or dead ends. If so, make
// 	cell dead end and head towards the only non-dead end.
// If not a dead end,  
Solver::DIRECTION Solver::update (const Position &pos, const DIRECTION &currDir, const Cell &newCell)
{
    // Drive straight until a wall is hit or entered a previously entered cell.
	if (!cells[pos.x + (pos.y * size_X)].ent)	
	{
        // Add new cell.
		setWalls(cells[pos.x + (pos.y * size_X)], newCell);
		cells[pos.x + (pos.y * size_X)].ent = true; 
	}

	DIRECTION dir_smallest = findSmallest(pos);

	// Conditions for running flood fill algorithm:
	// 	1) Must move to a higher value cell.
    if (findCell(pos, dir_smallest).dist > cells[pos.x + (pos.y * size_X)].dist)
    {
		// Run flood fill algorithm.
		floodFill(pos);

		// Find the new smallest distance.
		dir_smallest = findSmallest(pos);		
	}
	
	return dir_smallest;
}

/*
while stack is not empty:
	cur = top of the stack
	if cur.distance == 0, continue //don’t want to process the end goal
		min_distance = infinity //placeholder/”invalid” distance
		for each neighboring cell of cur:
			if no wall between cur and neighbor:
				if neighbor.distance < min_distance:
					min_distance = neighbor.distance
			if min_distance == infinity: //something went wrong. gtfo while we can
				continue
			if cur.distance > min_distance: //everything is fine, move on
				Continue
			// if cur.distance <= min_distance we reach this point
			cur.distance = min_distance + 1 //new minimum distance push every neighbor onto stack
*/
void Solver::floodFill (const Position &pos)
{
	std::stack<Cell> cellStack;
	cellStack.push(cells[pos.x + (pos.y * size_X)]);	

	Cell cur;

	while (!cellStack.empty())
	{	
		// Pop item off stack to modify.
		cur = cellStack.top();
		cellStack.pop();
		if (cur.dist == 0)
			continue;
		int min_distance = 255;	
		Cell neighbor;
		// Check all neighbors for walls.
		if (!cur.n && cur.pos.y < 15)
		{
			neighbor = findCell(cur.pos, NORTH);
			if (neighbor.dist < min_distance)
			{
				min_distance = neighbor.dist;
			}
		}
		if (!cur.s && cur.pos.y > 0)
		{
			neighbor = findCell(cur.pos, SOUTH);
			if (neighbor.dist < min_distance)
			{
				min_distance = neighbor.dist;
			}		
		}
		if (!cur.e && cur.pos.x < 15)
		{
			neighbor = findCell(cur.pos, EAST);
			if (neighbor.dist < min_distance)
			{
				min_distance = neighbor.dist;
			}		
		}
		if (!cur.w && cur.pos.x > 0)
		{
			neighbor = findCell(cur.pos, WEST);	
			if (neighbor.dist < min_distance)
			{
				min_distance = neighbor.dist;	
			}		
		}

		std::cout << min_distance << std::endl;

		// Check for error or success
		if (min_distance == 255)	// Problems happened and we don't want to talk about them.
			continue;
		if (cur.dist == (min_distance + 1))
			continue;

		// Push neighbors.
		if (!cur.n && cur.pos.y < 15)
		{
			neighbor = findCell(cur.pos, NORTH);
			cellStack.push(neighbor);
		}
		if (!cur.s && cur.pos.y > 0)
		{
			neighbor = findCell(cur.pos, SOUTH);
			cellStack.push(neighbor);		
		}
		if (!cur.e && cur.pos.x < 15)
		{
			neighbor = findCell(cur.pos, EAST);
			cellStack.push(neighbor);		
		}
		if (!cur.w && cur.pos.x > 0)
		{
			neighbor = findCell(cur.pos, WEST);
			cellStack.push(neighbor);			
		}	

		cells[cur.pos.x + (cur.pos.y * size_X)].dist = min_distance + 1;	
	}
}

// Find direction without wall with smallest manhattan distance.
Solver::DIRECTION Solver::findSmallest (const Position &pos)
{
	DIRECTION dir;

	// This array contains the value of the tiles adjacent to this one.
	int * values = new int[4];
	if(!cells[pos.x + (pos.y * size_X)].n)
		values[0] = findCell(pos, NORTH).dist;
	else
		values[0] = 255;
	if(!cells[pos.x + (pos.y * size_X)].s)
		values[1] = findCell(pos, SOUTH).dist;
	else
		values[1] = 255;
	if(!cells[pos.x + (pos.y * size_X)].e)
		values[2] = findCell(pos, EAST).dist;
	else
		values[2] = 255;
	if(!cells[pos.x + (pos.y * size_X)].w)
		values[3] = findCell(pos, WEST).dist;
	else
		values[3] = 255;

	int smallest_dist = 0;
	for (int i = 1; i < 4; i++)
	{
		if (values[smallest_dist] > values[i])
			smallest_dist = i;
	}

	dir = DIRECTION(smallest_dist);

	delete values;

	return dir;
}

Solver::Cell Solver::findCell (Position pos, const DIRECTION dir)
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

	return cells[pos.x + (pos.y * size_X)];
}

void Solver::setWalls (Cell &toSet, const Cell & data)
{
	toSet.n = data.n;
	toSet.s = data.s;
	toSet.e = data.e;
	toSet.w = data.w;
}

int Solver::getNumCells()
{
    return numCells;
}

uint8_t Solver::getDist(int pos)
{
    int x = pos%size_X,
        y = size_Y - pos/size_X-1;

    return cells[x + (y * size_X)].dist;
}
