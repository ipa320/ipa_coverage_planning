#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <stdio.h>

//This is th object to reperesent a node in a Graph. It is used by A_star_pathplanner.cpp and was applied from:
//		http://code.activestate.com/recipes/577457-a-star-shortest-path-algorithm/
//In the estimate() function the distance from the node to the goal is calculated. Uncomment there which distance calculation
//you want to use (Euclidean, Manhattan or Chebyshev). Euclidean is more precisly but could slow the planner a little bit on
//long paths.

#pragma once //make sure this header gets included only one time when multiple classes need it in the same project
			 //regarding to https://en.wikipedia.org/wiki/Pragma_once this is more efficient than #define

class NodeAstar
{
protected:
	// current position
	int xPos_;
	int yPos_;
	// total distance already travelled to reach the node
	int level_;
	// priority=level+remaining distance estimate
	int priority_; // smaller: higher priority

public:
	NodeAstar(int xp, int yp, int d, int p);
	int getxPos() const;
	int getyPos() const;
	int getLevel() const;
	int getPriority() const;
	void updatePriority(const int& xDest, const int& yDest);
	void nextLevel(const int& i); // i: direction
	const int& estimate(const int& xDest, const int& yDest) const;

};
