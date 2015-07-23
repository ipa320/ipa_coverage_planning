#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <stdio.h>



class nodeAstar
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
	nodeAstar(int xp, int yp, int d, int p);
	int getxPos() const;
	int getyPos() const;
	int getLevel() const;
	int getPriority() const;
	void updatePriority(const int& xDest, const int& yDest);
	void nextLevel(const int& i); // i: direction
	const int& estimate(const int& xDest, const int& yDest) const;

};
