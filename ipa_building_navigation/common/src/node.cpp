#include <ipa_building_navigation/node.h>

const int dir_ = 8;

nodeAstar::nodeAstar(int xp, int yp, int d, int p)
{
	xPos_ = xp;
	yPos_ = yp;
	level_ = d;
	priority_ = p;
}

int nodeAstar::getxPos() const
{
	return xPos_;
}
int nodeAstar::getyPos() const
{
	return yPos_;
}
int nodeAstar::getLevel() const
{
	return level_;
}
int nodeAstar::getPriority() const
{
	return priority_;
}

void nodeAstar::updatePriority(const int& xDest, const int& yDest)
{
	priority_ = level_ + estimate(xDest, yDest) * 10; //A*
}

// give better priority to going strait instead of diagonally
void nodeAstar::nextLevel(const int& i) // i: direction
{
	level_ += (dir_ == 8 ? (i % 2 == 0 ? 10 : 14) : 10);
}

// Estimation function for the remaining distance to the goal.
const int& nodeAstar::estimate(const int& xDest, const int& yDest) const
{
	static int xd, yd, d;
	xd = xDest - xPos_;
	yd = yDest - yPos_;

	// Euclidian Distance
	d = static_cast<int>(sqrt(xd * xd + yd * yd));

	// Manhattan distance
//	d = abs(xd) + abs(yd);

	// Chebyshev distance
//	d=std::max(abs(xd), abs(yd));

	return (d);
}
