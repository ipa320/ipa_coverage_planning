#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <stdio.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <fstream>

#include <ipa_building_navigation/contains.h>

class setCoverSolver
{
protected:
	//function to merge groups together, which have at least one node in common
	std::vector<std::vector<int> > mergeGroups(const std::vector<std::vector<int> >& found_groups);

public:
	setCoverSolver();

	std::vector<std::vector<int> > solveSetCover(const std::vector<std::vector<int> >& given_cliques, const int number_of_nodes);
};
