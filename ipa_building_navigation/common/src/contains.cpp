#include <ipa_building_navigation/contains.h>

bool contains(std::vector<int> vector, int element)
{
	//this functions checks, if the given element is in the given vector (in this case for cv::Sclar elements)
	if (!vector.empty())
	{
		return vector.end() != std::find(vector.begin(), vector.end(), element);
	}
	else
	{
		return false;
	}
}

