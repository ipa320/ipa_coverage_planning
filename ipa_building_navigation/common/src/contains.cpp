#include <ipa_building_navigation/contains.h>

bool contains(std::vector<int> vector, int element)
{
	//this functions checks, if the given element is in the given vector (in this case for int elements)
	if (!vector.empty())
	{
		return vector.end() != std::find(vector.begin(), vector.end(), element);
	}
	else
	{
		return false;
	}
}

bool contains(std::vector<cv::Point> vector, cv::Point element)
{
	//this functions checks, if the given element is in the given vector (in this case for int elements)
	if (!vector.empty())
	{
		return vector.end() != std::find(vector.begin(), vector.end(), element);
	}
	else
	{
		return false;
	}
}
