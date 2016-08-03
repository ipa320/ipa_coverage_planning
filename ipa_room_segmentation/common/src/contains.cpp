#include <ipa_room_segmentation/contains.h>

bool contains(std::vector<cv::Scalar> vector, cv::Scalar element)
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

bool contains(std::vector<cv::Point> vector, cv::Point element)
{
	//this functions checks, if the given element is in the given vector (in this case for cv::Point elements)
	if (!vector.empty())
	{
		return vector.end() != std::find(vector.begin(), vector.end(), element);
	}
	else
	{
		return false;
	}
}

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

bool contains(std::vector<std::vector<uint> > vector, std::vector<uint> element)
{
	//this functions checks, if the given element is in the given vector (in this case for vector<double>)
	if (!vector.empty())
	{
		return vector.end() != std::find(vector.begin(), vector.end(), element);
	}
	else
	{
		return false;
	}
}
