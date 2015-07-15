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
