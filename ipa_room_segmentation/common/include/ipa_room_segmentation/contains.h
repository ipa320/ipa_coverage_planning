#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <list>
#include <vector>
#include <set>
#include <math.h>

template<typename T>
bool contains(const std::vector<T>& vector, const T& element)
{
	//this functions checks, if the given element is in the given vector
	if (!vector.empty())
	{
		return vector.end() != std::find(vector.begin(), vector.end(), element);
	}
	else
	{
		return false;
	}
}
