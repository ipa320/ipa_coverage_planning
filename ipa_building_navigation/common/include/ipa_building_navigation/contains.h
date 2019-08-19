#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>

#pragma once

//This function searches for the given element in the given vector and returns true if it is in it or false if not
template<typename T>
bool inline contains(std::vector<T> vector, T element)
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
