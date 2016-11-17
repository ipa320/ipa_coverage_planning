#include <Eigen/Dense>

#include <iostream>
#include <list>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>

#include <opencv/cv.h>

class Neuron
{
protected:

	// vector that stores the direct neighbors of this neuron
	std::vector<Neuron*> neighbors_;

	// vector that stores the weights to the neighbors
	std::vector<float> weights_;

	// position of the neuron
	cv::Point position_;

	// booleans to check if this neuron is cleaned or an obstacle
	bool cleaned_, obstacle_;

	// state (activity) of this neuron
	float state_;

	// parameters used to update the state
	float A_, B_, D_, E_, mu_;

	// function to generate the external input
	float I()
	{
		if(obstacle_ == true)
			return -1.0*E_;
		else if(cleaned_ == false)
			return E_;
		else
			return 0;
	}

public:

	// constructor
	Neuron(float A, float B, float D, float E, float mu, bool obstacle, bool cleaned=false)
	{
		state_ = 0;
		A_ = A;
		B_ = B;
		D_ = D;
		E_ = E;
		mu_ = mu;
		obstacle_ = obstacle;
		cleaned_ = cleaned;
	}

	// function to insert a neighbor
	void addNeighbor(Neuron* new_neighbor)
	{
		// save pointer to neighbor
		neighbors_.push_back(new_neighbor);

		// calculate distance and corresponding weight to it
		cv::Point difference = position_ - new_neighbor->position_;
		float distance = cv::norm(difference);
		weights_.push_back(mu_/distance);
	}

	// function to get the position of the neuron
	cv::Point getPosition()
	{
		return position_;
	}

	// function to get the state of the neuron
	float getState()
	{
		return state_;
	}

	// function to update the state of the neuron
	void updateState()
	{
		// TODO: finish
	}
};
