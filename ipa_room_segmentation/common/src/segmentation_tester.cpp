#include <ipa_room_segmentation/voronoi_random_field_segmentation.h>


double rosener(const column_vector& m)
{
	const double x = m(0);
	const double y = m(1);

	return 100.0*std::pow(y - x*x, 2.0) + std::pow(1 - x, 2.0);
}


int main()
{
	VoronoiRandomFieldSegmentation segmenter;

	column_vector found_min = segmenter.find_min_value();

	column_vector starting_point(2);

	starting_point = 4,8;

	dlib::find_min_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), rosener, starting_point, -1);


	std::cout << "rosen solution main: \n" << starting_point << " rosen solution class: \n" << found_min << std::endl;

	return 0;
}
