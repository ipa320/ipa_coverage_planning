#include <ipa_room_segmentation/voronoi_random_field_segmentation.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>


int main()
{
	cv::Mat map = cv::imread("/home/rmb-fj/Pictures/map.png", 0);

	VoronoiRandomFieldSegmentation segmenter;

	segmenter.segmentMap(map, 17, 50, true);

	return 0;
}
