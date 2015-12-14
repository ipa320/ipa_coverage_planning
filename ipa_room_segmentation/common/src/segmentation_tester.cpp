#include <ipa_room_segmentation/voronoi_random_field_segmentation.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>


int main()
{
	cv::Mat map = cv::imread("/home/rmb-fj/Pictures/map.png", 0); // office_b.png

	for(unsigned int u = 0; u < map.rows; ++u)
	{
		for(unsigned int v = 0; v < map.cols; ++v)
		{
			if(map.at<unsigned char>(u,v) < 250)
			{
				map.at<unsigned char>(u,v) = 0;
			}
			else
			{
				map.at<unsigned char>(u,v) = 255;
			}
		}
	}

	VoronoiRandomFieldSegmentation segmenter(false);

	segmenter.segmentMap(map, 7, 50, 4, 7, true); // 7, 50, 4, 5

	return 0;
}
