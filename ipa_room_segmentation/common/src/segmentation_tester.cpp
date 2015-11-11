#include <ipa_room_segmentation/voronoi_random_field_segmentation.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>


int main()
{
	cv::Mat map = cv::imread("/home/rmb-fj/git/care-o-bot-indigo/src/autopnp/ipa_room_segmentation/common/files/test_maps/office_e.png", 0);

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

	VoronoiRandomFieldSegmentation segmenter;

	segmenter.segmentMap(map, 17, 50, true);

	return 0;
}
