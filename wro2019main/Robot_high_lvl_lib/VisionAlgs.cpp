#include "VisionAlgs.h"
#include "debug.h"
box_color_t VisionGetSmallBox(const cv::Mat& frame)
{
	save_debug_img("SmallBox.jpg", frame);
	return undefined_bc;
}
