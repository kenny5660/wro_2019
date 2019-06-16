#include "VisionAlgs.h"
#include "debug.h"
box_color_t VisionGetSmallBox(const cv::Mat& frame)
{

	
	#if !(defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__))
	save_debug_img("SmallBox.jpg", frame);
	#endif
	return undefined_bc;
}
