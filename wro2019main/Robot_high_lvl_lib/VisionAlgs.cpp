#include "VisionAlgs.h"
#include <opencv2/imgproc.hpp>
#include "debug.h"
#include <algorithm>
#if !(defined(WIN32) || defined(_WIN32) || \
    defined(__WIN32) && !defined(__CYGWIN__))



#include <zbar.h>


typedef struct {
	std::string type;
	std::string data;
	std::vector<cv::Point> location;
} decodedObject;

// Find and decode barcodes and QR codes
void decode_zbar(cv::Mat &im, std::vector<decodedObject> &decodedObjects) {


	using namespace zbar;
	// Create zbar scanner
	ImageScanner scanner;

	// Configure scanner
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 0);
	scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
	//	cv::Mat res;
	//	cv::resize(im, res, cv::Size(640, 480));
	//	cv::rotate(res, res, cv::ROTATE_180);
		// Convert image to grayscale
		cv::Mat imGray;
	cvtColor(im, imGray, cv::COLOR_BGR2GRAY);
	save_debug_img("greyQR", imGray);
	// Wrap image data in a zbar image
	Image image(im.cols,
		im.rows,
		"Y800",
		(uchar *)imGray.data,
		im.cols * im.rows);

	// Scan the image for barcodes and QRCodes
	int n = scanner.scan(image);

	// Print results
	for(Image::SymbolIterator symbol = image.symbol_begin() ;
	     symbol != image.symbol_end() ; ++symbol) {
		decodedObject obj;

		obj.type = symbol->get_type_name();
		obj.data = symbol->get_data();

		// Print type and data
		std::cout << "Type : " << obj.type << std::endl;
		std::cout << "Data : " << obj.data << std::endl << std::endl;

		// Obtain location
		for(int i = 0 ; i < symbol->get_location_size() ; i++) {
			obj.location.push_back(
			    cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
		}

		decodedObjects.push_back(obj);
	}
}
std::string qr_detect_frame(cv::Mat qr, QrDetectorTypeEnum qrDetectorType)
{
	
	std::string s;

	//	cv::resize(qr, qr, cv::Size(640, 480));
		if(qrDetectorType == QrDetectorTypeEnum::CV)
	{	
		
		cv::QRCodeDetector qd;
		s = qd.detectAndDecode(qr);
		if (s.length() < 35)
		{
			std::cout << "Can't detect qr code using openCV, not enough symbols!";
		}
	}
	if (qrDetectorType == QrDetectorTypeEnum::ZBAR)
	{
		std::vector<decodedObject> dec_obj;
		decode_zbar(qr, dec_obj);
		if (dec_obj.size() > 0)
		{
			s = dec_obj[0].data;
		}
		else
		{
			std::cout << "Can't detect qr code using ZBAR, not enough symbols!";
		}
	}
//	save_debug_img("qrdetect", qr);
	return s;
}

#endif

std::string color_t2str(color_t color)
{
	const std::string strs[8] = { "un", "blue", "red", "green", "orange", "yel", "black", "white" };
	return strs[(int)color];
}
struct bin_hsv_params {
  std::pair<int, int> hue;
  std::pair<int, int> sut;
  std::pair<int, int> val;
};
color_t GetSquareBin(cv::Mat frame, bin_hsv_params* briks_colors);
bin_hsv_params big_briks_colors[8]{
    {{0, 0}, {0, 0}, {0, 0}},            // undefined_c
    {{83, 135}, {60, 255}, {0, 255}},  // blue_c
    {{156, 185}, {60, 255}, {0, 255}},  // red_c
    {{40, 70}, {60, 255}, {0, 255}},    // green_c
    {{1, 15}, {47, 255}, {0, 255}},     // orange_c
    {{18, 40}, {60, 255}, {0, 255}},    // yellow_c
    {{101, 255}, {0, 61}, {0, 151}},       // black_c
    {{0, 255}, {0, 50}, {125, 255}}     // white_c
};

bin_hsv_params small_briks_colors[8]{
    {{0, 0}, {0, 0}, {0, 0}},           // undefined_c
    {{89, 125}, {60, 255}, {0, 255}},   // blue_c
    {{157, 184}, {60, 255}, {0, 255}},  // red_c
    {{40, 70}, {68, 255}, {0, 230}},    // green_c
    {{1, 15}, {71, 255}, {0, 255}},     // orange_c
    {{18, 40}, {77, 255}, {0, 255}},    // yellow_c
    {{0, 0}, {0, 0}, {0, 0}},            // black_c
    {{0, 0}, {0, 0}, {0, 0}}             // white_c
};

color_t VisionGetSmallBox(const cv::Mat& frame, Robot::CatchCubeSideEnum side) {
  cv::Rect cut_rect;
	if (side == Robot::CatchCubeSideEnum::LEFT) {
    cut_rect = cv::Rect(cv::Point(460, 438), cv::Size(20, 20));
  } else {
          cut_rect = cv::Rect(cv::Point(600, 408), cv::Size(20, 20));
  }
  cv::Mat cut_mat(frame, cut_rect);
  cv::Mat f_with_rect;
  frame.copyTo(f_with_rect);

  color_t color = GetSquareBin(cut_mat, small_briks_colors);
  cv::putText(f_with_rect, color_t2str(color),
              cv::Point(cut_rect.x, cut_rect.y - 10), cv::FONT_HERSHEY_SIMPLEX,
              0.8, 255);
   cv::rectangle(f_with_rect, cut_rect, cv::Scalar(255, 0, 0));
#if !(defined(WIN32) || defined(_WIN32) || \
      defined(__WIN32) && !defined(__CYGWIN__))
  //save_debug_img("SmallBox.jpg", frame);
  save_debug_img("SmallBoxRect.jpg", f_with_rect);
#endif
  return color;
}
color_t VisionGetBigBox(const cv::Mat& frame,double dist) {
  const double dist_coef = 0.0683478260869565;
int x_rect = 510 - (dist * dist_coef);
	//far 355    near 505     off 120  385
	  if(dist < 400)
	{
		x_rect = 746;
	}

  cv::Rect cut_rect(cv::Point(550, x_rect), cv::Size(15, 15));
  cv::Mat cut_mat(frame, cut_rect);
  cv::Mat f_with_rect;
  frame.copyTo(f_with_rect);
	color_t color = GetSquareBin(cut_mat,big_briks_colors);
	cv::putText(f_with_rect,
		color_t2str(color) + std::to_string(dist),
		cv::Point(cut_rect.x, cut_rect.y - 10),
		cv::FONT_HERSHEY_SIMPLEX,
		0.8,
		255);
  cv::rectangle(f_with_rect, cut_rect, cv::Scalar(255, 5, 44));

#if !(defined(WIN32) || defined(_WIN32) || \
      defined(__WIN32) && !defined(__CYGWIN__))
  //save_debug_img("BigBox.jpg", frame);
  save_debug_img("BigBoxRect.jpg", f_with_rect);
#endif
  return color;  // undefined_c;
}

color_t GetSquareBin(cv::Mat frame, bin_hsv_params* briks_colors) {
  const int kMorthSize = 3;
  double area = 0;
  std::vector<int> areas_max;
  color_t color = undefined_c;
  cv::Mat src_hsv, src_hsv_bin, src_canny;
  cv::cvtColor(frame, src_hsv, cv::COLOR_BGR2HSV);
  std::vector < std::vector < cv::Point >> contours, brickContours;
  std::vector<cv::Point> brickContour;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<cv::Vec3f> circles;

  for (int i = 0; i < 8; ++i) {
    int area_max = 0;
  inRange(src_hsv,
            cv::Scalar(briks_colors[i].hue.first, briks_colors[i].sut.first,briks_colors[i].val.first),
            cv::Scalar(briks_colors[i].hue.second, briks_colors[i].sut.second,briks_colors[i].val.second),
            src_hsv_bin);

  cv::erode(src_hsv_bin, src_hsv_bin,
            getStructuringElement(cv::MORPH_ELLIPSE,
								      cv::Size(kMorthSize, kMorthSize)));
  cv::dilate(src_hsv_bin, src_hsv_bin,
             getStructuringElement(cv::MORPH_ELLIPSE,
                                   cv::Size(kMorthSize, kMorthSize)));
  cv::dilate(src_hsv_bin, src_hsv_bin,
             getStructuringElement(cv::MORPH_ELLIPSE,
                                   cv::Size(kMorthSize, kMorthSize)));
  cv::erode(src_hsv_bin, src_hsv_bin,
            getStructuringElement(cv::MORPH_ELLIPSE,
                                  cv::Size(kMorthSize, kMorthSize)));
  cv::findContours(src_hsv_bin, contours, hierarchy, cv::RETR_CCOMP,
               cv::CHAIN_APPROX_NONE);
  for (int j = 0; j < contours.size(); ++j) {
    area = cv::contourArea(contours[j]);
    if (area > area_max) {
      brickContour = contours[j];
      area_max = area;
      break;
    }
  }

	areas_max.push_back(area_max);
  }
  int area_max = 0;
  auto it = std::max_element(areas_max.begin(),areas_max.end());
 
  return (color_t)std::distance(areas_max.begin(), it);
}