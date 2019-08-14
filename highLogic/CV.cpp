//
// Created by Danila on 06.05.2019.
//

#include "CV.h"
#include <iostream>
#include "settings.h"
#include "debug.h"
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
#else
    #include <zbar.h>
#endif

typedef struct {
	std::string type;
	std::string data;
	std::vector<cv::Point> location;
} decodedObject;

// Find and decode barcodes and QR codes
void decode_zbar(cv::Mat &im, std::vector<decodedObject> &decodedObjects) {
    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
    return;
    #else
	using namespace zbar;
	// Create zbar scanner
	ImageScanner scanner;

	// Configure scanner
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 0);
	scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
	cv::Mat res;
	cv::resize(im, res, cv::Size(640, 480));
	cv::rotate(res, res, cv::ROTATE_180);
	// Convert image to grayscale
	cv::Mat imGray;
	cvtColor(res, imGray, cv::COLOR_BGR2GRAY);
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
    #endif
}

inline double letter2coordinat(const char a) {
    return (a - 'A') * field_sett::size_field_unit;
}

inline Point letter2coordinat(const char a, const char b) {
    return Point{letter2coordinat(a), letter2coordinat(b)};
}

Point (*rot2point[4])(Point &p) = {
    [](Point &p) {
      return p;
    },
    [](Point &p) {
      return Point{field_sett::max_field_height - p.get_y(), p.get_x()};
    },
    [](Point &p) {
      return Point{field_sett::max_field_width - p.get_x(), field_sett::max_field_height - p.get_y()};
    },
    [](Point &p) {
      return Point{p.get_y(), field_sett::max_field_width - p.get_x()};
    }
};
std::string qr_detect_frame(cv::Mat qr)
{
	
	std::string s;

//	cv::resize(qr, qr, cv::Size(640, 480));
	if (kQrDetectorType == QrDetectorTypeEnum::CV)
	{
		cv::QRCodeDetector qd;
		s = qd.detectAndDecode(qr);
		if (s.length() < 35)
		{
			throw std::runtime_error("Can't detect qr code using openCV, not enough symbols!");
		}
	}
	if (kQrDetectorType == QrDetectorTypeEnum::ZBAR)
	{
		std::vector<decodedObject> dec_obj;
		decode_zbar(qr, dec_obj);
		if (dec_obj.size() > 0)
		{
			s = dec_obj[0].data;
		}
		else
		{
			throw std::runtime_error("Can't detect qr code using Zbar!");
		}
	}
	save_debug_img("qrdetect", qr);
	return s;
}

RobotPoint qr_detect(cv::Mat qr, std::array<BoxMap, 3> &boxes_pos, std::pair<Point, Point> &pz, std::string start_s) {
    std::string s = start_s;
    if (s == "") {
	    
        s= qr_detect_frame(qr);
	    
    }
    Point pz_p1 = letter2coordinat(s[1], s[3]);
    Point pz_p2 = letter2coordinat(s[5], s[7]);
    int rot = 0;
    if (fabs(pz_p1.get_y() - pz_p2.get_y()) > fabs(pz_p1.get_x() - pz_p2.get_x())) {
	    if (pz_p1.get_y() < pz_p2.get_y()) {
            rot = 3;
        } else {
            rot = 1;
        }
	} else if (pz_p1.get_x() > pz_p2.get_x()) {
        rot = 2;
    }
    pz_p1 = rot2point[rot](pz_p1);
    pz_p2 = rot2point[rot](pz_p2);
    double d1 = pz_p1.get_y() - pz_p2.get_y();
    double d2 = pz_p1.get_x() - pz_p2.get_x();
    double ang = atan2(d1, d2);
    if (pz_p1.get_x() > pz_p2.get_x()) {
        ang *= -1;
    }
    double x_offset = field_sett::parking_zone_door_size * sin(M_PI - ang);
    double y_offset = field_sett::parking_zone_door_size * cos(M_PI - ang);
    pz = std::make_pair(pz_p1, Point{pz_p1.get_x() + x_offset, pz_p1.get_y() + y_offset});
    Point offset_box_rot[4] = {{0, 0}, {-field_sett::climate_box_width, 0}, {-field_sett::climate_box_width, -field_sett::climate_box_height}, {0, -field_sett::climate_box_width}};
    for (int i = 0; i < 3; i++) {
        Point p = letter2coordinat(std::min(s[10 + i * 9], s[14 + i * 9]), std::min(s[12 + i * 9], s[16 + i * 9]));
        boxes_pos[i] = BoxMap(rot2point[rot](p) + offset_box_rot[rot]);
    }
    Point robot_pos = pz.first + Point{field_sett::parking_zone_door_size * sin(ang) / 2., -field_sett::parking_zone_door_size * cos(ang) / 2.};
	return { robot_pos.get_x(), robot_pos.get_y(), (fabs(ang) < 0.02) ? (0) : (ang - M_PI) };
}
