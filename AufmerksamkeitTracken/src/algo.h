
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>



#include "blob_gen.h"


namespace ELSE{


static cv::RotatedRect run(cv::Mat input_img){


	float rz_fakk=float(input_img.cols)/384.0;

	cv::Mat pic=cv::Mat::zeros(input_img.rows/rz_fakk, input_img.cols/rz_fakk, CV_8UC1);
	cv::resize(input_img, pic,pic.size());



	

	cv::normalize(pic, pic, 0, 255, cv::NORM_MINMAX, CV_8U);



	double border=0.1;


	cv::RotatedRect ellipse=blob_finder(&pic,border);



	ellipse.size.height=ellipse.size.height*rz_fakk;
	ellipse.size.width=ellipse.size.width*rz_fakk;

	ellipse.center.x=ellipse.center.x*rz_fakk;
	ellipse.center.y=ellipse.center.y*rz_fakk;


	return ellipse;
	


	
}




}



