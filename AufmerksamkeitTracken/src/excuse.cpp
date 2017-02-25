// excuse.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//

#include "stdafx.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>            
#include <fstream>
#include <time.h>


#include "algo.h"






cv::Mat convert_rgb(cv::Mat in){


	int hh=in.rows;
	int ww=in.cols;
	float std=sqrt((float)(in.rows+in.cols));
	hh=hh/std;
	ww=ww/std;

	if(ww%2==0)
		ww++;
	
	if(hh%2==0)
		hh++;

	//GaussianBlur(in, in, cv::Size(ww,hh), std );

	cv::Mat out=cv::Mat::zeros(in.size(),CV_32FC1);
	in.convertTo(in,CV_32FC1);


	for(int i=0;i<in.rows;i++){
		for(int j=0;j<in.cols;j++){
			float blue=in.at<cv::Vec3f>(i,j)[0];
			float green=in.at<cv::Vec3f>(i,j)[1];
			float red=in.at<cv::Vec3f>(i,j)[2];





			float gray_val= (0.299*red) + (0.587*green) + (0.114*blue);
			//float gray_val= (0.333*red) + (0.333*green) + (0.333*blue);

			
			float val_red=abs(gray_val-red);
			float val_green=abs(gray_val-green);
			float val_blue=abs(gray_val-red);



			//if(230<=gray_val)
				//gray_val=10;


			float maxi=1;

			if(val_red>val_blue && val_red>val_green)
				maxi = val_red;

			
			if(val_green>val_blue && val_green>val_red)
				maxi = val_green;

			
			if(val_blue>val_green && val_blue>val_red)
				maxi = val_blue;


			gray_val=gray_val/sqrt(1+maxi);

			//val=val*val;

			//val=1.0/(1.0+val);

			out.at<float>(i,j)=gray_val;



		}
	}


	
	//GaussianBlur(out, out, cv::Size(ww,hh), std );


	cv::normalize(out, out, 0, 255, cv::NORM_MINMAX, CV_32FC1);
	out.convertTo(out,CV_8UC1);
	return out;
}








int _tmain(int argc, _TCHAR* argv[])
{

	
	//std::string name_out = "else_nn.txt";
	std::string name_out = "else_bi.txt";
	

	
	std::string path_data="H:\\THOMAS_BILDER_REMOTE\\bilder\\";
	std::string path_data_out="H:\\THOMAS_BILDER_REMOTE\\bilder_out\\";
	
	

	int lauf=224;





	






	//for(int kk=0;kk<=lauf;kk++){
	for(int kk=0;kk<=lauf;kk++){

		
		
	
	std::string path_image_l = path_data+"Eye_"+ std::to_string((unsigned long long)kk) +"_L.png";
	std::string path_image_r = path_data+"Eye_"+ std::to_string((unsigned long long)kk) +"_R.png";

	//std::cout<<path_image<<std::endl;
	cv::Mat ml = cv::imread(path_image_l, cv::IMREAD_COLOR);
	cv::Mat mr = cv::imread(path_image_r, cv::IMREAD_COLOR);
	

	bool doit=true;

	if(!ml.data){ 
		std::cout<<"error img read"<<std::endl;
		doit=false;
	}
	if(!mr.data){ 
		std::cout<<"error img read"<<std::endl;
		doit=false;
	}









	if(doit){

	cv::Mat mlc = convert_rgb(ml);
	cv::Mat mrc = convert_rgb(mr);



	

	
	//imshow("left conv", mlc);
	//imshow("right conv", mrc);



	cv::RotatedRect posl=ELSE::run(mlc);
	
	cv::RotatedRect posr=ELSE::run(mrc);

	cv::ellipse(ml,posl,cv::Scalar(255,255,255,255));
	cv::ellipse(mr,posr,cv::Scalar(255,255,255,255));
	

	//imshow("left", ml);
	//imshow("right", mr);

	//cv::waitKey(100000);

	
	std::string path_image_lo = path_data_out+"Eye_"+ std::to_string((unsigned long long)kk) +"_L.png";
	std::string path_image_ro = path_data_out+"Eye_"+ std::to_string((unsigned long long)kk) +"_R.png";

	cv::imwrite(path_image_lo,ml);
	cv::imwrite(path_image_ro,mr);

	}

	}



	std::cout<<"ENDE"<<std::endl;
	int tt;
	std::cin>>tt;

    return 0;
}

