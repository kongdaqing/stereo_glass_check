#include <iomanip>
#include "estimate_stereo_pose.hpp"
#include "eigen3/Eigen/Dense"

EstimateStereoPose::EstimateStereoPose(/* args */) {

}

EstimateStereoPose::~EstimateStereoPose() {

}

void EstimateStereoPose::EstimatePose(cv::Mat& left_im_calib,cv::Mat& right_im_calib,
    const VecPointD& left_points,const VecPointD& right_points) {
    
    
}



void EstimateStereoPose::CreateErrorItem(std::string title,cv::Mat& im,cv::Point2i start_pos,int w,int h,float thres,float range,float error) {
    //step0:Create Title
    cv::Point2i title_origin;
    title_origin.x = start_pos.x - 30;
    title_origin.y = start_pos.y + h*2/3;
    cv::putText(im,title,title_origin,cv::FONT_HERSHEY_SIMPLEX,0.85,cv::Scalar(0,0,0),2);
    
    //step1:Create Outside Rectangle
    RectangleItem outside_rec(w,h,start_pos,cv::Scalar(0,0,0),2,false);
    outside_rec.CreateRectangle(im);
    cv::Point2i mid_start_pos = outside_rec.GetMiddlePosition();

    //step2:Create Error Rectangle
    float limit_error = error;
    if (fabs(limit_error) > range) {
        limit_error = limit_error > 0. ? range : -range;
    }
    float width_ratio = 0.5 * w / range;
    int sub_height = 0.8 * h;
    int sub_width = width_ratio * fabs(limit_error);
    printf("width_ratio: %f, sub_width : %d,sub_height: %d\n",width_ratio,sub_width,sub_height);
    cv::Point2i sub_start_pos;
    sub_start_pos.y = mid_start_pos.y + 0.1 * h;
    if (limit_error >= 0) {
        sub_start_pos.x = mid_start_pos.x - sub_width;
    } else {
        sub_start_pos.x = mid_start_pos.x;
    }
    cv::Scalar sub_color(0,255,0);
    if (abs(error) > thres) {
        sub_color = cv::Scalar(0,0,255);
    }
    RectangleItem sub_rec(sub_width,sub_height,sub_start_pos,sub_color);
    sub_rec.CreateRectangle(im);

    //step3: Lines 
    int line_start_y = mid_start_pos.y - h;
    int line_end_y = mid_start_pos.y +  2 * h;
    cv::Point2i start_line(mid_start_pos.x,line_start_y);
    cv::Point2i end_line(mid_start_pos.x,line_end_y);
    cv::line(im,start_line,end_line,cv::Scalar(0,0,0),2);
    
    int line_thres_start_x1 = mid_start_pos.x - thres * width_ratio;
    cv::Point2i thres_line_start1(line_thres_start_x1,mid_start_pos.y);
    cv::Point2i thres_line_end1(line_thres_start_x1,mid_start_pos.y + h);
    cv::line(im,thres_line_start1,thres_line_end1,cv::Scalar(0,255,0),2);

    int line_thres_start_x2 = mid_start_pos.x + thres * width_ratio;
    cv::Point2i thres_line_start2(line_thres_start_x2,mid_start_pos.y);
    cv::Point2i thres_line_end2(line_thres_start_x2,mid_start_pos.y + h);
    cv::line(im,thres_line_start2,thres_line_end2,cv::Scalar(0,255,0),2);   

    //step4: Note
    std::stringstream ss1,ss2;
    ss1 << std::fixed << std::setprecision(1) << thres;
    std::string thres_str1 = ss1.str();
    ss2 << std::fixed << std::setprecision(1) << -thres;
    std::string thres_str2 = ss2.str();
    cv::Point2i thres_txt_start1(line_thres_start_x1 - 15,mid_start_pos.y + 2 * h);
    cv::putText(im,thres_str1,thres_txt_start1,cv::FONT_HERSHEY_SIMPLEX,0.55,cv::Scalar(0,0,0),2);
    cv::Point2i thres_txt_start2(line_thres_start_x2 - 20,mid_start_pos.y + 2 * h);
    cv::putText(im,thres_str2,thres_txt_start2,cv::FONT_HERSHEY_SIMPLEX,0.55,cv::Scalar(0,0,0),2);

}

void EstimateStereoPose::TestDisplayRectangle() {
    cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    CreateErrorItem("x",image,cv::Point2i(50,100),400,30,3.0,10.0,-5.0);
    CreateErrorItem("y",image,cv::Point2i(50,200),400,30,3.0,10.0,2.0);
    CreateErrorItem("z",image,cv::Point2i(50,300),400,30,3.0,10.0,12.0);

    cv::imshow("test_rect",image);

    cv::waitKey(0);
}

void EstimateStereoPose::DisplayError(cv::Mat& img,std::vector<Vector3d>& angle) {
    //RectangleItem outside_rec(40,400,cv::Point2i(20,20));

}