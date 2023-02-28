#pragma once
#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Dense"
using namespace Eigen;

typedef std::vector<cv::Point2f> VecPointD;




class EstimateStereoPose
{
private:
    struct RectangleItem {
        RectangleItem(int w,int h,cv::Point2i start_position,cv::Scalar color,int edge_width = 2,bool filled = true) : 
            width_(w), 
            height_(h), 
            start_position_(start_position),
            color_(color),
            edge_width_(edge_width),
            filled_(filled) {
            end_position_.x = start_position_.x + width_;
            end_position_.y = start_position_.y + height_;
        }
        void CreateRectangle(cv::Mat& img) {
            if (img.channels() == 1) {
                cv::cvtColor(img,img,cv::COLOR_GRAY2RGB);
            }
            if (!filled_) 
                cv::rectangle(img,start_position_,end_position_,color_,edge_width_);
            else 
                cv::rectangle(img,start_position_,end_position_,color_,cv::FILLED);
        }
        cv::Point2i GetMiddlePosition() {
            cv::Point2i mid_position;
            mid_position.x = (start_position_.x + end_position_.x) / 2;
            mid_position.y = start_position_.y;
            return mid_position;
        }
        int width_;
        int height_;
        int edge_width_;
        cv::Point2i start_position_;
        cv::Point2i end_position_;
        cv::Scalar color_;
        bool filled_;
    };

    cv::Mat left_map_x_,left_map_y_;
    cv::Mat right_map_x_,right_map_y_;
    cv::Mat K_;
    void CreateErrorItem(std::string title,cv::Mat& im,cv::Point2i start_pos,int w,int h,float range,float thres,float error,bool black_background = true);
    /* data */
public:
    EstimateStereoPose();
    ~EstimateStereoPose();

    void AnalysisStereoPose(const cv::Mat& left_im_raw,const cv::Mat& right_im_raw);

    Eigen::Vector3d EstimatePose(cv::Mat& left_im_calib,cv::Mat& right_im_calib,
            const VecPointD& left_points,const VecPointD& right_points);

    void DisplayError(cv::Mat& img,std::vector<Vector3d>& angle);

    void TestDisplayRectangle();

    bool GenerateRemapMatrix(std::string stereo_yml,int width,int height);

    
};

