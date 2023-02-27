#pragma once
#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Dense"
using namespace Eigen;

typedef std::vector<Vector2d> VecPointD;




class EstimateStereoPose
{
private:
    /* data */
public:
    EstimateStereoPose();
    ~EstimateStereoPose();

    void EstimatePose(cv::Mat& left_im_calib,cv::Mat& right_im_calib,
            const VecPointD& left_points,const VecPointD& right_points);
};

