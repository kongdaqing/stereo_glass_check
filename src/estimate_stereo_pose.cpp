#include <iomanip>
#include "estimate_stereo_pose.hpp"
#include "eigen3/Eigen/Dense"
#include "opencv2/core/eigen.hpp"
#include "aslam/cameras/GridCalibrationTargetAprilgrid.hpp"

EstimateStereoPose::EstimateStereoPose() {
    left_map_x_.release();
    left_map_y_.release();
    right_map_x_.release();
    right_map_y_.release();

}

EstimateStereoPose::~EstimateStereoPose() {

}


bool EstimateStereoPose::GenerateRemapMatrix(std::string stereo_yml,int width,int height) {
    cv::FileStorage fs(stereo_yml, cv::FileStorage::FORMAT_YAML);
    if (!fs.isOpened()) {
        printf("Cannot open ./stereo_rectify.yml!\n");
        return false;
    }
    cv::Mat K1, K2, D1, D2, R, T, R1, R2, P1, P2, Q;
    fs["K1"] >> K1;
    fs["K2"] >> K2;
    fs["D1"] >> D1;
    fs["D2"] >> D2;
    fs["R"] >> R;
    fs["T"] >> T;
    if (!K1.data || !K2.data || !D1.data || !D2.data || !R.data || !T.data) {
      printf("Read stereo_rectify.yml failed!\n");
      return false;
    }
    cv::stereoRectify(K1, D1, K2, D2, cv::Size(width, height), R, T, R1, R2, P1, P2, Q);
    K_ = P1.rowRange(0,3).colRange(0,3).clone();
    std::cout << "K : \n" << K_ << std::endl;
    cv::initUndistortRectifyMap(K1, D1, R1, P1, cv::Size(width, height), CV_16SC2, left_map_x_, left_map_y_);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, cv::Size(width, height), CV_16SC2, right_map_x_, right_map_y_);
    return true;
}



void EstimateStereoPose::AnalysisStereoPose(const cv::Mat& left_im_raw,const cv::Mat& right_im_raw) {
    if (left_map_x_.empty() || right_map_x_.empty()) {
        printf("Please load stereo yaml file for remapping!\n");
        return;
    }
    aslam::cameras::GridCalibrationTargetAprilgrid::AprilgridOptions options;
    options.maxSubpixDisplacement2 = 16;
    options.minBorderDistance = 8.0;
    aslam::cameras::GridCalibrationTargetAprilgrid grid(5, 8, 1, 0.3, options);
    cv::Mat left_im_calib,right_im_calib;
    cv::remap(left_im_raw, left_im_calib, left_map_x_, left_map_y_, cv::INTER_LINEAR);
    cv::remap(right_im_raw, right_im_calib, right_map_x_, right_map_y_, cv::INTER_LINEAR);

    Eigen::MatrixXd outPointsLeft, outPointsRight;
    std::vector<bool> validPointsLeft, validPointsRight;
    grid.computeObservation(left_im_calib, outPointsLeft, validPointsLeft);
    grid.computeObservation(right_im_calib, outPointsRight, validPointsRight);
    VecPointD left_points,right_points;
    for (size_t i = 0; i < validPointsLeft.size(); i++)
    {
        if (!validPointsLeft[i] || !validPointsRight[i]) {
            continue;
        }
        cv::Point2d left_point = cv::Point2d{outPointsLeft(i, 0), outPointsLeft(i, 1)};
        cv::Point2d right_point = cv::Point2d{outPointsRight(i, 0), outPointsRight(i, 1)};
        left_points.push_back(left_point);
        right_points.push_back(right_point);
        {
//#define IMSHOW_POINT
#ifdef IMSHOW_POINT
            cv::Mat tmp_left = left_im_calib.clone();
            cv::Mat tmp_right = right_im_calib.clone();
            cv::circle(tmp_left, left_point, 5, 255, 2);
            cv::circle(tmp_right, right_point, 5, 255, 2);
            cv::Mat tmp_all;
            cv::hconcat(tmp_left,tmp_right,tmp_all);
            cv::resize(tmp_all,tmp_all,tmp_all.size()/3);
            cv::imshow("detect_result",tmp_all);
            cv::waitKey(50);
#endif
        }
    }
    if (left_points.size() < 10) {
        printf("Please input apriltag image which can be detect!\n");
    } else {
        Eigen::Vector3d euler_deg =  EstimatePose(left_im_calib,right_im_calib,left_points,right_points);
        CreateErrorItem("x",right_im_calib,cv::Point2i(50,100),400,30,1.0,3.0,euler_deg[0]);
        CreateErrorItem("y",right_im_calib,cv::Point2i(50,200),400,30,1.0,3.0,euler_deg[1]);
        CreateErrorItem("z",right_im_calib,cv::Point2i(50,300),400,30,1.0,3.0,euler_deg[2]);
        cv::imshow("test_rect",right_im_calib);  
        cv::waitKey(0);    
    }
}

Eigen::Vector3d EstimateStereoPose::EstimatePose(cv::Mat& left_im_calib,cv::Mat& right_im_calib,
    const VecPointD& left_points,const VecPointD& right_points) {
    cv::Mat H = cv::findHomography(left_points,right_points,0,1.0);
    std::vector<cv::Mat> Rs, Ts, normals;
    cv::decomposeHomographyMat(H, K_, Rs, Ts, normals);
    for(size_t i = 0; i < Rs.size(); i++) {
        std::cout << i << " -- R:\n" << Rs[i] << std::endl << Ts[i] << std::endl;
    }
    
    Eigen::Matrix3d R;
    cv::cv2eigen(Rs[0],R);
    Eigen::Vector3d euler;
    euler = R.eulerAngles(2,1,0) * 180. * M_1_PI;
    return euler;
}



void EstimateStereoPose::CreateErrorItem(std::string title,cv::Mat& im,cv::Point2i start_pos,int w,int h,float thres,float range,float error) {
    //step0:Create Title
    cv::Point2i title_origin;
    title_origin.x = start_pos.x - 30;
    title_origin.y = start_pos.y + h*2/3;
    cv::putText(im,title,title_origin,cv::FONT_HERSHEY_SIMPLEX,0.85,cv::Scalar(255,255,255),2);
    
    //step1:Create Outside Rectangle
    RectangleItem outside_rec(w,h,start_pos,cv::Scalar(255,255,255));
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