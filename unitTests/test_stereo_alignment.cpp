#include "estimate_stereo_pose.hpp"

int main(int argc,char** argv) {
    std::string stereo_yml = argv[1];
    std::string left_img = argv[2];
    std::string right_img = argv[3];
    EstimateStereoPose est_pose;
    cv::Mat left = cv::imread(left_img,cv::IMREAD_GRAYSCALE);
    cv::Mat right = cv::imread(right_img,cv::IMREAD_GRAYSCALE);
    int width = left.cols;
    int height = left.rows;
    if (est_pose.GenerateRemapMatrix(stereo_yml,width,height)) {
        est_pose.AnalysisStereoPose(left,right);
    }
}
