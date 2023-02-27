#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MvCameraControl.h"
#include "opencv2/opencv.hpp"
#include "aslam/cameras/GridCalibrationTargetAprilgrid.hpp"

#define CAMERA_NUM 2
#define IMG_WIDTH 2448
#define IMG_HEIGHT 2048
#define IMG_SIZE (IMG_WIDTH * IMG_HEIGHT)

uint8_t g_leftImageBuffer[IMG_SIZE];
bool is_left_new = false;
uint8_t g_rightImageBuffer[IMG_SIZE];
bool is_right_new = false;
std::mutex left_mtx, right_mtx;

bool g_bExit = false;


bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo) {
  if (nullptr == pstMVDevInfo) {
    printf("The Pointer of pstMVDevInfo is nullptr!\n");
    return false;
  }
  if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
    printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
    printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
  } else {
    printf("Not support.\n");
  }

  return true;
}

static void *WorkThread(void *pUser) {
  int nRet = MV_OK;

  MVCC_STRINGVALUE stStringValue = {0};
  char camSerialNumber[256] = {0};
  nRet = MV_CC_GetStringValue(pUser, "DeviceSerialNumber", &stStringValue);
  if (MV_OK == nRet) {
    memcpy(camSerialNumber, stStringValue.chCurValue, sizeof(stStringValue.chCurValue));
  } else {
    printf("Get DeviceUserID Failed! nRet = [%x]\n", nRet);
  }

  // ch:获取数据包大小 | en:Get payload size
  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
    return nullptr;
  }

  MV_FRAME_OUT_INFO_EX stImageInfo = {0};
  memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
  auto *pData = (unsigned char *) malloc(stParam.nCurValue);
  if (nullptr == pData) {
    return nullptr;
  }
  unsigned int nDataSize = stParam.nCurValue;

  while (true) {
    if (g_bExit) {
      break;
    }

    nRet = MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 1000);
    if (nRet == MV_OK) {
      printf("Cam[%s]: Got frame[%6d], size[%dx%d], time[%12.6f], data[%d]\n",
             camSerialNumber, stImageInfo.nFrameNum, stImageInfo.nWidth, stImageInfo.nHeight,
             5e-9 * ((uint64_t) stImageInfo.nDevTimeStampHigh * 0x100000000 + stImageInfo.nDevTimeStampLow),
             pData[0]);
      if (std::string(camSerialNumber) == "K40261838") {
        left_mtx.lock();
        memcpy(g_leftImageBuffer, pData, nDataSize);
        is_left_new = true;
        left_mtx.unlock();
      } else if (std::string(camSerialNumber) == "K40261849") {
        right_mtx.lock();
        is_right_new = true;
        memcpy(g_rightImageBuffer, pData, nDataSize);
        right_mtx.unlock();
      }

    } else {
      printf("cam[%s]:Get One Frame failed![%x]\n", camSerialNumber, nRet);
    }
  }

  return nullptr;
}

int main() {
  cv::FileStorage fs("./stereo_rectify.yml", cv::FileStorage::FORMAT_YAML);
  if (!fs.isOpened()) {
    printf("Cannot open ./stereo_rectify.yml!\n");
    return -1;
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
    return -1;
  }

  cv::Mat remapLX, remapLY, remapRX, remapRY;

  cv::stereoRectify(K1, D1, K2, D2, cv::Size(IMG_WIDTH, IMG_HEIGHT), R, T, R1, R2, P1, P2, Q);
  cv::initUndistortRectifyMap(K1, D1, R1, P1, cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_16SC2, remapLX, remapLY);
  cv::initUndistortRectifyMap(K2, D2, R2, P2, cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_16SC2, remapRX, remapRY);


  int nRet = MV_OK;

  void *handle[CAMERA_NUM] = {};

  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  // 枚举设备
  // enum device
  nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet) {
    printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
    return -1;
  }
  unsigned int nIndex = 0;
  if (stDeviceList.nDeviceNum > 0) {
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
      printf("[device %d]:\n", i);
      MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
      if (nullptr == pDeviceInfo) {
        break;
      }
      PrintDeviceInfo(pDeviceInfo);
    }
  } else {
    printf("Find No Devices!\n");
    return -1;
  }

  if (stDeviceList.nDeviceNum < CAMERA_NUM) {
    printf("only have %d camera\n", stDeviceList.nDeviceNum);
    return -1;
  }

  // 提示为多相机测试
  // Tips for multicamera testing
  printf("Start %d camera Grabbing Image test\n", CAMERA_NUM);

  for (int i = 0; i < CAMERA_NUM; i++) {
    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&handle[i], stDeviceList.pDeviceInfo[i]);
    if (MV_OK != nRet) {
      printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
      MV_CC_DestroyHandle(handle[i]);
      return -1;
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(handle[i]);
    if (MV_OK != nRet) {
      printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
      MV_CC_DestroyHandle(handle[i]);
      return -1;
    }
  }

  for (int i = 0; i < CAMERA_NUM; i++) {
    MV_CC_SetEnumValue(handle[i], "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
    MV_CC_SetEnumValue(handle[i], "TriggerMode", MV_TRIGGER_MODE_OFF);
    MV_CC_SetFloatValue(handle[i], "AcquisitionFrameRate", 1.f);
    MV_CC_SetEnumValue(handle[i], "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF); //MV_EXPOSURE_AUTO_MODE_CONTINUOUS
    MV_CC_SetFloatValue(handle[i], "ExposureTime", 39000);
//    MV_CC_SetIntValue(handle[i], "AutoExposureTimeupperLimit", 39000);
    MV_CC_SetEnumValue(handle[i], "GainAuto", MV_GAIN_MODE_OFF); //MV_GAIN_MODE_CONTINUOUS
    MV_CC_SetFloatValue(handle[i], "Gain", 24.f);
//    MV_CC_SetFloatValue(handle[i], "AutoGainupperLimit", 12.f);
//    MV_CC_SetIntValue(handle[i], "Brightness", 100);

    // 开始取流
    // start grab image
    nRet = MV_CC_StartGrabbing(handle[i]);
    if (MV_OK != nRet) {
      printf("Cam[%d]: MV_CC_StartGrabbing fail! nRet [%x]\n", i, nRet);
      return -1;
    }

    pthread_t nThreadID;
    nRet = pthread_create(&nThreadID, nullptr, WorkThread, handle[i]);
    if (nRet != 0) {
      printf("Cam[%d]: thread create failed.ret = %d\n", i, nRet);
      return -1;
    }
  }

  aslam::cameras::GridCalibrationTargetAprilgrid::AprilgridOptions options;
  options.maxSubpixDisplacement2 = 16;
  options.minBorderDistance = 8.0;
  aslam::cameras::GridCalibrationTargetAprilgrid grid(5, 8, 1, 0.3, options);


  cv::namedWindow("left", cv::WINDOW_NORMAL);
  cv::namedWindow("right", cv::WINDOW_NORMAL);
  auto *left_img_data = (uint8_t *) malloc(IMG_SIZE);
  auto *right_img_data = (uint8_t *) malloc(IMG_SIZE);
  cv::Mat left_im_calib, right_im_calib;


  while (true) {
    left_mtx.lock();
    if (is_left_new) {
      memcpy(left_img_data, g_leftImageBuffer, IMG_SIZE);
      is_left_new = false;
    }
    left_mtx.unlock();
    right_mtx.lock();
    if (is_right_new) {
      memcpy(right_img_data, g_rightImageBuffer, IMG_SIZE);
      is_right_new = false;
    }
    right_mtx.unlock();
    cv::Mat left_im(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC1, left_img_data);
    cv::Mat right_im(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC1, right_img_data);
    cv::remap(left_im, left_im_calib, remapLX, remapLY, cv::INTER_LINEAR);
    cv::remap(right_im, right_im_calib, remapRX, remapRY, cv::INTER_LINEAR);

    Eigen::MatrixXd outPointsLeft, outPointsRight;
    std::vector<bool> validPointsLeft, validPointsRight;
    grid.computeObservation(left_im_calib, outPointsLeft, validPointsLeft);
    grid.computeObservation(right_im_calib, outPointsRight, validPointsRight);
    for (int i = 0; i < validPointsLeft.size(); i++) {
      if (validPointsLeft[i] && validPointsRight[i]) {
        cv::circle(left_im_calib, cv::Point2d{outPointsLeft(i, 0), outPointsLeft(i, 1)}, 5, 255, 2);
        cv::line(left_im_calib,
                 cv::Point2d{outPointsLeft(i, 0), outPointsLeft(i, 1)},
                 cv::Point2d{outPointsLeft(i, 0),
                             outPointsLeft(i, 1) + 4.0 * (outPointsRight(i, 1) - outPointsLeft(i, 1))}, 255, 2);

//        printf("Point[%d], Y-diff: %f\n", i, fabs(outPointsLeft(i, 1) - outPointsRight(i, 1)));
//        cv::circle(left_im_calib, cv::Point2d{outPointsLeft(i, 0), outPointsLeft(i, 1)}, 5, 255, 2);
//        cv::putText(left_im_calib, std::to_string(i), cv::Point2d{outPointsLeft(i, 0), outPointsLeft(i, 1)},
//                    cv::FONT_HERSHEY_SIMPLEX, 1, 128, 2);
//        cv::circle(right_im_calib, cv::Point2d{outPointsRight(i, 0), outPointsRight(i, 1)}, 5, 255, 2);
//        cv::putText(right_im_calib, std::to_string(i), cv::Point2d{outPointsRight(i, 0), outPointsRight(i, 1)},
//                    cv::FONT_HERSHEY_SIMPLEX, 1, 128, 2);
      }
    }
    cv::imshow("left", left_im_calib);
    cv::imshow("right", right_im_calib);

    int k = cv::waitKey(10) & 0xff;
    if (k == 'q') {
      g_bExit = true;
      break;
    }
    if (k == 's') {
      static int count = 0;
      if (count == 0) {
        system("rm -rf dataset");
        system("mkdir -p dataset/cam0");
        system("mkdir -p dataset/cam1");
      }
      count++;
      cv::imwrite("dataset/cam0/" + std::to_string(count) + "000000000.png", left_im_calib);
      cv::imwrite("dataset/cam1/" + std::to_string(count) + "000000000.png", right_im_calib);
    }
  }

  for (int i = 0; i < CAMERA_NUM; i++) {
    // 停止取流
    // end grab image
    nRet = MV_CC_StopGrabbing(handle[i]);
    if (MV_OK != nRet) {
      printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
      return -1;
    }

    // 关闭设备
    // close device
    nRet = MV_CC_CloseDevice(handle[i]);
    if (MV_OK != nRet) {
      printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
      return -1;
    }

    // 销毁句柄
    // destroy handle
    nRet = MV_CC_DestroyHandle(handle[i]);
    if (MV_OK != nRet) {
      printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
      return -1;
    }
  }

  printf("exit\n");
  return 0;
}
