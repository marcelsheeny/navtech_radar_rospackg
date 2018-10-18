/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to
https://opensource.org/licenses/MIT
for full license details.
*/

#include <math.h>
#include <opencv2/opencv.hpp>
//#include "/home/sapratshi/catkin_ws/src/navtech_radar/include/navtech_radar/radarclient.h"
#include "radarclient.h"
//#include "/home/sapratshi/navtech/navtech_radar_sdk/cpp/radarclient.h"
#include <ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>


using namespace Navtech;
image_transport::Publisher pub;
uint16_t _lastAzimuth = 0;
uint64_t _lastRotationReset = Helpers::Now();
RadarClientPtr_t _radarClient;
int azimuth_counter = 0;
constexpr int ROWS = 576;
constexpr int COLS = 400;
cv::Mat radar_image = cv::Mat::zeros(ROWS, COLS, CV_8UC1);
long frame_number = 0;
cv::Mat X;
cv::Mat Y;
cv::Mat radar_image_cart;

static void meshgrid(const cv::Mat &xgv, const cv::Mat &ygv, cv::Mat &X,
                     cv::Mat &Y) {
  cv::repeat(xgv.reshape(1, 1), ygv.total(), 1, X);
  cv::repeat(ygv.reshape(1, 1).t(), 1, xgv.total(), Y);
}

void remap(const cv::Mat &im, const cv::Mat &x, const cv::Mat &y,
           cv::Mat &output, const int width) {
  float ratio = 1.0;
  int height = round(width * ratio);
  double min_x, max_x;
  cv::minMaxLoc(x, &min_x, &max_x);
  cv::Mat div_x;
  cv::divide((x - min_x), (max_x - min_x), div_x);
  cv::Mat xx = (div_x * (height - 1));

  double min_y, max_y;
  cv::minMaxLoc(y, &min_y, &max_y);
  cv::Mat div_y;
  cv::divide((y - min_y), (max_y - min_y), div_y);
  cv::Mat yy = (div_y * (width - 1));

  output = cv::Mat::zeros(height, width, CV_8UC1);
  for (int i = 0; i < im.rows; i++) {
    for (int j = 0; j < im.cols; j++) {
      output.at<uchar>(static_cast<int>(floor(xx.at<float>(i, j))),
                       static_cast<int>(floor(yy.at<float>(i, j)))) =
          im.at<uchar>(i, j);
    }
  }
}

void polar_to_cartesian(const cv::Mat &im, cv::Mat &output, const int &size) {
  cv::Mat theta = cv::Mat::zeros(1, im.cols, CV_32FC1);

  cv::Mat range = cv::Mat::zeros(1, ROWS, CV_32FC1);

  for (int i = 0; i < im.cols; i++) {
    theta.at<float>(0, i) = (i * 2.0 * M_PI) / im.cols;
  }

  for (int i = 0; i < ROWS; i++) {
    range.at<float>(0, i) = i;
  }

  cv::Mat angles;
  cv::Mat ranges;
  meshgrid(theta, range, angles, ranges);
  cv::polarToCart(ranges, angles, X, Y);
  remap(im, X, Y, output, size);
}

// FFT data handler callback
void FFTDataHandler(const FFTDataPtr_t &data) {
  if (data->Azimuth < _lastAzimuth) {
    auto diff = Helpers::Now() - _lastRotationReset;
    Helpers::Log("FFTDataHandler - Rotating @ [" +
                 std::to_string(1000.0 / diff) + "Hz]");
    // Helpers::Log("Angle:" + std::to_string(data->Angle));
    // Helpers::Log("NTPSeconds:" + std::to_string(data->NTPSeconds));
    // Helpers::Log("NTPSplitSeconds:" + std::to_string(data->NTPSplitSeconds));
    // Helpers::Log("SweepCounter:" + std::to_string(data->SweepCounter));
    // Helpers::Log("Azimuth:" + std::to_string(data->Azimuth));
    _lastRotationReset = Helpers::Now();
    Helpers::Log("Number of angles" + std::to_string(azimuth_counter));
    if (frame_number > 2) {
      cv::imshow("Radar Polar", radar_image);
      // threshold(radar_image, radar_image, 50, 255, 3);
      cv::resize(radar_image, radar_image, cv::Size(10000, radar_image.rows));
      polar_to_cartesian(radar_image, radar_image_cart, ROWS);
      cv::flip(radar_image_cart, radar_image_cart, 1);
      cv::imshow("Radar Cartesian", radar_image_cart);
      cv::waitKey(1);
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", radar_image_cart).toImageMsg();
	  pub.publish(msg);
      // radar_image_cart = cv::Scalar(0);
      radar_image = cv::Mat(ROWS, COLS, CV_8UC1);
    }

    // update/reset variables
    frame_number++;
    azimuth_counter = 0;
  }
  // populate image
  if (frame_number > 2) {
    for (size_t i = 0; i < data->Data.size(); i++) {
      radar_image.at<uchar>(i, azimuth_counter) = data->Data[i];
    }
  }

  // Helpers::Log(std::to_string(data->Data.size()));

  // update
  azimuth_counter++;
  _lastAzimuth = data->Azimuth;
}

void NavigationDataHandler(const NavigationDataPtr_t &data) {
  if (data->Azimuth < _lastAzimuth) {
    auto diff = Helpers::Now() - _lastRotationReset;
    Helpers::Log("FFTDataHandler - Rotating @ [" +
                 std::to_string(1000.0 / diff) + "Hz]");
    _lastRotationReset = Helpers::Now();
    auto peaks = data->Peaks;
  }

  auto peaks2 = data->Peaks;

  _lastAzimuth = data->Azimuth;
}

void ConfigurationDataHandler(const ConfigurationDataPtr_t &data) {
  Helpers::Log("ConfigurationDataHandler - Expected Rotation Rate [" +
               std::to_string(data->ExpectedRotationRate) + "Hz]");
  Helpers::Log("ConfigurationDataHandler - Range In Bins [" +
               std::to_string(data->RangeInBins) + "]");
  Helpers::Log("ConfigurationDataHandler - Bin Size [" +
               std::to_string(data->BinSize / 10000.0) + "cm]");
  Helpers::Log("ConfigurationDataHandler - Range In Metres [" +
               std::to_string((data->BinSize / 10000.0) * data->RangeInBins) +
               "m]");
  Helpers::Log("ConfigurationDataHandler - Azimuth Samples [" +
               std::to_string(data->AzimuthSamples) + "]");

  _radarClient->StartFFTData();
  //_radarClient->StartNavigationData();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "testclient_dynamic");
  ros::NodeHandle node;
 image_transport::ImageTransport it(node);
  pub = it.advertise("camera/image", 1);

  Helpers::Log("Test Client Starting");

  _radarClient = std::make_shared<RadarClient>("192.168.0.1");
  _radarClient->SetFFTDataCallback(
      std::bind(&FFTDataHandler, std::placeholders::_1));

  _radarClient->SetNavigationDataCallback(
      std::bind(&NavigationDataHandler, std::placeholders::_1));
  _radarClient->SetConfigurationDataCallback(
      std::bind(&ConfigurationDataHandler, std::placeholders::_1));
  _radarClient->Start();

  std::this_thread::sleep_for(std::chrono::milliseconds(10000000));
  _radarClient->StopFFTData();
  //_radarClient->StopNavigationData();

  _radarClient->Stop();

  return 0;
}
