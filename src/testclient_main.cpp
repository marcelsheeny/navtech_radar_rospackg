/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to
https://opensource.org/licenses/MIT
for full license details.
*/

#include <math.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
//#include
//"/home/sapratshi/catkin_ws/src/navtech_radar/include/navtech_radar/radarclient.h"
#include "/home/saptarshi/navtech/navtech_radar_sdk/cpp/radarclient.h"
//#include "/home/sapratshi/navtech/navtech_radar_sdk/cpp/radarclient.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
//#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>s

using namespace Navtech;
image_transport::Publisher CartesianPublisher, PolarPublisher;
ros::Publisher navigationDataPublisher;
uint16_t _lastAzimuth = 0;
uint16_t _lastAzimuth_navigation = 0;
uint64_t _lastRotationReset = Helpers::Now();
RadarClientPtr_t _radarClient;
int azimuth_counter = 0;
int azimuth_counter_navigation = 0;
constexpr int ROWS = 576;
constexpr int COLS = 400;
cv::Mat radar_image_polar = cv::Mat::zeros(ROWS, COLS, CV_8UC1);
cv::Mat radar_image = cv::Mat::zeros(ROWS, COLS, CV_8UC1);
long frame_number = 0;
cv::Mat X;
cv::Mat Y;
cv::Mat radar_image_cart;
std::vector<std::vector<std::tuple<float, uint16_t>>> all_peaks_raw;
std::vector<uint16_t> all_azimuths_raw;
std::vector<std::vector<float>> all_peaks_cart;
std_msgs::Header header;  // empty header

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
    Helpers::Log("FFT data captured.");
    // auto diff = Helpers::Now() - _lastRotationReset;
    // Helpers::Log("FFTDataHandler - Rotating @ [" +
    //              std::to_string(1000.0 / diff) + "Hz]");
    // Helpers::Log("Angle:" + std::to_string(data->Angle));
    // Helpers::Log("NTPSeconds:" + std::to_string(data->NTPSeconds));
    // Helpers::Log("NTPSplitSeconds:" + std::to_string(data->NTPSplitSeconds));
    // Helpers::Log("SweepCounter:" + std::to_string(data->SweepCounter));
    // Helpers::Log("Azimuth:" + std::to_string(data->Azimuth));
    _lastRotationReset = Helpers::Now();
    // Helpers::Log("Number of angles" + std::to_string(azimuth_counter));
    if (frame_number > 2) {
      //  cv::imshow("Radar Polar", radar_image);
      // threshold(radar_image, radar_image, 50, 255, 3);
      cv::resize(radar_image_polar, radar_image,
                 cv::Size(10000, radar_image.rows));
      polar_to_cartesian(radar_image, radar_image_cart, ROWS * 2);
      cv::flip(radar_image_cart, radar_image_cart, 0);
      // cv::flip(radar_image_cart, radar_image_cart, 1);
      // cv::imshow("Radar Cartesian", radar_image_cart);
      //  cv::waitKey(1);

      // Publish the cartesian and polar image

      header.seq = frame_number;        // user defined counter
      header.stamp = ros::Time::now();  // time

      sensor_msgs::ImagePtr PolarMsg =
          cv_bridge::CvImage(header, "mono8", radar_image_polar).toImageMsg();
      sensor_msgs::ImagePtr CartesianMsg =
          cv_bridge::CvImage(header, "mono8", radar_image_cart).toImageMsg();
      CartesianPublisher.publish(CartesianMsg);
      PolarPublisher.publish(PolarMsg);
      // radar_image_cart = cv::Scalar(0);
      radar_image = cv::Mat(ROWS, COLS, CV_8UC1);
    }

    // update/reset variables
    frame_number++;
    azimuth_counter = 0;
  }
  // populate image
  if (frame_number > 2) {
    int bearing = (data->Azimuth / 5600.0) * 400.0;

    for (size_t i = 0; i < data->Data.size(); i++) {
      radar_image_polar.at<uchar>(i, bearing) = data->Data[i];
    }

    // Helpers::Log("azimuth: " + std::to_string(data->Azimuth));
    // Helpers::Log("bearing: " + std::to_string(bearing));
  }

  // Helpers::Log(std::to_string(data->Data.size()));

  // update
  azimuth_counter++;
  _lastAzimuth = data->Azimuth;
}

void NavigationDataHandler(const NavigationDataPtr_t &data) {
  if (data->Azimuth < _lastAzimuth_navigation) {
    Helpers::Log("Navigation data captured.");
    //   auto diff = Helpers::Now() - _lastRotationReset;
    //   Helpers::Log("FFTDataHandler - Rotating @ [" +
    //                std::to_string(1000.0 / diff) + "Hz]");
    //   _lastRotationReset = Helpers::Now();
    // auto peaks = data->Peaks;
    // Helpers::Log(std::to_string(peaks.size()));
    azimuth_counter_navigation = 0;

    // process all peaks and submit to ros

    // polar to cartesian
    size_t N = all_peaks_raw.size();

    for (size_t i = 0; i < N; i++) {
      // float theta = (static_cast<float>(i) / static_cast<float>(N)) * (2.0 *
      // M_PI);
      float theta = (all_azimuths_raw[i] / 5600.0) * (2.0 * M_PI);
      for (size_t j = 0; j < all_peaks_raw[i].size(); j++) {
        float x = std::get<0>(all_peaks_raw[i][j]) * cos(theta);
        float y = std::get<0>(all_peaks_raw[i][j]) * sin(theta);
        auto intensity = std::get<1>(all_peaks_raw[i][j]);

        std::vector<float> xyi;
        xyi.push_back(x);
        xyi.push_back(y);
        xyi.push_back(static_cast<float>(intensity));

        all_peaks_cart.push_back(xyi);
        // Helpers::Log("X:" + std::to_string(all_peaks_cart.back()[0]));
        // Helpers::Log("Y:" + std::to_string(all_peaks_cart.back()[1]));
        // Helpers::Log("I:" + std::to_string(all_peaks_cart.back()[2]));
        // Helpers::Log("----------------------");
      }

      // ----------------------------__ROS ----------------------------
      sensor_msgs::PointCloud2Ptr points_msg =
          boost::make_shared<sensor_msgs::PointCloud2>();
      points_msg->header = header;  // TODO fill in header
      points_msg->height =
          1;  // if this is a "full 3D" pointcloud, height is 1; if this is
              // Kinect-like pointcloud, height is the vertical resolution
      points_msg->width = 4;
      points_msg->is_bigendian = false;
      points_msg->is_dense = false;  // there may be invalid points

      sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);
      // this call also resizes the data structure according to the given width,
      // height and fields
      pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
      pcd_modifier.resize(50000);

      sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");
      sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*points_msg, "r");
      sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*points_msg, "g");
      sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*points_msg, "b");

      for (size_t ind = 0; ind < all_peaks_cart.size();
           ++ind, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
        // TODO fill in x, y, z, r, g, b local variables
        *iter_x = all_peaks_cart[ind][1];
        *iter_y = all_peaks_cart[ind][0];
        *iter_z = 0;
        *iter_r = static_cast<uint8_t>(all_peaks_cart[ind][2]);
        *iter_g = static_cast<uint8_t>(all_peaks_cart[ind][2]);
        *iter_b = static_cast<uint8_t>(all_peaks_cart[ind][2]);
        // Helpers::Log("X:" + std::to_string(all_peaks_cart[ind][0]));
        // Helpers::Log("Y:" + std::to_string(all_peaks_cart[ind][1]));
        // Helpers::Log("I:" +
        // std::to_string(static_cast<uint8_t>(all_peaks_cart[ind][2])));
        // Helpers::Log("IND:" + std::to_string(ind));
        // Helpers::Log("Size: " + std::to_string(all_peaks_cart.size()));
      }

      points_msg->header.frame_id = "velodyne";
      navigationDataPublisher.publish(points_msg);

      //---------------------------- ROS ----------------------------
      Helpers::Log(std::to_string(all_peaks_raw.size()));

      all_peaks_raw.clear();

      all_azimuths_raw.clear();
      all_peaks_cart.clear();
    }
  }

  auto peaks = data->Peaks;
  auto azimuth = data->Azimuth;

  // populate peaks
  all_peaks_raw.push_back(peaks);
  all_azimuths_raw.push_back(azimuth);

  // Helpers::Log(std::to_string(std::get<0>(peaks2[0])));

  azimuth_counter_navigation++;

  _lastAzimuth_navigation = data->Azimuth;

  // auto firstRange = std::get<0>(data->Peaks[0]);
  // auto firstPower = std::get<1>(data->Peaks[0]);
  // Helpers::Log("NavigationDataHandler - First Target [" +
  // std::to_string(firstRange) + "] [" + std::to_string(firstPower / 10) + "]");
}

void ConfigurationDataHandler(const ConfigurationDataPtr_t &data) {
  // Helpers::Log("ConfigurationDataHandler - Expected Rotation Rate [" +
  //              std::to_string(data->ExpectedRotationRate) + "Hz]");
  // Helpers::Log("ConfigurationDataHandler - Range In Bins [" +
  //              std::to_string(data->RangeInBins) + "]");
  // Helpers::Log("ConfigurationDataHandler - Bin Size [" +
  //              std::to_string(data->BinSize / 10000.0) + "cm]");
  // Helpers::Log("ConfigurationDataHandler - Range In Metres [" +
  //              std::to_string((data->BinSize / 10000.0) * data->RangeInBins)
  //              + "m]");
  // Helpers::Log("ConfigurationDataHandler - Azimuth Samples [" +
  //              std::to_string(data->AzimuthSamples) + "]");

  _radarClient->StartNavigationData();

  _radarClient->SetNavigationGainAndOffset(1.0f, 0.0f);
  _radarClient->SetNavigationThreshold(30 * 10);

  _radarClient->StartFFTData();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "testclient_dynamic");
  ros::NodeHandle node;
  image_transport::ImageTransport it(node);
  CartesianPublisher = it.advertise("Navtech/Cartesian", 1);
  PolarPublisher = it.advertise("Navtech/Polar", 1);

  navigationDataPublisher =
      node.advertise<sensor_msgs::PointCloud2>("Navtech/NavigationData", 1);

  Helpers::Log("Test Client Starting");

  // _radarClient->StartNavigationData();
  // _radarClient->StartFFTData();

  _radarClient = std::make_shared<RadarClient>("192.168.0.1");
  _radarClient->SetFFTDataCallback(
      std::bind(&FFTDataHandler, std::placeholders::_1));

  _radarClient->SetConfigurationDataCallback(
      std::bind(&ConfigurationDataHandler, std::placeholders::_1));
  _radarClient->SetNavigationDataCallback(
      std::bind(&NavigationDataHandler, std::placeholders::_1));

  _radarClient->Start();

  std::this_thread::sleep_for(std::chrono::milliseconds(10000000));
  _radarClient->StopFFTData();
  _radarClient->StopNavigationData();

  _radarClient->SetNavigationDataCallback();
  _radarClient->SetConfigurationDataCallback();
  _radarClient->SetFFTDataCallback();

  _radarClient->Stop();

  return 0;
}
