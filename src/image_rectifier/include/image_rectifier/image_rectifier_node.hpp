#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>


class ImageRectifierNode : public rclcpp::Node {
  public:
    explicit ImageRectifierNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~ImageRectifierNode() = default;
    void initialize();

  private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg);
    void updateCameraMatrices();

    // Subscribers
    image_transport::Subscriber imageSub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoSub;

    // Publishers
    image_transport::Publisher rectifiedImagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rectifiedCameraInfoPublisher;

    // Camera calibration data
    cv::Mat cameraMatrix;
    cv::Mat distortioncoeffs;
    cv::Mat rectificationMatrix;
    cv::Mat projectionMatrix;
    cv::Mat newCamMatrix;
    cv::Size image_size_;

    // Rectification maps
    cv::Mat map1;
    cv::Mat map2;
    bool mapsInitialized;

    // Camera info
    sensor_msgs::msg::CameraInfo::ConstSharedPtr latestCamInfo;
    bool camInfoReceived;

    // Parameters
    std::string inputImageTopic;
    std::string inputCameraInfoTopic;
    std::string outputImageTopic;
    std::string outputCamInfoTopic;
    double alpha; // Free scaling parameter for getOptimalNewCameraMatrix
    bool interpolate;
};

