#include "image_rectifier/image_rectifier_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

ImageRectifierNode::ImageRectifierNode(const rclcpp::NodeOptions &options)
    : Node("image_rectifier_node", options), mapsInitialized(false), camInfoReceived(false) {
    // Declare parameters
    this->declare_parameter<std::string>("input_image_topic", "image_raw");
    this->declare_parameter<std::string>("input_camera_info_topic", "camera_info");
    this->declare_parameter<std::string>("output_image_topic", "image_rect");
    this->declare_parameter<std::string>("output_camera_info_topic", "camera_info_rect");
    this->declare_parameter<double>("alpha", 0.0);
    this->declare_parameter<bool>("interpolate", true);

    // Get parameters
    this->get_parameter("input_image_topic", inputImageTopic);
    this->get_parameter("input_camera_info_topic", inputCameraInfoTopic);
    this->get_parameter("output_image_topic", outputImageTopic);
    this->get_parameter("output_camera_info_topic", outputCamInfoTopic);
    this->get_parameter("alpha", alpha);
    this->get_parameter("interpolate", interpolate);

    RCLCPP_INFO(this->get_logger(), "Image Rectifier Node Starting...");
    RCLCPP_INFO(this->get_logger(), "  Input image topic: %s", inputImageTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Input camera info topic: %s", inputCameraInfoTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output image topic: %s", outputImageTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output camera info topic: %s", outputCamInfoTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Alpha: %.2f", alpha);
    RCLCPP_INFO(this->get_logger(), "  Interpolation: %s", interpolate ? "enabled" : "disabled");
}

void ImageRectifierNode::initialize() {
    image_transport::ImageTransport it(shared_from_this());

    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS()).keep_last(10);

    // Create subscribers
    imageSub =
        it.subscribe(inputImageTopic, 10, std::bind(&ImageRectifierNode::imageCallback, this, std::placeholders::_1));
    cameraInfoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        inputCameraInfoTopic, sensor_qos, std::bind(&ImageRectifierNode::cameraInfoCallback, this, std::placeholders::_1));

    // Create publsihers
    rectifiedImagePublisher = it.advertise(outputImageTopic, 10);
    rectifiedCameraInfoPublisher = this->create_publisher<sensor_msgs::msg::CameraInfo>(outputCamInfoTopic, sensor_qos);

    RCLCPP_INFO(this->get_logger(), "Image Rectifier Node initialized successfully");
}

void ImageRectifierNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
    // Check if camera info has changed
    bool infoChanged = false;

    if (!camInfoReceived) {
        infoChanged = true;
        camInfoReceived = true;
        RCLCPP_INFO(this->get_logger(), "Received camera calibration info");
    } else if (latestCamInfo) {
        // Check if calibration parameters have changed
        if (msg->k != latestCamInfo->k || msg->d != latestCamInfo->d || msg->r != latestCamInfo->r ||
            msg->p != latestCamInfo->p || msg->height != latestCamInfo->height || msg->width != latestCamInfo->width) {
            infoChanged = true;
            RCLCPP_INFO(this->get_logger(), "Camera calibration info updated");
        }
    }

    latestCamInfo = msg;

    if (infoChanged) {
        updateCameraMatrices();
    }
}

void ImageRectifierNode::updateCameraMatrices() {
    if (!latestCamInfo) {
        return;
    }

    // Extract camera matrix (K)
    cameraMatrix = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        cameraMatrix.at<double>(i / 3, i % 3) = latestCamInfo->k[i];
    }

    // Extract distortion coefficients (D)
    distortioncoeffs = cv::Mat(latestCamInfo->d.size(), 1, CV_64F);
    for (size_t i = 0; i < latestCamInfo->d.size(); ++i) {
        distortioncoeffs.at<double>(i, 0) = latestCamInfo->d[i];
    }

    // Extract rectification matrix (R)
    rectificationMatrix = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        rectificationMatrix.at<double>(i / 3, i % 3) = latestCamInfo->r[i];
    }

    // Extract projection matrix (P)
    projectionMatrix = cv::Mat(3, 4, CV_64F);
    for (int i = 0; i < 12; ++i) {
        projectionMatrix.at<double>(i / 4, i % 4) = latestCamInfo->p[i];
    }

    // Image size
    image_size_ = cv::Size(latestCamInfo->width, latestCamInfo->height);

    // Compute optimal new camera matrix
    newCamMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distortioncoeffs, image_size_, alpha, image_size_);

    RCLCPP_INFO(this->get_logger(), "Camera Matrix (K):");
    RCLCPP_INFO(this->get_logger(), "  [%.2f, %.2f, %.2f]", cameraMatrix.at<double>(0,0), cameraMatrix.at<double>(0,1), cameraMatrix.at<double>(0,2));
    RCLCPP_INFO(this->get_logger(), "  [%.2f, %.2f, %.2f]", cameraMatrix.at<double>(1,0), cameraMatrix.at<double>(1,1), cameraMatrix.at<double>(1,2));
    RCLCPP_INFO(this->get_logger(), "  [%.2f, %.2f, %.2f]", cameraMatrix.at<double>(2,0), cameraMatrix.at<double>(2,1), cameraMatrix.at<double>(2,2));
    
    RCLCPP_INFO(this->get_logger(), "Distortion Coefficients (D): ");
    for (size_t i = 0; i < distortioncoeffs.rows; ++i) {
        RCLCPP_INFO(this->get_logger(), "  d[%zu] = %.6f", i, distortioncoeffs.at<double>(i, 0));
    }

    // Initialize rectification maps
    cv::initUndistortRectifyMap(cameraMatrix, distortioncoeffs, rectificationMatrix, newCamMatrix, image_size_,
                                CV_32F, map1, map2);

    mapsInitialized = true;

    RCLCPP_INFO(this->get_logger(), "Rectification maps initialized for image size: %dx%d", image_size_.width,
                image_size_.height);
}

void ImageRectifierNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    if (!mapsInitialized) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for camera calibration info...");
        return;
    }

    // Convert ROS image to OpenCV
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Check if image size matches expected size
    if (cv_ptr->image.cols != image_size_.width || cv_ptr->image.rows != image_size_.height) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Image size mismatch! Expected %dx%d but got %dx%d. Reinitializing maps...",
                             image_size_.width, image_size_.height, cv_ptr->image.cols, cv_ptr->image.rows);

        image_size_ = cv::Size(cv_ptr->image.cols, cv_ptr->image.rows);
        updateCameraMatrices();
    }

    cv::Mat rectifiedImage;
    cv::remap(cv_ptr->image, rectifiedImage, map1, map2, cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    // Convert back to ROS message
    cv_bridge::CvImage rectified_msg;
    rectified_msg.header = msg->header;
    rectified_msg.encoding = msg->encoding;
    rectified_msg.image = rectifiedImage;

    // Publish rectified image
    rectifiedImagePublisher.publish(rectified_msg.toImageMsg());

    // Publish rectified camera info
    if (latestCamInfo) {
        auto rectified_info = std::make_shared<sensor_msgs::msg::CameraInfo>(*latestCamInfo);
        rectified_info->header = msg->header;

        // Update camera matrix to the new (rectified) camera matrix
        for (int i = 0; i < 9; ++i) {
            rectified_info->k[i] = newCamMatrix.at<double>(i / 3, i % 3);
        }

        // Rectified image has no distortion
        rectified_info->d.clear();
        rectified_info->d.resize(5, 0.0);

        // Identity rectification matrix
        rectified_info->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

        // Update projection matrix (3x4)
        for (int i = 0; i < 12; ++i) {
            if (i < 9) {
                rectified_info->p[i] = newCamMatrix.at<double>(i / 3, i % 3);
            } else {
                rectified_info->p[i] = 0.0;
            }
        }

        rectifiedCameraInfoPublisher->publish(*rectified_info);
    }
}
