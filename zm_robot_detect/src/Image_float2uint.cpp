#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class ImageConvertNode : public rclcpp::Node
{
public:
  ImageConvertNode()
  : Node("image_32fc1_to_16uc1")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/kinect_v2/depth/image_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&ImageConvertNode::imageCallback, this, std::placeholders::_1)
    );

    pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/kinect_v2/depth/image_data",
      rclcpp::SensorDataQoS()
    );

    RCLCPP_INFO(this->get_logger(), "Image converter node started.");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // 確認格式
    if (msg->encoding != "32FC1") {
      RCLCPP_WARN(this->get_logger(),
                  "Unsupported encoding: %s", msg->encoding.c_str());
      return;
    }

    // 轉成 OpenCV
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "32FC1");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    const cv::Mat &src = cv_ptr->image;
    cv::Mat dst(src.rows, src.cols, CV_16UC1);

    // scale：float → uint16
    const float scale = 1000.0f;  // 公尺 → 毫米（依需求調整）

    for (int r = 0; r < src.rows; ++r) {
      for (int c = 0; c < src.cols; ++c) {
        float val = src.at<float>(r, c);

        if (std::isfinite(val) && val > 0.0f) {
          uint32_t scaled = static_cast<uint32_t>(val * scale);
          dst.at<uint16_t>(r, c) =
            static_cast<uint16_t>(std::min(scaled, static_cast<uint32_t>(65535)));
        } else {
          dst.at<uint16_t>(r, c) = 0;
        }
      }
    }

    // 封裝成 ROS Image
    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = "16UC1";
    out_msg.image = dst;

    pub_->publish(*out_msg.toImageMsg());
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageConvertNode>());
  rclcpp::shutdown();
  return 0;
}
