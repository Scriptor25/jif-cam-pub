#include <chrono>
#include <cstdio>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

using namespace std::chrono_literals;

/*#define CAMERA_COUNT 8

class CamNode : public rclcpp::Node
{
public:
  CamNode()
      : Node("cam_pub_node")
  {
    int i = 0;
    for (; i < CAMERA_COUNT; i++)
      if (m_Video.open(i))
        break;

    if (i >= CAMERA_COUNT)
    {
      std::cerr << "Failed to open camera" << std::endl;
      return;
    }

    m_Timer = create_wall_timer(500ms, std::bind(&CamNode::timer_callback, this));
    m_Publisher = create_publisher<sensor_msgs::msg::CompressedImage>("/compressed", 10);
  }

private:
  void timer_callback()
  {
    cv::Mat image;
    if (!m_Video.read(image))
      return;

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toCompressedImageMsg();
    m_Publisher->publish(*msg);
  }

private:
  cv::VideoCapture m_Video;
  rclcpp::TimerBase::SharedPtr m_Timer;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_Publisher;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto cam_node = std::make_shared<CamNode>();
  rclcpp::spin(cam_node);
  rclcpp::shutdown();
  return 0;
}
*/

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("cam_pub_node", options);
  image_transport::ImageTransport it(node);
  image_transport::Publisher pub = it.advertise("/cam_pub", 1);

  cv::VideoCapture cap(0);
  if (!cap.isOpened())
    return 1;
  cv::Mat frame;
  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr msg;

  rclcpp::WallRate loop_rate(5ms);
  while (rclcpp::ok())
  {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if (!frame.empty())
    {
      msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
