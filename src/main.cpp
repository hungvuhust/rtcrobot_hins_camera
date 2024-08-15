#include "rtcrobot_hins_camera/rtcrobot_hins_camera.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node= std::make_shared<rtcrobot_hins_camera::RtcrobotHinsCamera>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}