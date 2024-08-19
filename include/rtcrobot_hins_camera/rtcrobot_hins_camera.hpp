#ifndef _RTCROBOT_HINS_CAMERA_HPP_
#define _RTCROBOT_HINS_CAMERA_HPP_

#include "change_parameter.hpp"
#include "epoll_udp.hpp"
#include "hinson_type.hpp"
#include "query_image.hpp"
#include "query_recognizer.hpp"
#include "read_parameter.hpp"
#include "set_code_length.hpp"
#include "set_type_code.hpp"
#include "socket_udp.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rtcrobot_interfaces/srv/change_code_length.hpp"
#include "rtcrobot_interfaces/srv/change_code_type.hpp"
#include "rtcrobot_interfaces/srv/change_param_camera_hins.hpp"
#include "rtcrobot_interfaces/srv/read_param_camera_hins.hpp"
#include "rtcrobot_navutil/lifecycle_node.hpp"
#include "rtcrobot_navutil/robot_utils.hpp"
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
namespace rtcrobot_hins_camera {

using PoseStamped          = geometry_msgs::msg::PoseStamped;
using ReadParamCameraHins  = rtcrobot_interfaces::srv::ReadParamCameraHins;
using ChangeParamCameraHins= rtcrobot_interfaces::srv::ChangeParamCameraHins;
using ChangeCodeLength     = rtcrobot_interfaces::srv::ChangeCodeLength;
using ChangeCodeType       = rtcrobot_interfaces::srv::ChangeCodeType;
using String               = std_msgs::msg::String;
using Trigger              = std_srvs::srv::Trigger;
class RtcrobotHinsCamera : public rtcrobot_navutil::LifecycleNode {
private:
  std::shared_ptr<SocketUDP> socket_;
  std::shared_ptr<Epoll>     epoll_;

  //   param socketUDP
  std::string address_{"192.168.1.88"};
  int         port_{8888}, gain_{246}, exposure_{33}, pwm_{80}, corrosion_{1};

  // publisher
  rclcpp_lifecycle::LifecyclePublisher<PoseStamped>::SharedPtr
                                                          publisher_dm_code_;
  rclcpp_lifecycle::LifecyclePublisher<String>::SharedPtr publisher_images_;
  // subscriber

  // service
  rclcpp::Service<ChangeParamCameraHins>::SharedPtr change_parameter_service_;
  rclcpp::Service<ReadParamCameraHins>::SharedPtr   read_parameter_service_;
  rclcpp::Service<ChangeCodeLength>::SharedPtr      change_code_length_service_;
  rclcpp::Service<ChangeCodeType>::SharedPtr        change_code_type_service_;
  rclcpp::Service<Trigger>::SharedPtr               pub_images_service_;
  rclcpp::Service<Trigger>::SharedPtr               unpub_images_service_;

  // excutor_thead
  rclcpp::CallbackGroup::SharedPtr                     callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<rtcrobot_navutil::NodeThread>        excutor_thread_;

  // atomic
  std::atomic<bool>                   is_ready_{false};
  std::atomic<PoseStamped::SharedPtr> dm_code_;
  std::atomic<String::SharedPtr>      images_;
  std::atomic<int> flag_read_param_{0}, flag_change_param_{0},
      flag_change_code_length_{0}, flag_change_code_type_{0},
      flag_pub_images_{-1};

  // thread
  std::thread thread_epoll_;
  std::thread thread_publisher_images_;

  // mutex
  std::mutex mutex_socket_;

public:
  explicit RtcrobotHinsCamera();
  ~RtcrobotHinsCamera();

  rtcrobot_navutil::CallbackReturn
      on_configure(const rclcpp_lifecycle::State &state) override;

  rtcrobot_navutil::CallbackReturn
      on_activate(const rclcpp_lifecycle::State &state) override;

  rtcrobot_navutil::CallbackReturn
      on_deactivate(const rclcpp_lifecycle::State &state) override;

  rtcrobot_navutil::CallbackReturn
      on_cleanup(const rclcpp_lifecycle::State &state) override;

  rtcrobot_navutil::CallbackReturn
      on_shutdown(const rclcpp_lifecycle::State &state) override;

protected:
  void initParameters();
  void initNetwork();
  void initService();
  void initPubSub();

private:
  void threadEpoll();
  void threadPublisherImages();

private:
  void changeParamCallback(
      const std::shared_ptr<ChangeParamCameraHins::Request> request,
      std::shared_ptr<ChangeParamCameraHins::Response>      response);

  void readParamCallback(
      const std::shared_ptr<ReadParamCameraHins::Request> request,
      std::shared_ptr<ReadParamCameraHins::Response>      response);

  void changeCodeLengthCallback(
      const std::shared_ptr<ChangeCodeLength::Request> request,
      std::shared_ptr<ChangeCodeLength::Response>      response);

  void changeCodeTypeCallback(
      const std::shared_ptr<ChangeCodeType::Request> request,
      std::shared_ptr<ChangeCodeType::Response>      response);

  void pubImagesCallback(
      const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response>      response);

  void unpubImagesCallback(
      const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response>      response);
};

} // namespace rtcrobot_hins_camera
#endif //_RTCROBOT_HINS_CAMERA_HPP_
