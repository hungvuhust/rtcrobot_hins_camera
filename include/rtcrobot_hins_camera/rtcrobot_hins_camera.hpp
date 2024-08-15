#ifndef _RTCROBOT_HINS_CAMERA_HPP_
#define _RTCROBOT_HINS_CAMERA_HPP_

#include "change_parameter.hpp"
#include "epoll_udp.hpp"
#include "hinson_type.hpp"
#include "query_image.hpp"
#include "query_recognizer.hpp"
#include "read_parameter.hpp"
#include "set_code_length.hpp"
#include "socket_udp.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rtcrobot_interfaces/srv/change_param_camera_hins.hpp"
#include "rtcrobot_interfaces/srv/read_param_camera_hins.hpp"
#include "rtcrobot_navutil/lifecycle_node.hpp"
#include "rtcrobot_navutil/robot_utils.hpp"
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace rtcrobot_hins_camera {

using PoseStamped          = geometry_msgs::msg::PoseStamped;
using ReadParamCameraHins  = rtcrobot_interfaces::srv::ReadParamCameraHins;
using ChangeParamCameraHins= rtcrobot_interfaces::srv::ChangeParamCameraHins;

class RtcrobotHinsCamera : public rtcrobot_navutil::LifecycleNode {
private:
  std::shared_ptr<SocketUDP> socket_;
  std::shared_ptr<Epoll>     epoll_;

  //   param socketUDP
  std::string address_{"192.168.1.88"};
  int         port_{8888}, gain{246}, exposure{33}, pwm{80}, corrosion{1};

  // publisher
  rclcpp_lifecycle::LifecyclePublisher<PoseStamped>::SharedPtr
      publisher_dm_code_;
  // subscriber

  // service
  rclcpp::Service<ChangeParamCameraHins>::SharedPtr change_parameter_service_;
  rclcpp::Service<ReadParamCameraHins>::SharedPtr   read_parameter_service_;

  // excutor_thead
  rclcpp::CallbackGroup::SharedPtr                     callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<rtcrobot_navutil::NodeThread>        excutor_thread_;

  // atomic
  std::atomic<bool>                   is_ready_{false};
  std::atomic<PoseStamped::SharedPtr> dm_code_;

  // thread
  std::thread thread_epoll_;
  std::thread thread_publisher_;

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
  void threadPublisher();

private:
  void changeParamCallback(
      const std::shared_ptr<ChangeParamCameraHins::Request> request,
      std::shared_ptr<ChangeParamCameraHins::Response>      response);

  void readParamCallback(
      const std::shared_ptr<ReadParamCameraHins::Request> request,
      std::shared_ptr<ReadParamCameraHins::Response>      response);
};

} // namespace rtcrobot_hins_camera
#endif //_RTCROBOT_HINS_CAMERA_HPP_
