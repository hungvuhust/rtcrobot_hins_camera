#include "rtcrobot_hins_camera/rtcrobot_hins_camera.hpp"
#include <sstream>
namespace rtcrobot_hins_camera {

using namespace hinson_protocol;

RtcrobotHinsCamera::RtcrobotHinsCamera()
    : LifecycleNode("rtcrobot_hins_camera") {}

RtcrobotHinsCamera::~RtcrobotHinsCamera() {
  RCLCPP_INFO(get_logger(), "Destructor");
  is_ready_.store(false);
  if (thread_epoll_.joinable()) {
    thread_epoll_.join();
  }
  if (thread_publisher_.joinable()) {
    thread_publisher_.join();
  }
}

rtcrobot_navutil::CallbackReturn
    RtcrobotHinsCamera::on_configure(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Configuring");
  initParameters();
  initNetwork();
  initPubSub();
  callback_group_=
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  executor_= std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, get_node_base_interface());
  excutor_thread_= std::make_unique<rtcrobot_navutil::NodeThread>(executor_);
  initService();

  return rtcrobot_navutil::CallbackReturn::SUCCESS;
}

rtcrobot_navutil::CallbackReturn
    RtcrobotHinsCamera::on_activate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating");
  RCLCPP_INFO(get_logger(), "Activating Publisher");
  publisher_dm_code_->on_activate();
  RCLCPP_INFO(get_logger(), "Activating Thread");
  is_ready_.store(true);
  thread_epoll_    = std::thread(&RtcrobotHinsCamera::threadEpoll, this);
  thread_publisher_= std::thread(&RtcrobotHinsCamera::threadPublisher, this);
  RCLCPP_INFO(get_logger(), "Activated");

  return rtcrobot_navutil::CallbackReturn::SUCCESS;
}

rtcrobot_navutil::CallbackReturn
    RtcrobotHinsCamera::on_deactivate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  publisher_dm_code_->on_deactivate();
  is_ready_.store(false);
  if (thread_epoll_.joinable()) {
    thread_epoll_.join();
  }
  if (thread_publisher_.joinable()) {
    thread_publisher_.join();
  }
  return rtcrobot_navutil::CallbackReturn::SUCCESS;
}

rtcrobot_navutil::CallbackReturn
    RtcrobotHinsCamera::on_cleanup(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cleaning");

  publisher_dm_code_.reset();
  is_ready_.store(false);
  if (thread_epoll_.joinable()) {
    thread_epoll_.join();
  }
  if (thread_publisher_.joinable()) {
    thread_publisher_.join();
  }
  return rtcrobot_navutil::CallbackReturn::SUCCESS;
}

rtcrobot_navutil::CallbackReturn
    RtcrobotHinsCamera::on_shutdown(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Shutting down");

  publisher_dm_code_.reset();
  is_ready_.store(false);
  if (thread_epoll_.joinable()) {
    thread_epoll_.join();
  }
  if (thread_publisher_.joinable()) {
    thread_publisher_.join();
  }
  return rtcrobot_navutil::CallbackReturn::SUCCESS;
}

void RtcrobotHinsCamera::initParameters() {
  get_parameter("address", address_);
  get_parameter("port", port_);
  get_parameter("gain", gain);
  get_parameter("exposure", exposure);
  get_parameter("pwm", pwm);
  get_parameter("corrosion", corrosion);

  RCLCPP_INFO_STREAM(get_logger(), "address: " << address_.c_str());
  RCLCPP_INFO_STREAM(get_logger(), "port: " << port_);
  RCLCPP_INFO_STREAM(get_logger(), "gain: " << gain);
  RCLCPP_INFO_STREAM(get_logger(), "exposure: " << exposure);
  RCLCPP_INFO_STREAM(get_logger(), "pwm: " << pwm);
  RCLCPP_INFO_STREAM(get_logger(), "corrosion: " << corrosion);
}

void RtcrobotHinsCamera::initNetwork() {
  socket_= std::make_shared<SocketUDP>(address_, port_);
  epoll_ = std::make_shared<Epoll>();
  epoll_->addSocket(socket_->getFd(), EPOLLIN);
}

void RtcrobotHinsCamera::initService() {
  change_parameter_service_= create_service<ChangeParamCameraHins>(
      "/change_parameter",
      std::bind(
          &RtcrobotHinsCamera::changeParamCallback, this, std::placeholders::_1,
          std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
  read_parameter_service_= create_service<ReadParamCameraHins>(
      "/read_parameter",
      std::bind(
          &RtcrobotHinsCamera::readParamCallback, this, std::placeholders::_1,
          std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
}

void RtcrobotHinsCamera::initPubSub() {
  publisher_dm_code_= create_publisher<PoseStamped>(
      "/align_code",
      rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable());
}

void RtcrobotHinsCamera::changeParamCallback(
    const std::shared_ptr<ChangeParamCameraHins::Request> request,
    std::shared_ptr<ChangeParamCameraHins::Response>      response) {
  RCLCPP_INFO(get_logger(), "Change parameter");
}

void RtcrobotHinsCamera::readParamCallback(
    const std::shared_ptr<ReadParamCameraHins::Request> request,
    std::shared_ptr<ReadParamCameraHins::Response>      response) {
  RCLCPP_INFO(get_logger(), "Read parameter");
}

void RtcrobotHinsCamera::threadEpoll() {
  std::vector<struct epoll_event> events;
  while (is_ready_) {
    events= epoll_->wait();
    for (auto &event: events) {
      if (event.events & EPOLLIN) {
        uint8_t buffer[32768];
        ssize_t n= epoll_->read(event.data.fd, buffer, sizeof(buffer));
        buffer[n]= '\0';
        std::vector<uint8_t> response(buffer, buffer + n);
        if (buffer[0] == 0x7B && buffer[1] == 0x22 && buffer[n - 1] == 0x7D &&
            buffer[n - 2] == 0x22) {
          // query image
          QueryImage::Response resp(response);
          continue;
        }

        if (!isValidFrame(buffer, n)) {
          std::cerr << "Invalid frame" << std::endl;
          continue;
        }
        // print_debug_recv(buffer, n);
        if (buffer[8] == uint8_t(Command::QUERY_RECOGNIZER)) {
          QueryRecognizer::Response resp(response);
          PoseStamped::SharedPtr    msg= std::make_shared<PoseStamped>();

          msg->header.stamp= now();

          if (resp.isValid) {
            std::stringstream ss;
            ss << resp.id;

            msg->header.frame_id= ss.str();
            msg->pose.position.x= resp.x;
            msg->pose.position.y= resp.y;
            msg->pose.position.z= 0.0;
            if (resp.theta > 180.0) {
              resp.theta-= 360.0;
            }

            // theta -> quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, -resp.theta);
            msg->pose.orientation.x= q.x();
            msg->pose.orientation.y= q.y();
            msg->pose.orientation.z= q.z();
            msg->pose.orientation.w= q.w();
          }
          dm_code_.store(msg, std::memory_order_relaxed);
          continue;
        } else if (buffer[8] == uint8_t(Command::READ_PARAMETER)) {
          ReadParameter::Response resp(response);

          continue;
        } else if (buffer[8] == uint8_t(Command::CHANGE_PARAMETER)) {
          ChangeParameter::Response resp(response);
          continue;
        } else if (buffer[8] == uint8_t(Command::SET_CODE_LENGTH)) {
          SetCodeLength::Response resp(response);
          continue;
        }
      }
    }
  }
}

void RtcrobotHinsCamera::threadPublisher() {
  while (is_ready_) {
    if (!socket_->send(
            QueryRecognizer::Request(QueryRecognizer::MethodREQUEST::QUERY_AUTO)
                .getFrame())) {
      RCLCPP_ERROR(get_logger(), "Failed to send query recognizer");
    }
    auto msg=
        dm_code_.load(std::memory_order_relaxed); // get the latest dm_code
    if (msg) {
      publisher_dm_code_->publish(*msg);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

} // namespace rtcrobot_hins_camera