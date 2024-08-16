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
  // create bond connection
  createBond();
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
  destroyBond();
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
  get_parameter("gain", gain_);
  get_parameter("exposure", exposure_);
  get_parameter("pwm", pwm_);
  get_parameter("corrosion", corrosion_);

  RCLCPP_INFO_STREAM(get_logger(), "address: " << address_.c_str());
  RCLCPP_INFO_STREAM(get_logger(), "port: " << port_);
  RCLCPP_INFO_STREAM(get_logger(), "gain: " << gain_);
  RCLCPP_INFO_STREAM(get_logger(), "exposure: " << exposure_);
  RCLCPP_INFO_STREAM(get_logger(), "pwm: " << pwm_);
  RCLCPP_INFO_STREAM(get_logger(), "corrosion: " << corrosion_);
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

  change_code_length_service_= create_service<ChangeCodeLength>(
      "/change_code_length",
      std::bind(
          &RtcrobotHinsCamera::changeCodeLengthCallback, this,
          std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);

  change_code_type_service_= create_service<ChangeCodeType>(
      "/change_code_type",
      std::bind(
          &RtcrobotHinsCamera::changeCodeTypeCallback, this,
          std::placeholders::_1, std::placeholders::_2),
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
  RCLCPP_INFO_STREAM(
      get_logger(), "Change parameter: "
                        << "Gain: " << request->gain << " Exposure: "
                        << request->exposure << " PWM: " << request->pwm
                        << " Corrosion: " << request->corrosion);
  if (!socket_->send(ChangeParameter::Request(
                         request->gain, request->exposure, request->pwm,
                         request->corrosion)
                         .getFrame())) {
    RCLCPP_ERROR(get_logger(), "Cann't send socket to change parameter");
    response->status = false;
    response->message= "ERROR SOCKET SEND";
    return;
  }
  flag_change_param_.store(0);
  auto start_time= std::chrono::high_resolution_clock::now();
  RCLCPP_INFO(get_logger(), "Wait for response");
  while (std::chrono::high_resolution_clock::now() - start_time <
         std::chrono::seconds(2)) {
    if (flag_change_param_.load() == 1) {
      response->status = true;
      response->message= "SUCCESS";
      RCLCPP_INFO(get_logger(), "Change parameter success");
      return;
    } else if (flag_change_param_.load() == -1) {
      response->status = false;
      response->message= "FAILED";
      RCLCPP_ERROR(get_logger(), "Change parameter failed");
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_ERROR(get_logger(), "Change parameter timeout");
  response->status = false;
  response->message= "TIMEOUT";
  return;
}

void RtcrobotHinsCamera::readParamCallback(
    const std::shared_ptr<ReadParamCameraHins::Request> request,
    std::shared_ptr<ReadParamCameraHins::Response>      response) {
  RCLCPP_INFO(get_logger(), "Read parameter");

  if (!socket_->send(ReadParameter::Request().getFrame())) {
    RCLCPP_ERROR(get_logger(), "Cann't send socket to read parameter");
    response->status = false;
    response->message= "ERROR SOCKET SEND";
    return;
  }

  flag_read_param_.store(0);
  auto start_time= std::chrono::high_resolution_clock::now();
  RCLCPP_INFO(get_logger(), "Wait for response");
  while (std::chrono::high_resolution_clock::now() - start_time <
         std::chrono::seconds(2)) {
    if (flag_read_param_.load() == 1) {
      response->gain     = gain_;
      response->exposure = exposure_;
      response->pwm      = pwm_;
      response->corrosion= corrosion_;
      response->status   = true;
      response->message  = "SUCCESS";
      RCLCPP_INFO_STREAM(
          get_logger(), "Read parameter: "
                            << "Gain: " << response->gain << " Exposure: "
                            << response->exposure << " PWM: " << response->pwm
                            << " Corrosion: " << response->corrosion);
      return;
    } else if (flag_read_param_.load() == -1) {
      RCLCPP_ERROR(get_logger(), "Read parameter failed");
      response->status = false;
      response->message= "FAILED";
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_ERROR(get_logger(), "Read parameter timeout");
  response->status = false;
  response->message= "TIMEOUT";
  return;
}

void RtcrobotHinsCamera::changeCodeLengthCallback(
    const std::shared_ptr<ChangeCodeLength::Request> request,
    std::shared_ptr<ChangeCodeLength::Response>      response) {
  RCLCPP_INFO(get_logger(), "Change Code Lenght");
  if (!socket_->send(
          SetCodeLength::Request(uint16_t(request->length)).getFrame())) {
    RCLCPP_ERROR(get_logger(), "Cann't send socket change code length");
    response->status = false;
    response->message= "ERROR SOCKET SEND";
    return;
  }
  flag_change_code_length_.store(0);
  auto start_time= std::chrono::high_resolution_clock::now();
  RCLCPP_INFO(get_logger(), "Wait for response ...");
  while (std::chrono::high_resolution_clock::now() - start_time <
         std::chrono::seconds(2)) {
    if (flag_change_code_length_.load() == 1) {
      response->status = true;
      response->message= "SUCCESS";
      RCLCPP_INFO_STREAM(
          get_logger(),
          "Change length code to " << request->length << " success!");
      return;
    } else if (flag_change_code_length_.load() == -1) {
      response->status = false;
      response->message= "FAILED";
      RCLCPP_ERROR_STREAM(
          get_logger(),
          "Change length code to " << request->length << " failed!");
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_ERROR(get_logger(), "Change code length timeout!");
  response->status = false;
  response->message= "TIMEOUT";
  return;
}

void RtcrobotHinsCamera::changeCodeTypeCallback(
    const std::shared_ptr<ChangeCodeType::Request> request,
    std::shared_ptr<ChangeCodeType::Response>      response) {
  RCLCPP_INFO_STREAM(get_logger(), "Change Code Type: " << (request->type));
  if (!socket_->send(
          SetCodeLength::Request(uint16_t(request->type)).getFrame())) {
    RCLCPP_ERROR(get_logger(), "Cann't send socket change code type");
    response->status = false;
    response->message= "ERROR SOCKET SEND";
    return;
  }
  flag_change_code_type_.store(0);
  auto start_time= std::chrono::high_resolution_clock::now();
  RCLCPP_INFO(get_logger(), "Wait for response ...");
  while (std::chrono::high_resolution_clock::now() - start_time <
         std::chrono::seconds(2)) {
    if (flag_change_code_type_.load() == 1) {
      response->status = true;
      response->message= "SUCCESS";

      RCLCPP_INFO_STREAM(
          get_logger(),
          "Change length code to " << request->type << " success!");
      return;
    } else if (flag_change_code_type_.load() == -1) {
      response->status = false;
      response->message= "FAILED";

      RCLCPP_ERROR_STREAM(
          get_logger(),
          "Change length code to " << request->type << " failed!");
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_ERROR(get_logger(), "Change code type timeout!");
  response->status = false;
  response->message= "TIMEOUT";

  return;
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
          if (resp.isValid) {
            gain_     = resp.gain;
            exposure_ = resp.exposure;
            pwm_      = resp.pwm;
            corrosion_= resp.corrosion;
            flag_read_param_.store(1);
          } else {
            RCLCPP_ERROR(this->get_logger(), "Read Parameter Failed!");
            flag_read_param_.store(-1);
          }
          continue;
        } else if (buffer[8] == uint8_t(Command::CHANGE_PARAMETER)) {
          ChangeParameter::Response resp(response);
          if (resp.isValid) {
            RCLCPP_INFO(get_logger(), "Change Parameter Success");
            flag_change_param_.store(1);
          } else {
            RCLCPP_ERROR(get_logger(), "Change Parameter Failed!");
            flag_change_param_.store(-1);
          }
          continue;
        } else if (buffer[8] == uint8_t(Command::SET_CODE_LENGTH)) {
          SetCodeLength::Response resp(response);
          if (resp.isValid) {
            RCLCPP_INFO(get_logger(), "Set Length Success");
            flag_change_code_length_.store(1);
          } else {
            RCLCPP_ERROR(get_logger(), "Set Length Failed!");
            flag_change_code_length_.store(-1);
          }
          continue;
        } else if (buffer[8] == uint8_t(Command::SET_TYPE_CODE)) {
          SetTypeCode::Response resp(response);
          if (resp.isValid) {
            if (resp.isValid) {
              RCLCPP_INFO(get_logger(), "Set Code Type Success");
              flag_change_code_type_.store(1);
            } else {
              RCLCPP_ERROR(get_logger(), "Change Code Type Failed!");
              flag_change_code_type_.store(-1);
            }
          }
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