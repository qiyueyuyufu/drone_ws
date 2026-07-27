#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <regex>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CanmvUdpReceiver : public rclcpp::Node {
public:
  CanmvUdpReceiver() : Node("canmv_udp_receiver") {
    listen_host_ = declare_parameter<std::string>("listen_host", "0.0.0.0");
    listen_port_ = declare_parameter<int>("listen_port", 9000);

    raw_pub_ = create_publisher<std_msgs::msg::String>("canmv/raw_json", 10);
    qrcode_pub_ = create_publisher<std_msgs::msg::String>("canmv/qrcode_payload", 10);

    open_socket();

    timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&CanmvUdpReceiver::receive_once, this));
  }

  ~CanmvUdpReceiver() override {
    if (sock_fd_ >= 0) {
      close(sock_fd_);
    }
  }

private:
  void open_socket() {
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
      throw std::runtime_error("socket() failed: " + std::string(std::strerror(errno)));
    }

    int reuse = 1;
    if (setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
      throw std::runtime_error("setsockopt() failed: " + std::string(std::strerror(errno)));
    }

    sockaddr_in bind_addr {};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(static_cast<uint16_t>(listen_port_));

    if (inet_pton(AF_INET, listen_host_.c_str(), &bind_addr.sin_addr) != 1) {
      throw std::runtime_error("invalid listen_host: " + listen_host_);
    }

    if (bind(sock_fd_, reinterpret_cast<sockaddr *>(&bind_addr), sizeof(bind_addr)) < 0) {
      throw std::runtime_error("bind() failed: " + std::string(std::strerror(errno)));
    }

    const int flags = fcntl(sock_fd_, F_GETFL, 0);
    if (flags < 0 || fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
      throw std::runtime_error("fcntl(O_NONBLOCK) failed: " + std::string(std::strerror(errno)));
    }

    RCLCPP_INFO(get_logger(), "waiting udp on %s:%d", listen_host_.c_str(), listen_port_);
    RCLCPP_INFO(get_logger(), "publishing raw json to /canmv/raw_json");
    RCLCPP_INFO(get_logger(), "publishing QR payload to /canmv/qrcode_payload when present");
  }

  void receive_once() {
    while (true) {
      char buffer[4096];
      sockaddr_in sender_addr {};
      socklen_t sender_len = sizeof(sender_addr);

      const ssize_t n = recvfrom(
          sock_fd_,
          buffer,
          sizeof(buffer) - 1,
          0,
          reinterpret_cast<sockaddr *>(&sender_addr),
          &sender_len);

      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          return;
        }
        RCLCPP_WARN(get_logger(), "recvfrom() failed: %s", std::strerror(errno));
        return;
      }

      buffer[n] = '\0';
      const std::string text(buffer);
      const std::string sender_ip = inet_ntoa(sender_addr.sin_addr);
      const int sender_port = ntohs(sender_addr.sin_port);

      RCLCPP_INFO(get_logger(), "from %s:%d %s", sender_ip.c_str(), sender_port, text.c_str());

      std_msgs::msg::String raw_msg;
      raw_msg.data = text;
      raw_pub_->publish(raw_msg);

      publish_qrcode_payload_if_present(text);
    }
  }

  void publish_qrcode_payload_if_present(const std::string & text) {
    if (text.find("\"type\":\"qrcode\"") == std::string::npos &&
        text.find("\"type\": \"qrcode\"") == std::string::npos) {
      return;
    }

    std::smatch match;
    const std::regex payload_regex("\"payload\"\\s*:\\s*\"([^\"]*)\"");
    if (!std::regex_search(text, match, payload_regex) || match.size() < 2) {
      return;
    }

    std_msgs::msg::String payload_msg;
    payload_msg.data = match[1].str();
    qrcode_pub_->publish(payload_msg);
    RCLCPP_INFO(get_logger(), "qrcode payload: %s", payload_msg.data.c_str());
  }

  int sock_fd_ {-1};
  int listen_port_ {9000};
  std::string listen_host_ {"0.0.0.0"};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qrcode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<CanmvUdpReceiver>());
  } catch (const std::exception & e) {
    std::fprintf(stderr, "canmv_udp_receiver error: %s\n", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
