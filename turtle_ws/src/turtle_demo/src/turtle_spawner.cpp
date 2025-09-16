#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "turtlesim_ds/srv/spawn.hpp"
#include "turtlesim_ds/srv/teleport_absolute.hpp"

using namespace std::chrono_literals;

class TurtleSpawner : public rclcpp::Node
{
public:
  TurtleSpawner() : Node("turtle_tf2_frame_listener"),
    turtle_spawned_(false)
  {
    // Clients
    spawner_ =
      this->create_client<turtlesim_ds::srv::Spawn>("spawn");
    teleport_client_ =
      this->create_client<turtlesim_ds::srv::TeleportAbsolute>("turtle1/teleport_absolute");

    // Call on_timer function every second
    timer_ = this->create_wall_timer(
      1s, std::bind(&TurtleSpawner::on_timer, this));
  }

  bool turtle_spawned_;
  rclcpp::Client<turtlesim_ds::srv::TeleportAbsolute>::SharedPtr teleport_client_{nullptr};

private:
  void on_timer() {
    if (!turtle_spawned_) {
      // Check if the service is ready
      if (spawner_->service_is_ready()) {
        // Initialize request with turtle name and coordinates
        // Note that x, y and theta are defined as doubles in turtlesim_ds/srv/Spawn
        auto request = std::make_shared<turtlesim_ds::srv::Spawn::Request>();
        request->x = 5.5;
        request->y = 5.5;
        request->theta = 0.0;
        request->name = "turtle2";
        request->img_index = 5;

        // Call request
        using ServiceResponseFuture =
          rclcpp::Client<turtlesim_ds::srv::Spawn>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
            auto result = future.get();
            if (!strcmp(result->name.c_str(), "turtle2") == 0) {
              RCLCPP_ERROR(this->get_logger(), "Service callback result mismatch");
          };
        };
        auto result = spawner_->async_send_request(request, response_received_callback);
        turtle_spawned_ = true;
      } else {
        RCLCPP_INFO(this->get_logger(), "Service is not ready");
      }
    }
  }

  rclcpp::Client<turtlesim_ds::srv::Spawn>::SharedPtr spawner_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

int main(int argc, char * argv[])
{
  const float radius = 4.0;

  rclcpp::init(argc, argv);
  auto spawner = std::make_shared<TurtleSpawner>();

  // Set random seed with time and namespace
  std::string ns = spawner->get_namespace();
  srand(time(NULL) + std::hash<std::string>{}(ns));

  // Choose a random starting position on the circle
  const float start_angle = (rand() % 360) * M_PI / 180.0;
  auto teleport_req = std::make_shared<turtlesim_ds::srv::TeleportAbsolute::Request>();
  teleport_req->x = 5.5 + radius * cos(start_angle);
  teleport_req->y = 5.5 + radius * sin(start_angle);
  teleport_req->theta = start_angle + M_PI / 2.0;

  // Call teleport service
  RCLCPP_INFO(spawner->get_logger(), "Waiting for teleport service...");
  if (!spawner->teleport_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(spawner->get_logger(), "Timed out waiting for service.");
      return 1;
  }
  RCLCPP_INFO(spawner->get_logger(), "Sending teleport request...");
  auto teleport_future = spawner->teleport_client_->async_send_request(teleport_req);
  auto teleport_status = rclcpp::spin_until_future_complete(spawner, teleport_future, std::chrono::milliseconds(10));

  while (!spawner->turtle_spawned_) {
    rclcpp::spin_some(spawner);
  }
  rclcpp::shutdown();
  return 0;
}