#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleSensor : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifecycleSensor()
  : LifecycleNode("lifecycle_sensor"),
    gen_(rd_()),
    dist_(0.0, 100.0)
  {
    RCLCPP_INFO(this->get_logger(), "Lifecycle sensor node created");
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/sensor_data", 10);

    RCLCPP_INFO(this->get_logger(), "Sensor configured");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    publisher_->on_activate();

    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&LifecycleSensor::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Sensor activated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    timer_.reset();
    publisher_->on_deactivate();

    RCLCPP_INFO(this->get_logger(), "Sensor deactivated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    timer_.reset();
    publisher_.reset();

    RCLCPP_INFO(this->get_logger(), "Sensor cleaned up");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(this->get_logger(), "Sensor shutting down");
    return CallbackReturn::SUCCESS;
  }

private:
  void timer_callback()
  {
    if (publisher_->is_activated())
    {
      std_msgs::msg::Float64 msg;
      msg.data = dist_(gen_);

      RCLCPP_INFO(
        this->get_logger(),
        "Publishing sensor data: %.2f",
        msg.data);

      publisher_->publish(msg);
    }
  }

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<> dist_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<LifecycleSensor>()->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
