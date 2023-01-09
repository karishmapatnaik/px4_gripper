// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/gripper_engage_status.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class GripperPublisher : public rclcpp::Node
{
public:
  GripperPublisher()
  : Node("gripper_publisher"), count_(0)
  {
    // publisher initialize
    publisher_ = this->create_publisher<px4_msgs::msg::GripperEngageStatus>("/GripperEngageStatus_PubSubTopic", 10);

    // timesync subscription
    timesync_subscription_ = this->create_subscription<px4_msgs::msg::Timesync>("/Timesync_PubSubTopic", 10,
      [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
        timestamp_.store(msg->timestamp);
      });

    timer_ = this->create_wall_timer(
      500ms, std::bind(&GripperPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = px4_msgs::msg::GripperEngageStatus();

      // message.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
      message.timestamp = timestamp_.load();

      message.status = true;

      publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::GripperEngageStatus>::SharedPtr publisher_;
  rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_subscription_;

  std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperPublisher>());
  rclcpp::shutdown();
  return 0;
}
