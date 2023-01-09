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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <px4_msgs/msg/gripper_engage_status.hpp>

using std::placeholders::_1;

class GripperSubscriber : public rclcpp::Node
{
public:
  GripperSubscriber()
  : Node("gripper_subscriber")
  {
    subscription_ = this->create_subscription<px4_msgs::msg::GripperEngageStatus>(
      "GripperEngageStatus_PubSubTopic", 10, std::bind(&GripperSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const px4_msgs::msg::GripperEngageStatus::SharedPtr msg) const
  {
    std::cout << msg->status << std::endl;
  }
  rclcpp::Subscription<px4_msgs::msg::GripperEngageStatus>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperSubscriber>());
  rclcpp::shutdown();
  return 0;
}
