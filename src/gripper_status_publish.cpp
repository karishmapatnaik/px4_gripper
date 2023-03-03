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
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/gripper_engage_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using namespace std::chrono_literals;

using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class GripperPublisher : public rclcpp::Node
{
public:
  GripperPublisher()
  : Node("gripper_publisher"), count_(0)
  {
    // publisher initialize
    publisher_ = this->create_publisher<px4_msgs::msg::GripperEngageStatus>("/fmu/gripper_engage_status/in", 10);

    // timesync subscription
    timesync_subscription_ = this->create_subscription<px4_msgs::msg::Timesync>("/fmu/timesync/out", 10,
      [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
        timestamp_.store(msg->timestamp);
      });
      
    // vehicle odometry subscription
    odom_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "fmu/vehicle_odometry/out", 10, std::bind(&GripperPublisher::topic_callback, this, _1)); 

    timer_ = this->create_wall_timer(
      20ms, std::bind(&GripperPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = px4_msgs::msg::GripperEngageStatus();

      // message.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
      if (flag_reach_target && !flag_set) {
      	reach_time = time(0);
      	flag_set = true;
      }
      
      current_time = time(0); 
      wait_time = difftime(current_time,reach_time);
	
      if ((flag_set) && (wait_time < user_defined_wait_time)){
      	message.timestamp = timestamp_.load();
	message.status = true;
	publisher_->publish(message);
      }
      else {
        message.timestamp = timestamp_.load();
      	message.status = false;
	publisher_->publish(message);
      }
  }
  
  void topic_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) 
  {
    error = pow(((msg->x - target_pos[0])*(msg->x - target_pos[0])) + ((msg->y - target_pos[1])*(msg->y - target_pos[1])) + ((msg->z - target_pos[2])*(msg->z - target_pos[2])),0.5);
    std::cout<<error<<std::endl;
    std::cout<<flag_reach_target<<std::endl;
    if ((abs(error) < 0.5) && (!flag_reach_target)) {
    		flag_reach_target = true;
    }
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::GripperEngageStatus>::SharedPtr publisher_;
  rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_subscription_;

  std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
  
  bool flag_reach_target = false;
  bool flag_set = false;
  
  // std::vector<float> target_pos{-0.006, 0.09, -0.28};
  std::vector<float> target_pos{0.0, 0.0, -0.75};
  float error = 0.0;
  float wait_time = 0.0;
  time_t reach_time, current_time;
  float user_defined_wait_time = 10.0;

  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperPublisher>());
  rclcpp::shutdown();
  return 0;
}
