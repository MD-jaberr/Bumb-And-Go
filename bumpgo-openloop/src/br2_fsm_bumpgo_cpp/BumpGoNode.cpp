// Copyright 2021 Intelligent Robotics Lab
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

#include <utility>
#include "br2_fsm_bumpgo_cpp/BumpGoNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace br2_fsm_bumpgo_cpp
{

using namespace std::chrono_literals;
using std::placeholders::_1;

BumpGoNode::BumpGoNode()
: Node("bump_go"),
  state_(FORWARD)
{
  turning_duration_  = 0.0f;  // in seconds
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),
    std::bind(&BumpGoNode::scan_callback, this, _1));

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  timer_ = create_wall_timer(50ms, std::bind(&BumpGoNode::control_cycle, this));

  state_ts_ = now();
}


void
BumpGoNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void
BumpGoNode::control_cycle()
{
  // Do nothing until the first sensor read
  if (last_scan_ == nullptr) {
    return;
  }

  geometry_msgs::msg::Twist out_vel;

  switch (state_) {
    case FORWARD:
      out_vel.linear.x = SPEED_LINEAR;

      if (check_forward_2_stop()) {
        go_state(STOP);
      }
      else if (check_forward_2_back()) {
        go_state(BACK);
      }

      break;

    case BACK:
      out_vel.linear.x = -SPEED_LINEAR;

      if (check_back_2_turn()) {
        go_state(TURN);
      }

      break;

    case TURN:
      out_vel.angular.z = preferred_turn_dir_ * SPEED_ANGULAR;
    
      if (check_turn_2_forward()) {
        go_state(FORWARD);
      }
    
      break;
    
    

    case STOP:
      // No motion
      if (check_stop_2_forward()) {
        go_state(FORWARD);
      }
      break;
  }

  vel_pub_->publish(out_vel);
}


void
BumpGoNode::go_state(int new_state)
{
  state_ = new_state;
  state_ts_ = now();
}

bool
BumpGoNode::check_forward_2_back()
{
  if (!last_scan_) return false;

  const auto& ranges = last_scan_->ranges;
  float angle_min = last_scan_->angle_min;
  float angle_increment = last_scan_->angle_increment;

  int min_index = 196;
  int max_index = 470;

  float min_range_left = std::numeric_limits<float>::infinity();
  float min_range_right = std::numeric_limits<float>::infinity();

  for (int i = min_index; i <= max_index; ++i)
  {
    if (i < 0 || i >= static_cast<int>(ranges.size())) continue;

    float range = ranges[i];
    if (range >= last_scan_->range_min && range <= last_scan_->range_max)
    {
      if (i < 333) {  // right half of 196–470
        min_range_right = std::min(min_range_right, range);
      } else {
        min_range_left = std::min(min_range_left, range);
      }
    }
  }

  float min_range_all = std::min(min_range_left, min_range_right);

  // Only proceed if there's no obstacle ahead
  if (min_range_all >= OBSTACLE_DISTANCE) {
    return false;
  }

  
  float max_clear_range = 0.0;
  int best_index = 0;

  for (int i = 0; i < static_cast<int>(ranges.size()); ++i)
  {
    float range = ranges[i];
    if (range >= last_scan_->range_min && range <= last_scan_->range_max && std::isfinite(range))
    {
      if (range > max_clear_range)
      {
        max_clear_range = range;
        best_index = i;
      }
    }
  }

  float angle = angle_min + best_index * angle_increment;
  best_clear_angle_ = angle;
  preferred_turn_dir_ = angle >= 0.0 ? 1 : -1;
  turning_duration_ = std::abs(angle) / SPEED_ANGULAR;
  RCLCPP_INFO(this->get_logger(),
  "Obstacle detected. Turning %.1f° (%s) for %.2f s",
  best_clear_angle_ * 180.0 / M_PI,
  preferred_turn_dir_ > 0 ? "left" : "right",
  turning_duration_);

  return true;
}


bool
BumpGoNode::check_forward_2_stop()
{
  // Stop if no sensor readings for 1 second
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > SCAN_TIMEOUT;
}

bool
BumpGoNode::check_stop_2_forward()
{
  // Going forward if sensor readings are available
  // again
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < SCAN_TIMEOUT;
}

bool
BumpGoNode::check_back_2_turn()
{
  // Going back for 2 seconds
  return (now() - state_ts_) > BACKING_TIME;
}

bool
BumpGoNode::check_turn_2_forward()
{
  return (now() - state_ts_).seconds() > turning_duration_;

}

}  // namespace br2_fsm_bumpgo_cpp