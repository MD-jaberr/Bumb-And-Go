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
  turning_in_progress_ = false;
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

  if (new_state == TURN) {
    turning_in_progress_ = true;
  } else {
    turning_in_progress_ = false;
  }
  RCLCPP_INFO(this->get_logger(), "STATE CHANGE → %s",
  (new_state == FORWARD ? "FORWARD" :
   new_state == BACK    ? "BACK" :
   new_state == TURN    ? "TURN" :
   new_state == STOP    ? "STOP" : "UNKNOWN"));

}


bool
BumpGoNode::check_forward_2_back()
{
  if (!last_scan_) return false;

  const auto& ranges = last_scan_->ranges;
  float angle_min = last_scan_->angle_min;
  float angle_increment = last_scan_->angle_increment;

  // 1. Check for obstacle in ±45° cone
  float min_range_left = std::numeric_limits<float>::infinity();
  float min_range_right = std::numeric_limits<float>::infinity();

  for (int i = 196; i <= 470; ++i)
  {
    if (i < 0 || i >= static_cast<int>(ranges.size())) continue;

    float range = ranges[i];
    if (range >= last_scan_->range_min && range <= last_scan_->range_max)
    {
      if (i < 333) {
        min_range_right = std::min(min_range_right, range);
      } else {
        min_range_left = std::min(min_range_left, range);
      }
    }
  }

  float min_range_all = std::min(min_range_left, min_range_right);
  bool obstacle_in_center = min_range_all < OBSTACLE_DISTANCE;

  // 2. Find best clear direction (max range)
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

  // 3. If obstacle in ±45° and best direction is also inside → ignore it and reselect
  if (obstacle_in_center && best_index >= 196 && best_index <= 470)
  {
    max_clear_range = 0.0;

    for (int i = 0; i <= 195; ++i)
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

    for (int i = 471; i < static_cast<int>(ranges.size()); ++i)
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
  }

  float angle = angle_min + best_index * angle_increment;
  best_clear_angle_ = angle;
  preferred_turn_dir_ = angle >= 0.0 ? 1 : -1;

  if (obstacle_in_center)
  {
    RCLCPP_INFO(this->get_logger(),
      "Obstacle in center. Turning to index %d (%.1f°)",
      best_index, angle * 180.0 / M_PI);
    return true;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(),
      "No obstacle. Continuing FORWARD toward index %d (%.1f°)",
      best_index, angle * 180.0 / M_PI);
    return false;
  }
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
  if (!last_scan_) return false;

  const auto& ranges = last_scan_->ranges;
  int max_index = 0;
  float max_range = 0.0;

  for (int i = 0; i < static_cast<int>(ranges.size()); ++i)
  {
    float range = ranges[i];
    if (std::isfinite(range) &&
        range >= last_scan_->range_min &&
        range <= last_scan_->range_max)
    {
      if (range > max_range)
      {
        max_range = range;
        max_index = i;
      }
    }
  }

  RCLCPP_INFO(this->get_logger(),
    "Max index: %d", max_index);

  if (std::abs(max_index - 333) <= 3) {
    return true;
  }

  return false;  // ✅ Fallback return
}
}