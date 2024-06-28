//-----------------------------------------------------------------------------------
// MIT License

// Copyright (c) 2024 Takumi Asada

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//-----------------------------------------------------------------------------------

#include "tf_to_pos/tf_to_pos_component.hpp"


TFTrajectory::TFTrajectory()
    : Node("tf_to_pos"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {
        // Set target frames to subscribe
        target_frames_ = {
          "lf_foot_link", "lh_foot_link", "rf_foot_link", "rh_foot_link"
          // "link_end_Ltop", "link_end_Rtop", "link_end_Lmid", "link_end_Rmid", "link_end_Lbtm", "link_end_Rbtm"
        };
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/end_marker_array", 1
        );
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&TFTrajectory::timer_callback, this)
        );
    }

void TFTrajectory::timer_callback() {
  visualization_msgs::msg::MarkerArray marker_array;
  for (size_t i = 0; i < target_frames_.size(); ++i) {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer_.lookupTransform("base_link", target_frames_[i], tf2::TimePointZero);
      RCLCPP_INFO(this->get_logger(), "[%zu] Translation: x=%f, y=%f, z=%f", i,
        transformStamped.transform.translation.x,
        transformStamped.transform.translation.y,
        transformStamped.transform.translation.z);

      add_marker_to_array(
        marker_array,
        i,
        transformStamped.transform.translation.x,
        transformStamped.transform.translation.y,
        transformStamped.transform.translation.z
      );
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "[%zu] %s", i, ex.what());
    }
  }
  marker_publisher_->publish(marker_array);
}

void TFTrajectory::add_marker_to_array(visualization_msgs::msg::MarkerArray& marker_array, size_t index, double x, double y, double z) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "tf_trajectory";
        marker.id = index;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.025;
        marker.scale.y = 0.025;
        marker.scale.z = 0.025;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);
    }
