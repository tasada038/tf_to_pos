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

#include "tf_to_pos/pos_sub_component.hpp"


MarkerSubscriber::MarkerSubscriber()
    : Node("pos_sub") {
        marker_subscriber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/end_marker_array", 10, std::bind(&MarkerSubscriber::marker_callback, this, std::placeholders::_1)
        );
    }

void MarkerSubscriber::marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        for (const auto & marker : msg->markers) {
            RCLCPP_INFO(this->get_logger(), "Marker ID: %d, Position: [x: %f, y: %f, z: %f]",
                        marker.id, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
        }
}
