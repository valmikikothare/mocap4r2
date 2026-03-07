// Copyright 2019 Intelligent Robotics Lab
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
//
// Author: David Vargas Frutos <david.vargas@urjc.es>
// Author: Jose Miguel Guerrero Hernandez <josemiguel.guerrero@urjc.es>

#include "mocap4r2_marker_viz/mocap4r2_marker_viz_node.hpp"

#include <string>
#include <random>

using std::placeholders::_1;
using namespace std::chrono_literals;

MarkerVisualizer::MarkerVisualizer() : Node("marker_visualizer")
{
  declare_parameter<double>("marker_scale_x", 0.014f);
  declare_parameter<double>("marker_scale_y", 0.014f);
  declare_parameter<double>("marker_scale_z", 0.014f);
  declare_parameter<float>("marker_lifetime", 0.01f);
  declare_parameter<std::string>("namespace", "mocap4r2_markers");
  declare_parameter<std::string>("mocap4r2_system", "optitrack");
  declare_parameter<std::vector<std::string>>("marker_topics", { "markers" });
  declare_parameter<std::vector<std::string>>("rb_topics", { "rigid_bodies" });

  get_parameter<double>("marker_scale_x", marker_scale_.x);
  get_parameter<double>("marker_scale_y", marker_scale_.y);
  get_parameter<double>("marker_scale_z", marker_scale_.z);
  get_parameter<float>("marker_lifetime", marker_lifetime_);
  get_parameter<std::string>("namespace", namespace_);
  get_parameter<std::string>("mocap4r2_system", mocap4r2_system_);
  get_parameter<std::vector<std::string>>("marker_topics", marker_topics_);
  get_parameter<std::vector<std::string>>("rb_topics", rb_topics_);

  unsigned seed = 0;
  std::mt19937 generator(seed);
  std::uniform_real_distribution<float> distribution(0.0f, 1.0f);

  for (const auto& topic : marker_topics_)
  {
    marker_publishers_[topic] = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic + "_viz", 1000);
    markers_subscriptions_[topic] = this->create_subscription<mocap4r2_msgs::msg::Markers>(
        topic, 1000, [this, topic](const mocap4r2_msgs::msg::Markers::SharedPtr msg) {
          MarkerVisualizer::marker_callback(topic, msg);
        });
    std_msgs::msg::ColorRGBA color;
    color.r = distribution(generator);
    color.g = distribution(generator);
    color.b = distribution(generator);
    color.a = 1.0;
    marker_colors_[topic] = color;
  }

  for (const auto& topic : rb_topics_)
  {
    rb_publishers_[topic] = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic + "_viz", 1000);
    rb_subscriptions_[topic] = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
        topic, 1000, [this, topic](const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) {
          MarkerVisualizer::rb_callback(topic, msg);
        });
    std_msgs::msg::ColorRGBA marker_color;
    marker_color.r = distribution(generator);
    marker_color.g = distribution(generator);
    marker_color.b = distribution(generator);
    marker_color.a = 1.0;
    marker_colors_[topic] = marker_color;
    std_msgs::msg::ColorRGBA rb_color;
    rb_color.r = distribution(generator);
    rb_color.g = distribution(generator);
    rb_color.b = distribution(generator);
    rb_color.a = 1.0;
    marker_colors_[topic] = rb_color;
  }
}

// This function change mocap axis to match with rviz axis
geometry_msgs::msg::Pose MarkerVisualizer::mocap2rviz(const geometry_msgs::msg::Pose mocap4r2_pose) const
{
  geometry_msgs::msg::Pose rviz_pose;
  if (mocap4r2_system_ == "optitrack")
  {
    rviz_pose = mocap4r2_pose;
  }
  else if (mocap4r2_system_ == "vicon")
  {
    // TO-DO:
    rviz_pose = mocap4r2_pose;
  }
  else if (mocap4r2_system_ == "qualisys")
  {
    // TO-DO:
    rviz_pose = mocap4r2_pose;
  }
  else
  {
    rviz_pose = mocap4r2_pose;
  }
  return rviz_pose;
}

void MarkerVisualizer::marker_callback(const std::string& topic, const mocap4r2_msgs::msg::Markers::SharedPtr msg) const
{
  auto publisher = marker_publishers_.at(topic);
  if (publisher->get_subscription_count() == 0)
  {
    return;
  }

  static int counter = 0;
  visualization_msgs::msg::MarkerArray visual_markers;
  for (const mocap4r2_msgs::msg::Marker& marker : msg->markers)
  {
    visual_markers.markers.push_back(marker2visual(topic, counter++, marker.translation, msg->header));
  }
  publisher->publish(visual_markers);
}

visualization_msgs::msg::Marker MarkerVisualizer::marker2visual(const std::string& topic, int index,
                                                                const geometry_msgs::msg::Point& translation,
                                                                const std_msgs::msg::Header& header) const
{
  visualization_msgs::msg::Marker viz_marker;
  viz_marker.header = header;
  viz_marker.ns = namespace_;
  viz_marker.color = marker_colors_.at(topic);
  viz_marker.id = index;
  viz_marker.type = visualization_msgs::msg::Marker::SPHERE;
  viz_marker.action = visualization_msgs::msg::Marker::ADD;
  // Change mocap system axis to rviz axis
  geometry_msgs::msg::Pose marker_pose;
  marker_pose.position = translation;
  marker_pose.orientation.x = 0.0f;
  marker_pose.orientation.y = 0.0f;
  marker_pose.orientation.z = 0.0f;
  marker_pose.orientation.w = 1.0f;
  viz_marker.pose = mocap2rviz(marker_pose);
  viz_marker.scale = marker_scale_;
  viz_marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
  return viz_marker;
}

void MarkerVisualizer::rb_callback(const std::string& topic, const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) const
{
  auto publisher = rb_publishers_.at(topic);
  if (publisher->get_subscription_count() == 0)
  {
    return;
  }

  static int counter_rb = 0;
  static int counter_markers_rb = 0;
  visualization_msgs::msg::MarkerArray visual_markers_rb;

  for (const mocap4r2_msgs::msg::RigidBody& rb : msg->rigidbodies)
  {
    visual_markers_rb.markers.push_back(rb2visual(topic, counter_rb++, rb.pose, msg->header));

    for (const mocap4r2_msgs::msg::Marker& marker : rb.markers)
    {
      visual_markers_rb.markers.push_back(marker2visual(topic, counter_markers_rb++, marker.translation, msg->header));
    }
  }

  publisher->publish(visual_markers_rb);
}

visualization_msgs::msg::Marker MarkerVisualizer::rb2visual(const std::string& topic, int index,
                                                            const geometry_msgs::msg::Pose& poserb,
                                                            const std_msgs::msg::Header& header) const
{
  visualization_msgs::msg::Marker viz_marker;
  viz_marker.header = header;
  viz_marker.ns = namespace_;
  viz_marker.color = rb_colors_.at(topic);
  viz_marker.id = index;
  viz_marker.type = visualization_msgs::msg::Marker::ARROW;
  viz_marker.action = visualization_msgs::msg::Marker::ADD;

  // Change mocap system axis to rviz axis
  viz_marker.pose = mocap2rviz(poserb);

  geometry_msgs::msg::Vector3 marker_scale_;
  marker_scale_.x = 0.5f;
  marker_scale_.y = 0.014f;
  marker_scale_.z = 0.014f;
  viz_marker.scale = marker_scale_;
  viz_marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
  return viz_marker;
}
