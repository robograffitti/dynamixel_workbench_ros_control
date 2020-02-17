/*******************************************************************************
 * Copyright 2018 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

/* Authors: Taehun Lim (Darby) */
/* Editors: Yoshimaru Tanaka */

#ifndef DYNAMIXEL_HARWARE_H
#define DYNAMIXEL_HARWARE_H

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <dynamixel_workbench_msgs/DynamixelStateList.h>

// #include <dynamixel_workbench_controllers/dynamixel_workbench_controllers.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

namespace dynamixel_workbench_ros_control
{

class DynamixelHardware : public hardware_interface::RobotHW
{
private:
  struct ItemValue
  {
    std::string item_name;
    int32_t value;
  };

  struct WayPoint
  {
    double position;
    double velocity;
    double acceleration;
  };

  struct Joint
  {
    double position;
    // double position_offset; // only necessary for position
    double velocity;
    double current;
    double effort;
    double position_command;
    double velocity_command;
    double effort_command;

    Joint() :
      position(0.0), velocity(0.0), current(0.0), effort(0.0), position_command(0.0), velocity_command(0.0), effort_command(0.0)
    { }
  };
  std::vector<Joint> joints_;

  // Parameters
  // bool is_moving_;
  bool is_first_;

  // ROS node handle
  ros::NodeHandle nh_, private_nh_;

  // ROS Control interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;

  std::map<std::string, uint32_t> dynamixel_;
  std::map<std::string, const ControlItem*> control_items_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  // dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list_;
  // sensor_msgs::JointState joint_state_msg_;
  std::vector<WayPoint> pre_goal_; // WayPoint is declared as an original struct

  void initializeDynamixelHardware();
  void initializeJoints();
  void registerControlInterfaces();

public:
  DynamixelHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq);
  ~DynamixelHardware();

  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool getDynamixelsInfo(const std::string yaml_file);
  bool loadDynamixels(void);
  bool initDynamixels(void);
  bool initControlItems(void);
  bool initSDKHandlers(void);
  bool getPresentPosition(std::vector<std::string> dxl_name);

  void read();
  void write();

};

} // namespace dynamixel_workbench_ros_control
#endif // DYNAMIXEL_HARDWARE_H
