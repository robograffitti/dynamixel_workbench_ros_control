#include "dynamixel_workbench_ros_control/dynamixel_hardware.h"
// #include <boost/assign/list_of.hpp>

namespace dynamixel_workbench_ros_control
{
  DynamixelHardware::DynamixelHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq)
    :
    nh_(nh),
    private_nh_(private_nh),
    joints_(0),
    is_first_(true)
    // is_moving_(false) // necessary?
  {
    initializeDynamixelHardware();
    registerControlInterfaces();
  }

  DynamixelHardware::~DynamixelHardware() {}

  bool DynamixelHardware::initWorkbench(const std::string port_name, const uint32_t baud_rate)
  {
    bool result = false;
    const char* log;

    result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
    if (result == false)
      {
        ROS_ERROR("%s", log);
      }

    return result;
  }

  bool DynamixelHardware::getDynamixelsInfo(const std::string yaml_file)
  {
    YAML::Node dynamixel;
    dynamixel = YAML::LoadFile(yaml_file.c_str());

    if (dynamixel == NULL)
      return false;

    for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
      {
        std::string name = it_file->first.as<std::string>();
        if (name.size() == 0)
          {
            continue;
          }

        YAML::Node item = dynamixel[name];
        for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
          {
            std::string item_name = it_item->first.as<std::string>();
            int32_t value = it_item->second.as<int32_t>();

            if (item_name == "ID")
              dynamixel_[name] = value;

            ItemValue item_value = {item_name, value};
            std::pair<std::string, ItemValue> info(name, item_value);

            dynamixel_info_.push_back(info);
          }
      }

    return true;
  }

  bool DynamixelHardware::loadDynamixels(void)
  {
    bool result = false;
    const char* log;

    for (auto const& dxl:dynamixel_)
      {
        uint16_t model_number = 0;
        result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
        if (result == false)
          {
            ROS_ERROR("%s", log);
            ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
            return result;
          }
        else
          {
            ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
          }
      }

    return result;
  }

  bool DynamixelHardware::initDynamixels(void)
  {
    const char* log;

    for (auto const& dxl:dynamixel_)
      {
        dxl_wb_->torqueOff((uint8_t)dxl.second);

        for (auto const& info:dynamixel_info_)
          {
            if (dxl.first == info.first)
              {
                if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
                  {
                    bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
                    if (result == false)
                      {
                        ROS_ERROR("%s", log);
                        ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
                        return false;
                      }
                  }
              }
          }

        // dxl_wb_->torqueOn((uint8_t)dxl.second); // set torqueOn from yaml file?
      }

    // Torque On after setting up all servo
    for (auto const& dxl:dynamixel_)
      dxl_wb_->torqueOn((uint8_t)dxl.second);

    return true;
  }

  bool DynamixelHardware::initControlItems(void)
  {
    bool result = false;
    const char* log = NULL;

    auto it = dynamixel_.begin();

    const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
    if (goal_position == NULL) return false;

    const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
    if (goal_velocity == NULL)  goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
    if (goal_velocity == NULL)  return false;

    const ControlItem *present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
    if (present_position == NULL) return false;

    const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
    if (present_velocity == NULL)  present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
    if (present_velocity == NULL) return false;

    const ControlItem *present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
    if (present_current == NULL)  present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
    if (present_current == NULL) return false;

    control_items_["Goal_Position"] = goal_position;
    control_items_["Goal_Velocity"] = goal_velocity;

    control_items_["Present_Position"] = present_position;
    control_items_["Present_Velocity"] = present_velocity;
    control_items_["Present_Current"] = present_current;

    return true;
  }

  bool DynamixelHardware::initSDKHandlers(void)
  {
    bool result = false;
    const char* log = NULL;

    auto it = dynamixel_.begin();

    result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);
    if (result == false)
      {
        ROS_ERROR("%s", log);
        return result;
      }
    else
      {
        ROS_INFO("%s", log);
      }

    result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
    if (result == false)
      {
        ROS_ERROR("%s", log);
        return result;
      }
    else
      {
        ROS_INFO("%s", log);
      }

    if (dxl_wb_->getProtocolVersion() == 2.0f)
      {
        uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

        /* As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.*/
        // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
        uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length+2;

        result = dxl_wb_->addSyncReadHandler(start_address,
                                             read_length,
                                             &log);
        if (result == false)
          {
            ROS_ERROR("%s", log);
            return result;
          }
      }

    return result;
  }

  bool DynamixelHardware::getPresentPosition(std::vector<std::string> dxl_name)
  {
    bool result = false;
    const char* log = NULL;

    int32_t get_position[dxl_name.size()];

    uint8_t id_array[dxl_name.size()];
    uint8_t id_cnt = 0;

    for (auto const& name:dxl_name)
      id_array[id_cnt++] = dynamixel_[name];

    if (dxl_wb_->getProtocolVersion() == 2.0f)
      {
        result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                   id_array,
                                   dxl_name.size(),
                                   &log);
        if (result == false)
          {
            ROS_ERROR("%s", log);
          }

        WayPoint wp;

        result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                          id_array,
                                          id_cnt,
                                          control_items_["Present_Position"]->address,
                                          control_items_["Present_Position"]->data_length,
                                          get_position,
                                          &log);

        if (result == false)
          {
            ROS_ERROR("%s", log);
          }
        else
          {
            for(uint8_t index = 0; index < id_cnt; index++)
              {
                wp.position = dxl_wb_->convertValue2Radian(id_array[index], get_position[index]);
                pre_goal_.push_back(wp);
              }
          }
      }
    else if (dxl_wb_->getProtocolVersion() == 1.0f)
      {
        WayPoint wp;
        uint32_t read_position;
        for (auto const& dxl:dynamixel_)
          {
            result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                           control_items_["Present_Position"]->address,
                                           control_items_["Present_Position"]->data_length,
                                           &read_position,
                                           &log);
            if (result == false)
              {
                ROS_ERROR("%s", log);
              }

            wp.position = dxl_wb_->convertValue2Radian((uint8_t)dxl.second, read_position);
            pre_goal_.push_back(wp);
          }
      }

    return result;
  }

  void DynamixelHardware::initializeDynamixelHardware()
  {
    // init workbench class
    dxl_wb_ = new DynamixelWorkbench;

    std::string port_name;
    private_nh_.param<std::string>("port", port_name, "/dev/ttyUSB0");

    std::string baud_rate;
    private_nh_.param<std::string>("baud_rate", baud_rate, "57600");

    std::string yaml_file;
    private_nh_.param<std::string>("dynamixel_info", yaml_file, "");

    bool result = false;

    result = initWorkbench(port_name, std::stoi(baud_rate));
    if (result == false)
      {
        ROS_ERROR("Please check USB port name");
        return;
      }

    result = getDynamixelsInfo(yaml_file);
    if (result == false)
      {
        ROS_ERROR("Please check YAML file");
        return;
      }

    result = loadDynamixels();
    if (result == false)
      {
        ROS_ERROR("Please check Dynamixel ID or BaudRate");
        return;
      }

    result = initDynamixels();
    if (result == false)
      {
        ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
        return;
      }

    result = initControlItems();
    if (result == false)
      {
        ROS_ERROR("Please check control items");
        return;
      }

    result = initSDKHandlers();
    if (result == false)
      {
        ROS_ERROR("Failed to set Dynamixel SDK Handler");
        return;
      }

  }

  void DynamixelHardware::registerControlInterfaces()
  {
    // ascending order
    // std::vector<std::pair<uint32_t, std::string>> dynamixel(dynamixel_.size());
    // int count = 0;
    // for (auto iter = dynamixel.begin(); iter != dynamixel.end(); iter++)
    //   {
    //     for (auto iter = dynamixel_.begin(); iter != dynamixel_.end(); iter++)
    //       {
    //         if (count + 1 == iter->second)
    //           {
    //             std::pair<uint32_t, std::string> pair(iter->second, iter->first);
    //             dynamixel[count] = pair; // range : 0, 1, 2, 3, 4
    //             // ROS_INFO("joint_name : %s, servo ID: %d", iter->second.c_str(), iter->first);
    //             break;
    //           }
    //       }
    //     count++;
    //   }

    // resige vector
    // joints_.resize(dynamixel.size());
    joints_.resize(dynamixel_.size());

    for (auto iter = dynamixel_.begin(); iter != dynamixel_.end(); iter++)
      {
        // initialize joint vector
        Joint joint;
        joints_[iter->second - 1] = joint;
        ROS_INFO("joint_name : %s, servo ID: %d", iter->first.c_str(), iter->second);

        hardware_interface::JointStateHandle joint_state_handle(iter->first.c_str(),
                                                                &joints_[iter->second - 1].position,
                                                                &joints_[iter->second - 1].velocity,
                                                                &joints_[iter->second - 1].effort);
        joint_state_interface_.registerHandle(joint_state_handle);

        hardware_interface::JointHandle position_joint_handle(joint_state_handle, &joints_[iter->second - 1].position_command);
        position_joint_interface_.registerHandle(position_joint_handle);
        hardware_interface::JointHandle velocity_joint_handle(joint_state_handle, &joints_[iter->second - 1].velocity_command);
        velocity_joint_interface_.registerHandle(velocity_joint_handle);
        hardware_interface::JointHandle effort_joint_handle(joint_state_handle, &joints_[iter->second - 1].effort_command);
        effort_joint_interface_.registerHandle(effort_joint_handle);

      }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&effort_joint_interface_);
  }

  void DynamixelHardware::read()
  {
    bool result = false;
    const char* log = NULL;

    int32_t get_current[dynamixel_.size()];
    int32_t get_velocity[dynamixel_.size()];
    int32_t get_position[dynamixel_.size()];

    uint8_t id_array[dynamixel_.size()];
    uint8_t id_cnt = 0;

    for (auto const& dxl:dynamixel_)
      id_array[id_cnt++] = (uint8_t)dxl.second;

    if (dxl_wb_->getProtocolVersion() == 2.0f)
      {
        result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                   id_array,
                                   dynamixel_.size(),
                                   &log);
        if (result == false)
          {
            ROS_ERROR("%s", log);
          }

        result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                          id_array,
                                          id_cnt,
                                          control_items_["Present_Current"]->address,
                                          control_items_["Present_Current"]->data_length,
                                          get_current,
                                          &log);
        if (result == false)
          {
            ROS_ERROR("%s", log);
          }

        result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                          id_array,
                                          id_cnt,
                                          control_items_["Present_Velocity"]->address,
                                          control_items_["Present_Velocity"]->data_length,
                                          get_velocity,
                                          &log);
        if (result == false)
          {
            ROS_ERROR("%s", log);
          }

        result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                          id_array,
                                          id_cnt,
                                          control_items_["Present_Position"]->address,
                                          control_items_["Present_Position"]->data_length,
                                          get_position,
                                          &log);
        if (result == false)
          {
            ROS_ERROR("%s", log);
          }

        for(uint8_t index = 0; index < id_cnt; index++)
          {
            // index 0, 1, 2, 3, 4
            // joints_ 1, 2, 3, 4, 5
            // id_array 1, 5, 4, 2, 3
            joints_[id_array[index]-1].position = dxl_wb_->convertValue2Radian((uint8_t)id_array[index], (int32_t)get_position[index]);
            joints_[id_array[index]-1].velocity = dxl_wb_->convertValue2Velocity((uint8_t)id_array[index], (int32_t)get_velocity[index]);
            // joints_[index].current = get_current[index];

            if (strcmp(dxl_wb_->getModelName((uint8_t)id_array[index]), "XL-320") == 0)
              joints_[id_array[index]-1].effort = dxl_wb_->convertValue2Load((int16_t)get_current[index]);
            else
              joints_[id_array[index]-1].effort = dxl_wb_->convertValue2Current((int16_t)get_current[index]);

            if (is_first_ == true)
              joints_[id_array[index]-1].position_command = joints_[id_array[index]-1].position;

          }
      }
    else if(dxl_wb_->getProtocolVersion() == 1.0f)
      {
        uint16_t length_of_data = control_items_["Present_Position"]->data_length +
          control_items_["Present_Velocity"]->data_length +
          control_items_["Present_Current"]->data_length;
        uint32_t get_all_data[length_of_data];
        uint8_t dxl_cnt = 0;
        for (auto const& dxl:dynamixel_)
          {
            result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                           control_items_["Present_Position"]->address,
                                           length_of_data,
                                           get_all_data,
                                           &log);
            if (result == false)
              {
                ROS_ERROR("%s", log);
              }

            // ID required to get model name
            joints_[(uint8_t)dxl.second-1].position = dxl_wb_->convertValue2Radian((uint8_t)dxl.second, (int32_t)DXL_MAKEWORD(get_all_data[0], get_all_data[1]));
            joints_[(uint8_t)dxl.second-1].velocity = dxl_wb_->convertValue2Velocity((uint8_t)dxl.second, (int32_t)DXL_MAKEWORD(get_all_data[2], get_all_data[3]));
            joints_[(uint8_t)dxl.second-1].effort = dxl_wb_->convertValue2Load(DXL_MAKEWORD(get_all_data[4], get_all_data[5]));

            dxl_cnt++;
          }
      }
  }

  void DynamixelHardware::write()
  {
    bool result = false;
    const char* log = NULL;

    uint8_t id_array[dynamixel_.size()];
    uint8_t id_cnt = 0;

    int32_t dynamixel_position[dynamixel_.size()];

    for (auto const& dxl:dynamixel_)
      id_array[id_cnt++] = (uint8_t)dxl.second;

    for (uint8_t index = 0; index < id_cnt; index++)
      dynamixel_position[index] = dxl_wb_->convertRadian2Value(id_array[index], joints_[id_array[index]-1].position_command);

    // if (is_first_ == true)
    //   {
    //     ROS_INFO("First write.");
    //     for (uint8_t index = 0; index < id_cnt; index++)
    //       dynamixel_position[index] = dxl_wb_->convertRadian2Value(id_array[index], joints_[id_array[index]-1].position);

    //     is_first_ = false;
    //   }
    // else
    //   {
    //     for (uint8_t index = 0; index < id_cnt; index++)
    //       dynamixel_position[index] = dxl_wb_->convertRadian2Value(id_array[index], joints_[id_array[index]-1].position_command);
    //   }

    result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, id_array, id_cnt, dynamixel_position, 1, &log);
    if (result == false)
      {
        ROS_ERROR("%s", log);
      }
  }

} // namespace dynamixel_workbench_ros_control
