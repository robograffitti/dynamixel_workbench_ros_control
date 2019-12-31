#include <dynamixel_workbench_ros_control/dynamixel_hardware.h>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

void controlLoop(dynamixel_workbench_ros_control::DynamixelHardware &dynamixel,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{

  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  // dxl_hw.reportLoopDuration(elapsed); // not necessary
  dynamixel.read(); // read command
  cm.update(ros::Time::now(), elapsed);
  dynamixel.write(); // write command
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "dynamixel_workbench_ros_control");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 10.0);

  dynamixel_workbench_ros_control::DynamixelHardware dynamixel(nh, private_nh, control_frequency);
  controller_manager::ControllerManager cm(&dynamixel, nh);

  ros::CallbackQueue dynamixel_queue;
  ros::AsyncSpinner dynamixel_spinner(1, &dynamixel_queue);

  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer(
    ros::Duration(1 / control_frequency),
    boost::bind(controlLoop, boost::ref(dynamixel), boost::ref(cm), boost::ref(last_time)),
    &dynamixel_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  dynamixel_spinner.start();

  // Process remainder of ROS callbacks separately, mainly ControlManager related
  ros::spin();

  return 0;

}
