#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <angles/angles.h>
#include <std_msgs/Float32.h>
#include <odrive_ros_control/od_dual_control.h>
#include <odrive_ros_control/od_dual_control.h>
#include <odrive_ros_control/od_dual_supervision.h>


class ROBOTHardwareInterface : public hardware_interface::RobotHW 
{
    public:
        ROBOTHardwareInterface(ros::NodeHandle& nh);
        ~ROBOTHardwareInterface();
        void init();
        void update(const ros::TimerEvent& e);
        void read(const ros::Duration &period);
        void write(ros::Duration elapsed_time);        
        void callbackWheelDeltaAngles(const odrive_ros_control::od_dual_supervision msg);        
        
    protected:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;

        joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;
        
        std::string joint_name_[4]={"front_left_wheel_joint","front_right_wheel_joint","rear_left_wheel_joint","rear_right_wheel_joint"};        
        double joint_position_[4] = {0.0};
        double joint_velocity_[4] = {0.0};
        double joint_effort_[4] = {0.0};
        double joint_velocity_command_[4] = {0.0};
        double wheel_angle[4] = {0.0};
        double wheel_pos[4] = {0.0};
        double wheel_pos_before_start[4] = {0.0};
        double wheel_vel[4] = {0.0};
        bool isStarted = false;
               	
	double front_left_motor_pos=0, front_right_motor_pos=0, rear_left_motor_pos=0, rear_right_motor_pos=0;
        int front_left_prev_cmd=0, front_right_prev_cmd=0, rear_left_prev_cmd=0, rear_right_prev_cmd=0;

        ros::NodeHandle nh_;
        ros::Timer non_realtime_loop_;
        ros::Duration elapsed_time_;
        
        ros::Publisher pub_odrive_dual_control = nh_.advertise<odrive_ros_control::od_dual_control>("odrive_dual_control", 1);        
        ros::Subscriber sub_odrive_dual_supervision;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};


