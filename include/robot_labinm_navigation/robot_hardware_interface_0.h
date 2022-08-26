#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <rospy_tutorials/Floats.h>
#include <angles/angles.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <odrive_ros_control/od_dual_control.h>
#include <odrive_ros_control/od_control.h>

//#include <mobile_robot_autonomous_navigation/i2c_ros.h>

class ROBOTHardwareInterface : public hardware_interface::RobotHW 
{
	public:
        ROBOTHardwareInterface(ros::NodeHandle& nh);
        ~ROBOTHardwareInterface();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void callbackLeftWheelDeltaAngle(const std_msgs::Int32::ConstPtr& msg);
        void callbackRightWheelDeltaAngle(const std_msgs::Int32::ConstPtr& msg);
        void write(ros::Duration elapsed_time);
        ros::Publisher pub;
        ros::ServiceClient client;
        rospy_tutorials::Floats joints_pub;
        //three_dof_planar_manipulator::Floats_array joint_read;
        
    protected:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;

        joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;
        
        std::string joint_name_[2]={"left_wheel_joint","right_wheel_joint"};  
        double joint_position_[2];
        double joint_velocity_[2];
        double joint_effort_[2];
        double joint_velocity_command_[2];


	
	double left_motor_pos=0,right_motor_pos=0;
        int left_prev_cmd=0, right_prev_cmd=0;
        //i2c_ros::I2C left_motor= i2c_ros::I2C(0, 0x08);
        //i2c_ros::I2C right_motor= i2c_ros::I2C(1, 0x09);


        ros::NodeHandle nh_;
        ros::Timer non_realtime_loop_;
        ros::Duration elapsed_time_;
        ros::Publisher pub_left_wheel_speed = nh_.advertise<std_msgs::Float32>("left_wheel_speed", 10);
        ros::Publisher pub_right_wheel_speed = nh_.advertise<std_msgs::Float32>("right_wheel_speed", 10);
        ros::Publisher pub_odrive_dual_control = nh_.advertise<odrive_ros_control::od_dual_control>("odrive_dual_control", 10);
        ros::Subscriber sub_left_wheel_delta_angle; //= nh_.subscribe("angle_deltas", 10, &ROBOTHardwareInterface::callbackLeftWheelDeltaAngle, this);
        ros::Subscriber sub_right_wheel_delta_angle;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};


