#include <robot_labinm_navigation/robot_hardware_interface.h>


ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=20;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
	
    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);       
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {

}

void ROBOTHardwareInterface::init() {
	
	for(int i=0; i<4; i++)
	//for(int i=0; i<2; i++)
	{
	    // Create joint state interface
	    hardware_interface::JointStateHandle jointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
	    joint_state_interface_.registerHandle(jointStateHandle);

	    // Create velocity joint interface
	    hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
	    velocity_joint_interface_.registerHandle(jointVelocityHandle);

	     // Create Joint Limit interface   
	    joint_limits_interface::JointLimits limits;
	    joint_limits_interface::getJointLimits(joint_name_[i], nh_, limits);
	    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandle(jointVelocityHandle, limits);
	    velocityJointSaturationInterface.registerHandle(jointLimitsHandle);

	}
    
	// Register all joints interfaces    
	registerInterface(&joint_state_interface_);
	registerInterface(&velocity_joint_interface_);
	registerInterface(&velocityJointSaturationInterface);
	
	sub_odrive_dual_supervision = nh_.subscribe("axis_supervision", 1, &ROBOTHardwareInterface::callbackWheelDeltaAngles, this);
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    //read();
    read(elapsed_time_);
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::read(const ros::Duration &period) {
    
    joint_position_[0]=wheel_angle[0];    
    joint_velocity_[0]=wheel_vel[0];
        
    joint_position_[1]=wheel_angle[1];    
    joint_velocity_[1]=wheel_vel[1];
        
    joint_position_[2]=wheel_angle[2];    
    joint_velocity_[2]=wheel_vel[2];
        
    joint_position_[3]=wheel_angle[3];    
    joint_velocity_[3]=wheel_vel[3];    
}


void ROBOTHardwareInterface::callbackWheelDeltaAngles(const odrive_ros_control::od_dual_supervision msg)
{
        
    wheel_angle[0] = msg.Odrv1_sup.Ax0_pos_act*2.0*M_PI;
    wheel_angle[1] = msg.Odrv1_sup.Ax1_pos_act*2.0*M_PI;
    wheel_angle[2] = msg.Odrv2_sup.Ax0_pos_act*2.0*M_PI;
    wheel_angle[3] = msg.Odrv2_sup.Ax1_pos_act*2.0*M_PI;
        
    
    wheel_vel[0] = msg.Odrv1_sup.Ax0_vel_act*2.0*M_PI; //vel in radians/s
    wheel_vel[1] = msg.Odrv1_sup.Ax1_vel_act*2.0*M_PI;
    wheel_vel[2] = msg.Odrv2_sup.Ax0_vel_act*2.0*M_PI;
    wheel_vel[3] = msg.Odrv2_sup.Ax1_vel_act*2.0*M_PI;
            
    //std::cout<<wheel_vel[0]<<", "<<wheel_vel[1]<<std::endl;
    std::cout<<wheel_angle[0]<<", "<<wheel_angle[1]<<", "<<wheel_angle[2]<<", "<<wheel_angle[3]<<std::endl;    
}


void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
    
    velocityJointSaturationInterface.enforceLimits(elapsed_time);
        
    odrive_ros_control::od_dual_control msg_dual_control;    
    
    msg_dual_control.Odrive1.Vel_ax0 = joint_velocity_command_[0]/(2.0*M_PI); //en RPS
    msg_dual_control.Odrive1.Vel_ax1 = joint_velocity_command_[1]/(2.0*M_PI);
    msg_dual_control.Odrive2.Vel_ax0 = joint_velocity_command_[2]/(2.0*M_PI);
    msg_dual_control.Odrive2.Vel_ax1 = joint_velocity_command_[3]/(2.0*M_PI);        
    
    pub_odrive_dual_control.publish(msg_dual_control);    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mobile_robot_hardware_interface");
    
    //std::cout<<"HolaMundo0"<<std::endl;
    
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(4);  
    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);
    //spinner.start();
    spinner.spin();
    //ros::spin();
    return 0;
}
