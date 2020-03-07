#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/String.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandLongRequest.h>
#include <mavros_msgs/Mavlink.h>

std_msgs::String message;


void callback(const std_msgs::String msg)
{
	message.data = msg.data;
}
//
// void setServo(int index, float value) {
// 	index=1;
// 	value=1900; //for now
//   // thruster values should be between 1100 and 1900 microseconds (us)
//   // values less than 1500 us are backwards; values more than are forwards
//   int pulse_width = (value + 1) * 400 + 1100;
//
//   // send mavros command message
//   // http://docs.ros.org/api/mavros/html/srv/CommandLong.html
//   // CMD_DO_SET_SERVO: https://pixhawk.ethz.ch/mavlink/
//   ROS_INFO("setServo(%d,%f->%d)", index, value, pulse_width);
//   mavros_msgs::CommandLong srv;
//   srv.request.command = mavros_msgs::CommandLongRequest::CMD_DO_SET_SERVO;
//   srv.request.param1 = index + 1; // servos are 1-indexed here
//   srv.request.param2 = pulse_width;
//   bool result = command_client.call(srv);
// }


int main(int argc, char **argv)	{

    ros::init(argc, argv, "servo_control");

    ros::NodeHandle nh;

    ros::Subscriber pub_message = nh.subscribe<std_msgs::String>
            ("controle", 10, callback);
    /*Acima, instanciamos um "publisher" a fim de publicar a posição da aeronave, bem como "clients" apropriedades, de forma a dar o comando "arm".*/
		ros::Publisher actuator_publisher = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/target_actuator_control", 1000);
    //the setpoint publishing rate MUST be faster than 2Hz
		mavros_msgs::Mavlink mavlink_msg;
		mavlink_msg.len = 2;
    mavlink_msg.sysid = 1;
    mavlink_msg.compid = 1;
    mavlink_msg.msgid = 0;
    std::vector<long unsigned int> payload64(7, 0);
    mavlink_msg.payload64 = payload64;
    ros::Rate rate(2.0);

		mavros_msgs::ActuatorControl servocon;
		float value = 0.9;
		while (ros::ok()) {
			servocon.group_mix = 3;
			//servocon.header.frame_id = '';
			for (int i=0; i < 8; i++)
				servocon.controls[i] = value;
			actuator_publisher.publish(servocon);

			// ros::ServiceClient command_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
			// int index=1;
			// float value=1900; //for now
		  // // thruster values should be between 1100 and 1900 microseconds (us)
		  // // values less than 1500 us are backwards; values more than are forwards
		  // int pulse_width = (value + 1) * 400 + 1100;
			//
		  // // send mavros command message
		  // // http://docs.ros.org/api/mavros/html/srv/CommandLong.html
		  // // CMD_DO_SET_SERVO: https://pixhawk.ethz.ch/mavlink/
		  // //ROS_INFO("setServo(%d,%f->%d)", index, value, pulse_width);
		  // mavros_msgs::CommandLong srv;
		  // srv.request.command = 184; //mavros_msgs::CommandLong::CMD_DO_SET_SERVO;//
			// //srv.request.command = mavros_msgs::CommandLong::MAV_CMD_DO_MOUNT_CONTROL;
			// srv.request.param1 = index + 1; // servos are 1-indexed here
		  // srv.request.param2 = pulse_width;
			// while (ros::ok()) {
		  // bool result = command_client.call(srv);
			// //ROS_INFO(result);
			ros::spinOnce();
			rate.sleep();
	}
	return 0;
}
