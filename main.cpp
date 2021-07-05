#include <iostream>
#include "cmd_interface_linux.h"
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "trnet.h"
#include <unistd.h>
#include "rtrnet.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ld07");
	ros::NodeHandle nh; 
	std::string topic_name("LD07/LDLiDAR");
	ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>(topic_name, 1); /*create a ROS topic */
	CmdInterfaceLinux cmd_port;
	std::vector<std::pair<std::string, std::string>> device_list;
	std::string port_name;
	cmd_port.GetCmdDevices(device_list);
	RTRNet *lidar = new RTRNet();

	for (auto n : device_list)
	{
		std::cout << n.first << "    " << n.second << std::endl;
		if (strstr(n.second.c_str(), "CP2102"))
		{
			port_name = n.first;
		}
	}

	if (port_name.empty())
		std::cout << "CANNOT FIND LiDAR_LD07" << std::endl;

	if(cmd_port.Open(port_name))
	{
		std::cout << "FOUND LiDAR_LD07" << std::endl;
	}else
	{
		std::cout << "CANNOT FIND LiDAR_LD07" << std::endl;
		return (-1);
	}
	
	cmd_port.SetReadCallback([&lidar](const char *byte, size_t len) {
		lidar->UnpackData((uint8_t *)byte, len);
	});

	int error = 0;
	while (lidar->IsParametersReady() == false)
	{
		lidar->SendCmd(cmd_port, 0, PACK_CONFIG_ADDRESS);
		sleep(1);
		lidar->SendCmd(cmd_port, THIS_DEVICE_ADDREESS, PACK_GET_COE);
		sleep(1);
		lidar->SendCmd(cmd_port, THIS_DEVICE_ADDREESS, PACK_VIDEO_SIZE);
		error++;
		if (error > 2) /* Exit if the number of errors is more than 2*/
		{
			std::cout << "Error: GET LD07 PARAMETERS FAILED" << std::endl;
			return -1;
		}
	}
	std::cout << "get param successfull!!" << std::endl;
	lidar->SendCmd(cmd_port, THIS_DEVICE_ADDREESS, PACK_GET_DISTANCE);
	std::cout << "SEND PACK_GET_DISTANCE CMD" << std::endl;

	while (ros::ok())
	{
		if (lidar->IsFrameReady())
		{
			lidar_pub.publish(lidar->GetLaserScan());
			lidar->ResetFrameReady();
		}
	}

	return 0;
}
