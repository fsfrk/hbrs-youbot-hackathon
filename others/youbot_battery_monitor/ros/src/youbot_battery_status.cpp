#include <ros/ros.h>
#include <hbrs_msgs/PowerState.h>

#include <stdio.h> // standard input / output functions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <iostream>
#include <cstdlib>

enum voltagesource { battery1 = 0x04, battery2 = 0x05, powersupply = 0x0c };

int open_port(std::string port)
{
	int fd; // file description for the serial port

	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

	if (fd == -1) // if open is unsucessful
	{
		throw("unable to open port: " + port);
	}
	else
	{
		fcntl(fd, F_SETFL, 0);
		printf("port is open.\n");
	}

	return (fd);
}

int configure_port(int fd) // configure the port
{
	struct termios port_settings; // structure to store the port settings in

	tcgetattr(fd, &port_settings);

	cfsetispeed(&port_settings, B0); // set baud rates
	cfsetospeed(&port_settings, B0);

	port_settings.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode...

	//port_settings.c_cflag |= CRTSCTS; //Enable Hardware Flow Control
	port_settings.c_cflag &= ~CRTSCTS; //Disable Hardware Flow Control

	port_settings.c_cflag &= ~PARENB; // set no parity, stop bits, data bits
	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= CS8;

	port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //Choosing Raw Input
	port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); //disable software flow control
	port_settings.c_oflag &= ~OPOST; //Choosing Raw Output
	port_settings.c_cc[VMIN] = 0;
	port_settings.c_cc[VTIME] = 10; /* set raw input, 1 second timeout */

	tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port
	return (fd);

}

//return the voltage in [Volt]
double getVoltage(int fd, voltagesource source)
{
	unsigned char send_bytes[1];
	send_bytes[0] = source;

	write(fd, send_bytes, 1); //Send data

	const int readsize = 20;
	char read_bytes[readsize] =
	{ 0 };

	int nobytesread = 0;
	nobytesread = read(fd, read_bytes, readsize);
	read_bytes[nobytesread - 1] = 0; //delete the last tow character \CR+\LF
	read_bytes[nobytesread - 2] = 0;

	std::string value(read_bytes);

	return (double) atoi(value.c_str()) / 1000;
}



int main(int argc, char* argv[])
{
	ros::init(argc, argv, "raw_youbot_battery_monitor");

	int fd = 0;

	bool ros_node_initialized = false;
	ros::NodeHandle* p_nh = NULL;
	ros::Publisher pub_battery_status;
	hbrs_msgs::PowerState pwr_state;

	pwr_state.device_name = "youbot_battery";

	try
	{
		//int fd = open_port(argv[1]);
		//configure_port(fd);

	} catch (std::string &e) { std::cout << "Error: " << e << std::endl; }

	while(ros::ok())
	{
		sleep(2);

		//TODO: check out how to calculate the percentage
		pwr_state.battery_voltage = 24.2; //getVoltage(fd, battery1) + getVoltage(fd, battery2);
		pwr_state.battery_percentage = 100;
		pwr_state.power_supply_voltage = 0.0; //getVoltage(fd, powersupply);

		if(pwr_state.power_supply_voltage > 0.0)
			pwr_state.external_power_connected = true;
		else
			pwr_state.external_power_connected = false;

		if(pwr_state.battery_percentage <= 10)
		{
			//TODO: do here the system beep
			std::cout << "beep" << std::endl;
		}

		if(!ros_node_initialized)
		{
			std::cout << "check master" << std::endl;

			if(!ros::master::check())
				continue;

			std::cout << "do init" << std::endl;

			p_nh = new ros::NodeHandle();

			pub_battery_status.shutdown();
			pub_battery_status = p_nh->advertise<hbrs_msgs::PowerState>("/battery_status", 1);

			ros_node_initialized = true;
		}
		else
		{
			if(!ros::master::check())
			{
				//pub_battery_status.shutdown();
				delete p_nh;
				ros_node_initialized = false;
			}

			std::cout << "pub" << std::endl;

			if(pwr_state.battery_percentage <= 10)
				ROS_ERROR_STREAM("Critical battery level on device <<" << pwr_state.device_name << ">>: " << pwr_state.battery_percentage << "&");

			pub_battery_status.publish(pwr_state);
		}
	}

	close(fd);

	return (0);
}

