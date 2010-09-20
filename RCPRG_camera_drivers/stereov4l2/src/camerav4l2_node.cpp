/*
 * camerav4l_node.cpp
 *
 *  Created on: 02-09-2010
 *      Author: konrad
 */
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include "libcam.h"

/** Main entry point */
int main(int argc, char **argv) {
	int height, width, input_l, input_r;
	std::string dev_l, dev_r, left_url, right_url;
	sensor_msgs::Image image;
	sensor_msgs::CameraInfo cam_info_l, cam_info_r;

	ros::init(argc, argv, "camerav4l2_node");
	ros::NodeHandle node;
	ros::NodeHandle nh("~");

	CameraInfoManager cinfo_l(ros::NodeHandle(node, "left"), "left_camera");
	CameraInfoManager cinfo_r(ros::NodeHandle(node, "right"), "right_camera");

	image_transport::ImageTransport it(node);
	image_transport::CameraPublisher image_pub_l = it.advertiseCamera(
			"left/image_raw", 1);
	image_transport::CameraPublisher image_pub_r = it.advertiseCamera(
			"right/image_raw", 1);

	nh.param<int>("width", width, 640);
	nh.param<int>("height", height, 480);
	nh.param<int>("input_l", input_l, 1);
	nh.param<int>("input_r", input_r, 0);
	nh.param<std::string>("device_l", dev_l, "/dev/video0");
	nh.param<std::string>("device_r", dev_r, "/dev/video1");

 	nh.param<std::string>("left/camera_info_url", left_url, "/tmp/left.yaml");
	nh.param<std::string>("right/camera_info_url", right_url, "/tmp/right.yaml");

	cinfo_l.loadCameraInfo(left_url);
	cinfo_r.loadCameraInfo(right_url);

	ROS_INFO("Opening device : %s", dev_l.c_str());
	Camera cam_l(dev_l.c_str(), width, height);
	Camera cam_r(dev_r.c_str(), width, height);

	cam_l.setInput(input_l);
	cam_r.setInput(input_r);

	image.header.frame_id = "camara";
	image.header.seq = 0;
	image.height = cam_l.height;
	image.width = cam_l.width;
	image.encoding = sensor_msgs::image_encodings::BGR8;


	while (node.ok()) {
		unsigned char* ptr_l = cam_l.Update();
		unsigned char* ptr_r = cam_r.Update();
		++image.header.seq;
		image.header.stamp = ros::Time::now();
		int image_size = cam_l.width * cam_l.height * 3;
		image.step = cam_l.width * 3;
        	image.data.resize(image_size);
        	memcpy(&image.data[0], ptr_l, image_size);

        	cam_info_l = cinfo_l.getCameraInfo();
		cam_info_r = cinfo_r.getCameraInfo();

       		cam_info_r.header.frame_id = cam_info_l.header.frame_id = image.header.frame_id;
        	cam_info_r.header.stamp = cam_info_l.header.stamp = image.header.stamp;

        	image_pub_l.publish(image, cam_info_l);
		
		memcpy(&image.data[0], ptr_r, image_size);
		image_pub_r.publish(image, cam_info_r);

		ros::spinOnce();
	}

	return 0;
}
