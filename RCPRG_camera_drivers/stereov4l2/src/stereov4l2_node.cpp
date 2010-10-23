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
	std::string dev_l, dev_r, left_url, right_url, frame_id_l, frame_id_r;
	sensor_msgs::Image image_l, image_r;
	sensor_msgs::CameraInfo cam_info_l, cam_info_r;

	ros::init(argc, argv, "stereov4l2_node");
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
	nh.param<int>("left/input", input_l, 1);
	nh.param<int>("right/input", input_r, 0);
	nh.param<std::string>("left/device", dev_l, "/dev/video0");
	nh.param<std::string>("right/device", dev_r, "/dev/video1");
  nh.param<std::string>("left/frame_id", frame_id_l, "camera_l");
  nh.param<std::string>("right/frame_id", frame_id_r, "camera_r");

 	nh.param<std::string>("left/camera_info_url", left_url, "/tmp/left.yaml");
	nh.param<std::string>("right/camera_info_url", right_url, "/tmp/right.yaml");

	cinfo_l.loadCameraInfo(left_url);
	cinfo_r.loadCameraInfo(right_url);

	ROS_INFO("Opening device : %s", dev_l.c_str());
	Camera cam_l(dev_l.c_str(), width, height);
	Camera cam_r(dev_r.c_str(), width, height);

	cam_l.setInput(input_l);
	cam_r.setInput(input_r);

	image_l.header.frame_id = frame_id_l;
	image_l.height = cam_l.height;
	image_l.width = cam_l.width;
	image_l.encoding = sensor_msgs::image_encodings::BGR8;

	image_r.header.frame_id = frame_id_r;
	image_r.height = cam_l.height;
	image_r.width = cam_l.width;
	image_r.encoding = sensor_msgs::image_encodings::BGR8;

  cam_info_l = cinfo_l.getCameraInfo();
	cam_info_r = cinfo_r.getCameraInfo();

  cam_info_r.header.frame_id = image_r.header.frame_id;
  cam_info_l.header.frame_id = image_l.header.frame_id;


	while (node.ok()) {
		unsigned char* ptr_l = cam_l.Update();
		unsigned char* ptr_r = cam_r.Update();

		cam_info_l.header.stamp = cam_info_r.header.stamp = image_r.header.stamp = image_l.header.stamp = ros::Time::now();

		int image_size = cam_l.width * cam_l.height * 3;
		image_l.step = cam_l.width * 3;
    image_l.data.resize(image_size);
    memcpy(&image_l.data[0], ptr_l, image_size);
    image_pub_l.publish(image_l, cam_info_l);
		
    image_size = cam_r.width * cam_r.height * 3;
		image_r.step = cam_r.width * 3;
    image_r.data.resize(image_size);
		memcpy(&image_r.data[0], ptr_r, image_size);
		image_pub_r.publish(image_r, cam_info_r);

		ros::spinOnce();
	}

	return 0;
}
