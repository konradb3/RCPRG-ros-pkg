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
	int height, width;
	std::string dev;
	sensor_msgs::Image image;
	sensor_msgs::CameraInfo cam_info;

	ros::init(argc, argv, "camerav4l2_node");
	ros::NodeHandle node("camera");

	CameraInfoManager cinfo(node);

	image_transport::ImageTransport it(node);
	image_transport::CameraPublisher image_pub = it.advertiseCamera(
			"image_raw", 1);

	node.param<int>("width", width, 640);
	node.param<int>("height", height, 480);
	node.param<std::string>("device", dev, "/dev/video0");

	Camera cam(dev.c_str(), width, height);

	image.header.frame_id = "camara";
	image.header.seq = 0;
	image.height = cam.height;
	image.width = cam.width;
	image.encoding = sensor_msgs::image_encodings::RGB8;


	while (node.ok()) {
		unsigned char* ptr = cam.Update();
		++image.header.seq;
		image.header.stamp = ros::Time::now();
		int image_size = cam.width * cam.height * 3;
		image.step = cam.width * 3;
        image.data.resize(image_size);
        memcpy(&image.data[0], ptr, image_size);

        cam_info = cinfo.getCameraInfo();

        cam_info.header.frame_id = image.header.frame_id;
        cam_info.header.stamp = image.header.stamp;

        image_pub.publish(image, cam_info);

		ros::spinOnce();
	}

	return 0;
}
