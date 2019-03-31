// H264 receiver
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>

#include <nimbro_cam_transport/decoder.h>

ros::Publisher g_pub;
nimbro_cam_transport::H264Decoder g_dec;

void handleImage(const sensor_msgs::CompressedImageConstPtr& img)
{
    auto message = g_dec.handleImage(img);
    if (message != nullptr)
        g_pub.publish(message);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cam_receiver");

    ros::NodeHandle nh("~");

    g_pub = nh.advertise<sensor_msgs::Image>("image", 1);

    ros::Subscriber sub = nh.subscribe("encoded", 5, &handleImage);

    ros::spin();

    return 0;
}
