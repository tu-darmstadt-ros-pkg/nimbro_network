// H264 encoder
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <memory>

#include <ros/ros.h>
#include <nimbro_cam_transport/encoder.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/CompressedImage.h>


ros::Publisher g_pub;
std::unique_ptr<nimbro_cam_transport::H264Encoder> g_enc;

void handleImage(const sensor_msgs::ImageConstPtr& img)
{
    auto message = g_enc->handleImage(img);
    if (message != nullptr)
        g_pub.publish(message);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cam_sender");


    ros::NodeHandle nh("~");

    g_enc = std::make_unique<nimbro_cam_transport::H264Encoder>(nh);

    image_transport::ImageTransport it(nh);

    g_pub = nh.advertise<sensor_msgs::CompressedImage>("encoded", 1);

    image_transport::Subscriber sub = it.subscribe("image", 1, &handleImage);

    ros::spin();

    return 0;
}
