#include "nimbro_cam_transport/image_transport_publisher.h"
#include <pluginlib/class_list_macros.h>

void nimbro_cam_transport::ImageTransportPub::advertiseImpl(
        ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
        const image_transport::SubscriberStatusCallback &user_connect_cb,
        const image_transport::SubscriberStatusCallback &user_disconnect_cb,
        const ros::VoidPtr &tracked_object, bool latch)
{
    SimplePublisherPlugin::advertiseImpl(
        nh, base_topic, queue_size, user_connect_cb, user_disconnect_cb,
        tracked_object, latch);

    this->encoder = std::make_unique<H264Encoder>(this->nh());
}

void nimbro_cam_transport::ImageTransportPub::publish(
        const sensor_msgs::Image &message, const PublishFn &publish_fn) const
{
    const auto img = boost::make_shared<sensor_msgs::Image>(message);
    const auto compressed = this->encoder->handleImage(img);
    if (compressed != nullptr)
        publish_fn(*compressed.get());
}

std::string nimbro_cam_transport::ImageTransportPub::getTransportName() const
{
    return "h264";
}

PLUGINLIB_EXPORT_CLASS(nimbro_cam_transport::ImageTransportPub, image_transport::PublisherPlugin)