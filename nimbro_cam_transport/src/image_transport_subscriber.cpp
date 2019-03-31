#include "nimbro_cam_transport/image_transport_subscriber.h"

#include <pluginlib/class_list_macros.h>

namespace nimbro_cam_transport
{

std::string ImageTransportSub::getTransportName() const
{
    return "h264";
}

void ImageTransportSub::internalCallback(
        const sensor_msgs::CompressedImageConstPtr &message,
        const image_transport::SubscriberPlugin::Callback &user_cb)
{
    const auto img = this->decoder.handleImage(message);
    if (img != nullptr)
        user_cb(img);
}
}

PLUGINLIB_EXPORT_CLASS(nimbro_cam_transport::ImageTransportSub, image_transport::SubscriberPlugin)