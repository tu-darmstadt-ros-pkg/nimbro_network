#ifndef NIMBRO_CAM_TRANSPORT_IMAGE_TRANSPORT_PUBLISHER_H
#define NIMBRO_CAM_TRANSPORT_IMAGE_TRANSPORT_PUBLISHER_H

#include <memory>

#include <image_transport/simple_publisher_plugin.h>
#include <sensor_msgs/CompressedImage.h>

#include <nimbro_cam_transport/encoder.h>

namespace nimbro_cam_transport
{

class ImageTransportPub : public image_transport::SimplePublisherPlugin<sensor_msgs::CompressedImage>
{
public:
  std::string getTransportName() const override;

protected:
  void advertiseImpl(ros::NodeHandle &nh,
                     const std::string &base_topic,
                     uint32_t queue_size,
                     const image_transport::SubscriberStatusCallback &user_connect_cb,
                     const image_transport::SubscriberStatusCallback &user_disconnect_cb,
                     const ros::VoidPtr &tracked_object,
                     bool latch) override;

  void publish(const sensor_msgs::Image &message,
               const PublishFn &publish_fn) const override;

  std::unique_ptr<H264Encoder> encoder;
};

}

#endif //NIMBRO_CAM_TRANSPORT_IMAGE_TRANSPORT_PUBLISHER_H
