#ifndef NIMBRO_CAM_TRANSPORT_IMAGE_TRANSPORT_SUBSCRIBER_H
#define NIMBRO_CAM_TRANSPORT_IMAGE_TRANSPORT_SUBSCRIBER_H

#include "image_transport/simple_subscriber_plugin.h"
#include <sensor_msgs/CompressedImage.h>

#include <nimbro_cam_transport/decoder.h>

namespace nimbro_cam_transport
{
class ImageTransportSub : public image_transport::SimpleSubscriberPlugin<sensor_msgs::CompressedImage>
{
public:
  std::string getTransportName() const override;

protected:
  void internalCallback(const sensor_msgs::CompressedImageConstPtr &message,
                        const Callback &user_cb) override;

  H264Decoder decoder;
};
}

#endif //NIMBRO_CAM_TRANSPORT_IMAGE_TRANSPORT_SUBSCRIBER_H
