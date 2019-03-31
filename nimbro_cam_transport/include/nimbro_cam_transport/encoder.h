#ifndef NIMBRO_CAM_TRANSPORT_ENCODER_H
#define NIMBRO_CAM_TRANSPORT_ENCODER_H

#include <memory>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <ros/ros.h>
#include <nimbro_cam_transport/H264PublisherConfig.h>

namespace nimbro_cam_transport
{

class H264EncoderPIMPL;

class H264Encoder
{
public:
  explicit H264Encoder(const ros::NodeHandle& node);
  virtual ~H264Encoder();

  void configCb(nimbro_cam_transport::H264PublisherConfig& config, uint32_t level);

  sensor_msgs::CompressedImagePtr handleImage(const sensor_msgs::ImageConstPtr& img);

protected:
  void reloadEncoder();

private:
  std::unique_ptr<H264EncoderPIMPL> data;
};

}

#endif //NIMBRO_CAM_TRANSPORT_ENCODER_H
