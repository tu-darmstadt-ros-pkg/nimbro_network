#ifndef NIMBRO_CAM_TRANSPORT_DECODER_H
#define NIMBRO_CAM_TRANSPORT_DECODER_H

#include <memory>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

namespace nimbro_cam_transport
{

class H264DecoderPIMPL;

class H264Decoder
{
public:
  explicit H264Decoder();
  virtual ~H264Decoder();

  sensor_msgs::ImagePtr handleImage(const sensor_msgs::CompressedImageConstPtr& img);

private:
  std::unique_ptr<H264DecoderPIMPL> data;
};

}

#endif //NIMBRO_CAM_TRANSPORT_DECODER_H
