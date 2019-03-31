#include "nimbro_cam_transport/decoder.h"

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

namespace nimbro_cam_transport
{

struct H264DecoderPIMPL
{
  AVCodec* decoder = nullptr;
  AVCodecContext* codec = nullptr;
  SwsContext* sws = nullptr;
};

H264Decoder::H264Decoder()
{
    this->data = std::make_unique<H264DecoderPIMPL>();

    avcodec_register_all();
    av_log_set_level(AV_LOG_QUIET);

    this->data->decoder = avcodec_find_decoder(AV_CODEC_ID_H264);
    if(!this->data->decoder)
        throw std::runtime_error("H264 decoding not supported in this build of ffmpeg");

    this->data->codec = avcodec_alloc_context3(this->data->decoder);

    this->data->codec->flags |= CODEC_FLAG_LOW_DELAY;
    this->data->codec->flags2 |= CODEC_FLAG2_SHOW_ALL;

    this->data->codec->thread_type = 0;

    if(avcodec_open2(this->data->codec, this->data->decoder, nullptr) != 0)
        throw std::runtime_error("Could not open decoder");
}

H264Decoder::~H264Decoder()
{
    sws_freeContext(this->data->sws);
    this->data->sws = nullptr;
    if (this->data->codec != nullptr)
        avcodec_free_context(&this->data->codec);
}

sensor_msgs::ImagePtr H264Decoder::handleImage(const sensor_msgs::CompressedImageConstPtr &img)
{
    AVPacket packet;
    av_init_packet(&packet);
    packet.data = const_cast<uint8_t*>(img->data.data());
    packet.size = img->data.size();
    packet.pts = AV_NOPTS_VALUE;
    packet.dts = AV_NOPTS_VALUE;

    AVFrame frame;
    memset(&frame, 0, sizeof(frame));

    int gotPicture;

    if(avcodec_decode_video2(this->data->codec, &frame, &gotPicture, &packet) < 0)
    {
        return nullptr;
    }

    if(gotPicture)
    {
        this->data->sws = sws_getCachedContext(
                this->data->sws,
                frame.width, frame.height, AV_PIX_FMT_YUV420P,
                frame.width, frame.height, AV_PIX_FMT_RGB24,
                0, nullptr, nullptr, nullptr
        );

        sensor_msgs::ImagePtr result(new sensor_msgs::Image);

        result->encoding = "rgb8";
        result->data.resize(frame.width * frame.height * 3);
        result->step = frame.width * 3;
        result->width = frame.width;
        result->height = frame.height;
        result->header = img->header;

        uint8_t* destData[1] = {result->data.data()};
        int linesize[1] = {(int)result->step};

        sws_scale(this->data->sws, frame.data, frame.linesize, 0, frame.height,
                  destData, linesize);

        return result;
    }
}

}