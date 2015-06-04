// H264 receiver
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/ros.h>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

#include <image_transport/image_transport.h>

#include <sensor_msgs/CompressedImage.h>

ros::Publisher g_pub;
AVCodecContext* g_codec = 0;
SwsContext* g_sws = 0;

void handleImage(const sensor_msgs::CompressedImageConstPtr& img)
{
	//ROS_INFO("got data: %lu", img->data.size());

	AVPacket packet;
	av_init_packet(&packet);
	packet.data = const_cast<uint8_t*>(img->data.data());
	packet.size = img->data.size();
	packet.pts = AV_NOPTS_VALUE;
	packet.dts = AV_NOPTS_VALUE;

	AVFrame frame;
	memset(&frame, 0, sizeof(frame));

	int gotPicture;

	if(avcodec_decode_video2(g_codec, &frame, &gotPicture, &packet) < 0)
	{
		//ROS_ERROR("Could not decode frame");
		return;
	}

	if(gotPicture)
	{
		sensor_msgs::ImagePtr img(new sensor_msgs::Image);

		img->encoding = "rgb8";
		img->data.resize(frame.width * frame.height * 3);
		img->step = frame.width * 3;
		img->width = frame.width;
		img->height = frame.height;
		img->header.frame_id = "cam";
		img->header.stamp = ros::Time::now(); // FIXME

		uint8_t* destData[1] = {img->data.data()};
		int linesize[1] = {(int)img->step};

		sws_scale(g_sws, frame.data, frame.linesize, 0, frame.height,
			destData, linesize);

		//ROS_DEBUG("frame");
		g_pub.publish(img);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cam_receiver");
	
	ros::NodeHandle nh("~");

	avcodec_register_all();

	AVCodec* decoder = avcodec_find_decoder(AV_CODEC_ID_H264);
	if(!decoder)
		throw std::runtime_error("H264 decoding not supported in this build of ffmpeg");

	g_codec = avcodec_alloc_context3(decoder);

	g_codec->flags2 |= CODEC_FLAG2_SHOW_ALL;

	if(avcodec_open2(g_codec, decoder, 0) != 0)
		throw std::runtime_error("Could not open decoder");

	const int WIDTH = 640;
	const int HEIGHT = 480;
	g_sws = sws_getContext(WIDTH, HEIGHT, AV_PIX_FMT_YUV420P, WIDTH, HEIGHT,
	                       AV_PIX_FMT_RGB24, 0, 0, 0, 0);

	g_pub = nh.advertise<sensor_msgs::Image>("image", 1);
	
	ros::Subscriber sub = nh.subscribe("encoded", 5, &handleImage);
	
	ros::spin();
	
	return 0;
}
