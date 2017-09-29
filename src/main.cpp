#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.h"
#include "compressed_depth_image_transport/codec.h"
#include "compressed_depth_image_transport/compression_common.h"

#include "sensor_msgs/CompressedImage.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/CameraInfo.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "ctime"
#include "time.h"

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

image_transport::Publisher rgb_pub;
image_transport::Publisher depth_pub;

namespace compressed_depth_image_transport
{
sensor_msgs::Image::Ptr decodeCompressedImage(const sensor_msgs::CompressedImage& message)
{
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

        // Copy message header
        cv_ptr->header = message.header;

        // Decode color/mono image
        try {
            cv_ptr->image = cv::imdecode(cv::Mat(message.data), CV_LOAD_IMAGE_UNCHANGED);

            // Assign image encoding string
            const size_t split_pos = message.format.find(';');
            if (split_pos==string::npos) {
                // Older version of compressed_image_transport does not signal image format
                switch (cv_ptr->image.channels())
                {
                    case 1:
                        cv_ptr->encoding = enc::MONO8;
                        break;
                    case 3:
                        cv_ptr->encoding = enc::BGR8;
                        break;
                    default:
                        ROS_ERROR("Unsupported number of channels: %i", cv_ptr->image.channels());
                        break;
                }
            } else {
                string image_encoding = message.format.substr(0, message.format.find(';'));
                cv_ptr->encoding = image_encoding;

                if ( enc::isColor(image_encoding)) {
                    // Revert color transformation
                    if ((image_encoding == enc::BGR8) || (image_encoding == enc::BGR16)) {
                        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);
                    }

                    if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16)) {
                        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGRA);
                    }

                    if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16)) {
                        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2RGBA);
                    }
                }
            }
        } catch (cv::Exception& e) {
            ROS_ERROR("%s", e.what());
        }

        size_t rows = cv_ptr->image.rows;
        size_t cols = cv_ptr->image.cols;

        if ((rows > 0) && (cols > 0)) {
            // Publish message to user callback
            return cv_ptr->toImageMsg();
        }
        else
          return sensor_msgs::Image::Ptr();
}
sensor_msgs::Image::Ptr decodeCompressedDepthImage(const sensor_msgs::CompressedImage& message)
{

	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
 
  // Copy message header
  cv_ptr->header = message.header;
 
  // Assign image encoding
  std::string image_encoding = message.format.substr(0, message.format.find(';'));
  cv_ptr->encoding = image_encoding;
 
  // Decode message data
  if (message.data.size() > sizeof(ConfigHeader))
  {
 
  // Read compression type from stream
  ConfigHeader compressionConfig;
  memcpy(&compressionConfig, &message.data[0], sizeof(compressionConfig));
 
  // Get compressed image data
  const std::vector<uint8_t> imageData(message.data.begin() + sizeof(compressionConfig), message.data.end());
 
  // Depth map decoding
  float depthQuantA, depthQuantB;
  // Read quantization parameters
  depthQuantA = compressionConfig.depthParam[0];
  depthQuantB = compressionConfig.depthParam[1];
 
  if (enc::bitDepth(image_encoding) == 32)
  {
    cv::Mat decompressed;
    try
    {
      // Decode image data
      decompressed = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
    }
    catch (cv::Exception& e)
    {
      ROS_ERROR("%s", e.what());
      return sensor_msgs::Image::Ptr();
    }
 
    size_t rows = decompressed.rows;
    size_t cols = decompressed.cols;
 
    if ((rows > 0) && (cols > 0))
    {
      cv_ptr->image = Mat(rows, cols, CV_32FC1);
 
      // Depth conversion
      MatIterator_<float> itDepthImg = cv_ptr->image.begin<float>(),
                          itDepthImg_end = cv_ptr->image.end<float>();
      MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(),
                                        itInvDepthImg_end = decompressed.end<unsigned short>();
 
      for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
      {
        // check for NaN & max depth
        if (*itInvDepthImg)
        {
          *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
        }
        else
        {
          *itDepthImg = std::numeric_limits<float>::quiet_NaN();
        }
      }
 
      // Publish message to user callback
      return cv_ptr->toImageMsg();
    }
  }
  else
  {
    // Decode raw image
    try
    {
      cv_ptr->image = cv::imdecode(imageData, CV_LOAD_IMAGE_UNCHANGED);
    }
    catch (cv::Exception& e)
    {
      ROS_ERROR("%s", e.what());
      return sensor_msgs::Image::Ptr();
    }
 
    size_t rows = cv_ptr->image.rows;
    size_t cols = cv_ptr->image.cols;
 
    if ((rows > 0) && (cols > 0))
    {
      // Publish message to user callback
      return cv_ptr->toImageMsg();
    }
  }
 }
  return sensor_msgs::Image::Ptr();
}

};

void callback(const sensor_msgs::CompressedImageConstPtr& kinect2color_msg,
              const sensor_msgs::CompressedImageConstPtr& kinect2depth_msg)
{
  ROS_INFO("Enter Publish");
  sensor_msgs::Image::Ptr depth = compressed_depth_image_transport::decodeCompressedDepthImage(*kinect2depth_msg);   
  sensor_msgs::Image::Ptr rgb = compressed_depth_image_transport::decodeCompressedImage(*kinect2color_msg);   
  rgb_pub.publish(*rgb); 
  depth_pub.publish(*depth); 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "message_filter_node");
  ros::Time::init();
  ros::NodeHandle nh;
  image_transport::ImageTransport it_(nh);
  
  ROS_INFO("start message filter");

  rgb_pub = it_.advertise("/vslam/rgb/image_raw", 1);
  depth_pub = it_.advertise("/vslam/depth_registered/image_raw", 1);
  time_t t=std::time(0);
  struct tm * now = std::localtime( & t );
  string file_name;

  message_filters::Subscriber<sensor_msgs::CompressedImage> kinect2color_sub(nh,"/data_throttled_image/compressed" , 1);//订阅全向视觉Topic
  message_filters::Subscriber<sensor_msgs::CompressedImage> kinect2depth_sub(nh,"/data_throttled_image_depth/compressedDepth" , 1);//订阅Kinect的Topic

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage,sensor_msgs::CompressedImage> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),
                                                   kinect2color_sub,
                                                   kinect2depth_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  ros::spin();
  return 0;
}


