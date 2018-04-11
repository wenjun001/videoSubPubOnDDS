#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "transferVideo/src/videoPublisher.h"
#include "transferVideo/src/video.h"





videoPublisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {

    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("SubscriberView", frame);





    cv::Mat sourceMat = frame;
    size_t totalsize = sourceMat.step[sourceMat.dims - 1];
    const size_t rowsize = sourceMat.step[sourceMat.dims - 1] * sourceMat.size[sourceMat.dims - 1];
    size_t coordinates[sourceMat.dims - 1] = {0};
//        std::cout << "Image dimensions: ";
    for (int t = 0; t < sourceMat.dims; t++) {

      totalsize *= sourceMat.size[t];
//            std::cout << (t > 0 ? " X " : "") << sourceMat.size[t];
    }

    uint8_t *imagebuffer = new uint8_t[totalsize];
    size_t srcptr = 0, dptr = 0;
//        std::cout << std::endl;
//        std::cout << "Frame size is " << frame.dims << " frame.size:" << frame.size << "  frame.type():" << frame.type()
//                  << std::endl;
//        std::cout << "One pixel in image has " << sourceMat.step[sourceMat.dims - 1] << " bytes" << std::endl;
//        std::cout << "Copying data in blocks of " << rowsize << " bytes" << std::endl;
//        std::cout << "Total size is " << totalsize << " bytes" << std::endl;
    while (dptr < totalsize) {
      // we copy entire rows at once, so lowest iterator is always [dims-2]
      // this is legal since OpenCV does not use 1 dimensional matrices internally (a 1D matrix is a 2d matrix with only 1 row)
      std::memcpy(&imagebuffer[dptr], &(((uint8_t *) sourceMat.data)[srcptr]), rowsize);
      // destination matrix has no gaps so rows follow each other directly
      dptr += rowsize;
      // src matrix can have gaps so we need to calculate the address of the start of the next row the hard way
      // see *brief* text in opencv2/core/mat.hpp for address calculation
      coordinates[sourceMat.dims - 2]++;
      srcptr = 0;
      for (int t = sourceMat.dims - 2; t >= 0; t--) {
        if (coordinates[t] >= sourceMat.size[t]) {
          if (t == 0) break;
          coordinates[t] = 0;
          coordinates[t - 1]++;
        }
        srcptr += sourceMat.step[t] * coordinates[t];
      }
    }


  int size[2] = {480, 640};
    cv::Mat destination = cv::Mat(2, size, 16, (void *) imagebuffer);
    std::string text = "FROM ROS SUB";
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 2;
    int thickness = 2;
    int baseline;
    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
    cv::Point origin;
    origin.x = frame.cols / 2 - text_size.width / 2;
    origin.y = frame.rows / 2 + text_size.height / 2;
    cv::putText(destination, text, origin, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);

    imshow("duplicated", destination);

    Video video;
    std::copy(imagebuffer,imagebuffer+totalsize,video.message().begin());

      pub.run(video);

    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{


    pub.init();
    std::cout << "main";
  ros::init(argc, argv, "image_listener");

  cv::namedWindow("SubscriberView");
  cv::startWindowThread();
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/sensor_images/image", 1, imageCallback);

  ros::spin();
  cv::destroyWindow("SubscriberView");
}