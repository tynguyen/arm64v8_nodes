#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.h>


//#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("Get a new image frame!"); 
  //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
  cv::waitKey(30);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("hires/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
