#include<iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class ImageConverter
{
public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber rgb_image_sub_;
    image_transport::Subscriber depth_image_sub_;
    string rgb_image_topic;
    string depth_image_topic;
    string rgb_window ;
    string depth_window ;

    ImageConverter() : it_(nh_) {
        ros::NodeHandle nh_p("~");
        nh_p.param("rgb_image_topic", rgb_image_topic, std::string("/camera/rgb/image_raw"));
        nh_p.param("depth_image_topic", depth_image_topic, std::string("/camera/depth/image_raw"));
        rgb_image_sub_ = it_.subscribe(rgb_image_topic, 1, &ImageConverter::rgb_imageCb, this);
        depth_image_sub_ = it_.subscribe(depth_image_topic, 1, &ImageConverter::depth_imageCb, this);
        rgb_window = "rgb";
        depth_window = "depth";
        //cv::namedWindow("rgb");
        //cv::namedWindow("depth");
    }

    ~ImageConverter() {
        cv::destroyWindow(rgb_window);
        cv::destroyWindow(depth_window);
    }

    void rgb_imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            //cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("rgb cv_bridge exception: %s", e.what());
            return;
        }

        cv::imshow(rgb_window, cv_ptr->image);
        cv::waitKey(20);
    }

    void depth_imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("depth cv_bridge exception: %s", e.what());
            return;
        }

        cv::imshow(depth_window, cv_ptr->image);
        cv::waitKey(20);
    }
};

int main(int argc, char** argv)
{
  cout << "test" << endl;
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
