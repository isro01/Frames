#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <frame/dc.h>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW_MOD = "Processed";
float F = (145*4.13)/1.33;
float W = 1.33;
float d=10;
std::string topic;int rate;
const int max_lowThreshold =100;
int lowThreshold = 255/3;
int upperThreshold = 255;
const int ratio =3;
const int kernel_size =3;
float x=0;
float y=0;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/iris/camera_red_iris/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(OPENCV_WINDOW_MOD);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(OPENCV_WINDOW_MOD);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat original = cv_ptr->image;
    cv::Mat img;
    std::vector<std::vector<cv::Point>> contours, good_contours_;
    std::vector<cv::Vec4i> hierarchy;

    //Thresholding
    cv::GaussianBlur( img, img, cv::Size(3,3), 0, 0 );
    cv::cvtColor( img, img, CV_BGR2HSV);
    cv::inRange( img, cv::Scalar(100,110,110), cv::Scalar(179,255,255),img);
    //cv::inRange(img, cv::Scalar(0,0,0), cv::Scalar(30,30,30), img);
    cv::dilate( img, img, cv::Mat(), cv::Point(-1,-1), 2, 1, 1);
    cv::erode( img ,img, cv::Mat(), cv::Point(-1,-1), 2, 1, 1);


    //find Good contours
    cv::Canny(img , img, 200, 300 , 3);
    cv::findContours(img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    
    std::vector<std::vector<cv::Point>>::iterator ptr = contours.begin();
    std::vector<std::vector<cv::Point>>::iterator end = contours.end();
    std::cout<<contours.size()<<std::endl;

    for(ptr;ptr!=end;++ptr)
    {
        if(cv::contourArea(*ptr)>250){
            good_contours_.push_back(*ptr);
        }
    } 
    cv::Mat test = cv::Mat::zeros(img.size(), CV_8UC3);
    int size = good_contours_.size();
    int i=0;
    for(i;i<size;i+=1){
        cv::drawContours(test,good_contours_, i,cv::Scalar(0,255,255));
    }
    cv::putText(test, std::to_string(i), cv::Point(100,100),CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(255,255,255),1);


    //Frame Frame Centre

    int size, length, x, y, k;
    std::vector<cv::Point> approx;
    std::cout<<good_contours_.size()<<std::endl;
    for( i=0; i<good_contours_.size(); i+=1)
    {
        cv::approxPolyDP(good_contours_[i],approx, 0.02*cv::arcLength(good_contours_[i], true), true);
        size = approx.size();
        std::cout<<size<<std::endl;
        length =0;
        if(1)
        {
            x=0;
            y=0;
            std::cout<<"good"<<i<<std::endl;
            for(k=0;k<size;k+=1)
            {
                std::cout<<"Inside good"<<i<<std::endl;
                x = x + approx[k].x;
                y = y + approx[k].y;
                cv::circle(img,cv::Point(approx[k].x, approx[k].y),4,cv::Scalar(255, 255, 255),-1);
                cv::putText(img,std::to_string(i),approx[k], CV_FONT_HERSHEY_PLAIN,4,cv::Scalar(255,255,255));
            }
            x = x/size;
            y = y/size;
            length = abs(approx[0].x - approx[1].x);
            cv::circle(test,cv::Point(x,y),5,cv::Scalar(0,0,255),-1);
            cv::putText(test)
        }
    }
  


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, img);
    cv::imshow(OPENCV_WINDOW_MOD, test);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main(int argc, char** argv)
{
  ros::init(argc,argv, "image_converter");
  ImageConverter ic;
  ros::NodeHandle nh;
  nh.getParam("topic",topic);
  nh.getParam("rate",rate);
  ros::Publisher pub = nh.advertise<frame::dc>(topic,10);
  ros::Rate loopRate(rate);
  while(ros::ok()){
    frame::dc msg;
    msg.x =x;
    msg.y =y; 
    msg.d =d; 

    pub.publish(msg);
    loopRate.sleep();

    ros::spinOnce();
  }
  return 0;
}