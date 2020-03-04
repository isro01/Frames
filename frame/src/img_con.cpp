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
    cv::Mat src,hsv;
    cv::GaussianBlur( original, src, cv::Size(3,3),0,0);
    cv::cvtColor(src,hsv,CV_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(160,0,0), cv::Scalar(180, 255, 255), src);

    cv::dilate(src,src, cv::Mat(), cv::Point(-1,-1), 2, 1,1);
    cv::erode(src,src, cv::Mat(), cv::Point(-1,-1), 2, 1, 1);

    cv::Canny(src, src, lowThreshold, upperThreshold, kernel_size);

    cv::Mat mc = cv::Mat::zeros(src.size(), CV_8UC3);

    std::vector<std::vector<cv::Point>> better_contours;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point> approx;

    //cv::dilate(src,src, cv::Mat(), cv::Point(-1,-1), 2, 1,1);
    //cv::erode(src,src, cv::Mat(), cv::Point(-1,-1), 2, 1, 1);

    cv::findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    double min_area=1000000;

    for(int i=0;i<contours.size();i+=1)
    {
        if(cv::contourArea(contours[i])>250 && cv::contourArea(contours[i])<min_area)
        {
            if(better_contours.empty())
            {
                better_contours.push_back(contours[i]);
                min_area = cv::contourArea(contours[i]);
            }
            else
            {
                better_contours.clear();
                better_contours.push_back(contours[i]);
                min_area = cv::contourArea(contours[i]);
            }   
        }
    }

    for(int i=0; i<better_contours.size(); i+=1)
    {
        x=0;
        y=0;
        float tempx=0;
        float tempy=0;

        cv::approxPolyDP(better_contours[i],approx, cv::arcLength(contours[i], true), true);
        int size = approx.size();
            for(int j=0;j<size;j+=1)
            {
                cv::circle(mc, approx[j], 3, cv::Scalar(0,255,0),-1);
                cv::putText(mc, std::to_string(j), cv::Point(approx[j].x, approx[j].y), CV_FONT_HERSHEY_PLAIN, 3, cv::Scalar(0,0,255));

            }

            
            int length=0;

            
            if(approx.size()==4)
            {
              x=0;
              y=0;
              for(int k=0;k<size;k+=1)
              {
                x =x+ approx[k].x;
                y= y+approx[k].y;
              }
              x=x/size;
              y=y/size;
              
              length =abs(approx[0].x -approx[1].x);
              cv::line(mc, approx[0], approx[1], cv::Scalar(255,0,0), 2);
              int lx,ly;
              lx= (approx[0].x + approx[1].x)/2;
              ly= (approx[0].y + approx[1].y)/2 + 20;
              cv::putText(mc, std::to_string(length), cv::Point(lx,ly), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(200,200,200));
              cv::circle(mc, cv::Point(400, 400), 7, cv::Scalar(0,255,255),2);
              cv::putText(mc, std::to_string(size), cv::Point(100,200), CV_FONT_HERSHEY_PLAIN, 1 ,cv::Scalar(255,255,255));

              cv::circle(mc, cv::Point(x,y), 6, cv::Scalar(0,0,255), -1);
              cv::putText(mc, std::to_string(x)+ "," + std::to_string(y), cv::Point(x,y), CV_FONT_HERSHEY_PLAIN, 3, cv::Scalar(0,255,0));

              d = (W*F)/length;
              //cv::putText(mc, std::to_string(d), cv::Point(100,100), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
               cv::putText(mc, std::to_string(d), cv::Point(100,100), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
            }

            
    }
  


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, src);
    cv::imshow(OPENCV_WINDOW_MOD, mc);
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