#include <ros/ros.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <GeographicLib/AzimuthalEquidistant.hpp>
#include <nav_msgs/Odometry.h>
#include <frame/dc.h>
#include <geometry_msgs/PoseStamped.h>

float x,y,d;
bool height=false;
char dir;
float final_y;
int flag=0;
float glob_x=0,glob_y=0;
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped l_pose;
std::string topic; int rate;
nav_msgs::Odometry mav_pose_;
void sub_cb(frame::dc msg){
  x=msg.x;
  y=msg.y;
  d=msg.d;
 
  return;
}

void nav_cb(const nav_msgs::Odometry &msg)
{
  mav_pose_ = msg;
}

void mav_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
         l_pose = *msg;
        if(l_pose.pose.position.z>=1)
            height = true;
        else height = false;
       // std::cout<<height<<std::endl;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "transform");
    ros::NodeHandle nh;
    nh.getParam("topic",topic);
    nh.getParam("rate",rate);
    ros::Subscriber sub= nh.subscribe(topic,10,sub_cb);
    ros::Subscriber nav_sub = nh.subscribe("odom",10,nav_cb);
    ros::Subscriber mav_sub = nh.subscribe("mavros/local_position/pose", 10, mav_cb);
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);


    
    
    Eigen::Matrix3d scaleUp, quadToGlob, CamMatrix, invCamMatrix, CamtoQuad;
   

    CamMatrix(0,0) = 575.8357775641182;
    CamMatrix(0,1) = 0;
    CamMatrix(0,2) = 304.5915536401063;
    CamMatrix(1,0) = 0;
    CamMatrix(1,1) = 579.0890363516307;
    CamMatrix(1,2) = 236.6503982296557;
    CamMatrix(2,0) = 0;
    CamMatrix(2,1) = 0;
    CamMatrix(2,2) = 1;
    
    CamtoQuad(0,0) = 0;
    CamtoQuad(0,1) = 0;
    CamtoQuad(0,2) = 1;
    CamtoQuad(1,0) = 0;
    CamtoQuad(1,1) =1 ;
    CamtoQuad(1,2) = 0;
    CamtoQuad(2,0) = 1;
    CamtoQuad(2,1) = 0;
    CamtoQuad(2,2) = 0;

    

    invCamMatrix = CamMatrix.inverse();
    ros::Rate loopRate(rate);
    while(ros::ok()){
        tf::Quaternion q1(mav_pose_.pose.pose.orientation.x, mav_pose_.pose.pose.orientation.y,
                           mav_pose_.pose.pose.orientation.z, mav_pose_.pose.pose.orientation.w);
        Eigen::Quaterniond quat = Eigen::Quaterniond(q1.w(), q1.x(), q1.y(), q1.z());
        quadToGlob = quat.normalized().toRotationMatrix();
     
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (i == j)
                    scaleUp(i, j) = d;
                else
                    scaleUp(i, j) = 0;
            }
        }


        Eigen::Vector3d imgVec(x, y, 1);
        Eigen::Vector3d quadCoord = (CamtoQuad * scaleUp * invCamMatrix * imgVec) /*+ tCam*/;
        Eigen::Vector3d globCoord = quadToGlob * quadCoord;

        if(x>340 && x!=0){
            dir='L';
        }
        else if (x<320 && x!=0){
            dir='R';
        }

        if (x <=340 && x>=320 ){
            flag=1;
        }

        if(!height)
        {
            pose.pose.position.x=-1;
            pose.pose.position.y= 1;
            pose.pose.position.z=1.2;
        }
        //  else
        //  {
        //      if (globCoord.x()>=3 && globCoord.x() <=7 /*&& globCoord.y()<=0.75 && globCoord.y()>=-1*/)
        //      {
        //          pose.pose.position.x=globCoord.x() ;
        //         pose.pose.position.y=globCoord.y();
        //          pose.pose.position.z=1.9;
        //          glob_x = globCoord.x();
        //          glob_y = globCoord.y();
        //     }
        //      else
        //      {
        //          pose.pose.position.x=glob_x ;
        //          pose.pose.position.y=glob_y;
        //          pose.pose.position.z=1.9;
        //      }
        //  }

        else
        {
            if (flag==0 && dir=='L'){
                pose.pose.position.x = l_pose.pose.position.x;
                pose.pose.position.y = l_pose.pose.position.y-1;
                pose.pose.position.z = 1.2;
                final_y = l_pose.pose.position.y-1;
            }
            else if (flag==0 && dir=='R'){
                pose.pose.position.x = l_pose.pose.position.x;
                pose.pose.position.y = l_pose.pose.position.y+1;
                pose.pose.position.z = 1.2;
                final_y = l_pose.pose.position.y+1;
            }
            else if (flag==1){
                
                if (l_pose.pose.position.x >= glob_x){
                    flag=0;
                    dir='N';
                }
                if(globCoord.x()<15)
                {
                    pose.pose.position.x = globCoord.x();
                    pose.pose.position.y = globCoord.y();
                    pose.pose.position.z=1.2;
                    glob_x = globCoord.x();
                    glob_y = globCoord.y();
                    
                }
                else{
                    pose.pose.position.x=glob_x + 2 ;
                    pose.pose.position.y=glob_y;
                    pose.pose.position.z=1.2;
                }

                // if(globCoord.x()<=10)
                // {
                //     pose.pose.position.x = globCoord.x();
                //     pose.pose.position.y = globCoord.y();
                // }

            }
        }
        
        
        //  std::cout << height<<"Pos"<< pose <<std::endl;
         std::cout <<  "flag  "<< flag <<std::endl;
         std::cout << "glob_x: " << glob_x << "  glob_y: "<< glob_y << std::endl;
         std::cout << "x: "<< x <<std::endl <<std::endl;
        //  std::cout << "y " << l_pose.pose.position.y <<std::endl;
        //  std::cout << "global x " << globCoord.x() << "  global y " << globCoord.y() <<std::endl;
        std::cout << "local_pose.x "<< l_pose.pose.position.x << "  local_pose.y: "<<l_pose.pose.position.y<<std::endl;
        pub.publish(pose);
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
