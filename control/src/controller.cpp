#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>

mavros_msgs::State state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    state = *msg;
}

//void local_position(){}
int main(int argc, char** argv){
    ros::init (argc,argv,"controller");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",10,state_cb);
    //ros::Subscriber local_pos = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,local_position);
    ros::ServiceClient arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    ros::Rate rate(20.0);

    while(ros::ok && !state.connected){

        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x=0;
    pose.pose.position.y=0;
    pose.pose.position.z=2;
    
    int c=100;
    for (c=100;ros::ok() && c>0;c--){
        pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offboard;
    offboard.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm;
    arm.request.value =true; 
    
    float a=0.2;
    int i=1;
    int j=0;
    ros::Time last = ros::Time::now();
    while (ros::ok()){
        if (state.mode != "OFFBOARD" && (ros::Time::now()-last > ros::Duration(5.0))){
            if(mode.call(offboard) && offboard.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last= ros::Time::now();
        }
        else{
            if (!state.armed && (ros::Time::now()-last > ros::Duration(5.0))){
                if (arming.call(arm) && arm.response.success){
                    ROS_INFO("ARMED");
                }
                last=ros::Time::now();
            }

        }
    // if ( pose.pose.position.z>=1.8 && i<10){
    //     pose.pose.position.x=a*i;
        
    //     i++;
    // }
    
    pub.publish(pose);
    j++;
    
    ros::spinOnce();
    rate.sleep();
    }

    
    return 0;
 


}
