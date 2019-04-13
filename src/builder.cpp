#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <state_graph_builder/graph.h>
#include <state_graph_builder/posegraph.h>
#include <state_graph_builder/posestampedgraph.h>
class Builder
{
  public:
  Builder();
  ~Builder();
  private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_private_;
  ros::Publisher pub;
  std_msgs::Bool switch_signal;
  ros::Subscriber switch_sub;

  int n;
  int id;// 1 for ugv 0 for uav
  bool env;

  ros::Subscriber uav_msgs;
  std::vector<ros::Subscriber> uav_subs;
  std::vector<geometry_msgs::Point> uav_list;
  
  std::vector<geometry_msgs::Pose> ugv_list1;
  std::vector<geometry_msgs::Point> ugv_list2;
  std::vector<geometry_msgs::PoseStamped> ugvout_list;
  
  std::vector<ros::Subscriber> ugv_sub1;
  std::vector<ros::Subscriber> ugv_sub2;
  std::vector<ros::Subscriber> ugv_subout;

  void ugv1_subCallback(const nav_msgs::Odometry::ConstPtr& msgs, const int list_idx);
  void ugv2_subCallback(const geometry_msgs::Twist::ConstPtr& msgs, const int list_idx);
  void ugvout_subCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs, const int list_idx);
  void uav_subCallback(const geometry_msgs::PointStamped::ConstPtr& msgs, const int list_idx);
  void switch_subCallback(const std_msgs::Bool::ConstPtr& msg);
  state_graph_builder::graph msg;
  state_graph_builder::posegraph posemsg;
  state_graph_builder::posestampedgraph posetmsg;

};
void Builder::ugv1_subCallback(const nav_msgs::Odometry::ConstPtr& msgs, const int list_idx){
  ugv_list1[list_idx]=msgs->pose.pose;
  
}
void Builder::ugvout_subCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs, const int list_idx){
  ugvout_list[list_idx].pose=msgs->pose;
  
}
void Builder::ugv2_subCallback(const geometry_msgs::Twist::ConstPtr& msgs, const int list_idx){
  ROS_INFO("I heard: [%lf]", msgs->linear.x);
  ugv_list2[list_idx].x=msgs->linear.x;
  ugv_list2[list_idx].y=0.0;
  ugv_list2[list_idx].z=msgs->angular.z;
  
}

void Builder::uav_subCallback(const geometry_msgs::PointStamped::ConstPtr& msgs, const int list_idx){
  uav_list[list_idx].x = msgs->point.x;
  uav_list[list_idx].y = msgs->point.y;
  uav_list[list_idx].z = msgs->point.z;  
}
void Builder::switch_subCallback(const std_msgs::Bool::ConstPtr& msg){
 switch_signal.data = msg->data;
}

Builder::Builder()
  :nh_private_("~")
{
  
     nh_private_.param<int>("n", n, 15);
     nh_private_.param<bool>("env",env,0);
     uav_list.resize(n);
     ugv_list2.resize(n);
     ugv_list1.resize(n);
     ugvout_list.resize(n);
     int id = 1;

     if (env==0)
       pub=nh.advertise<state_graph_builder::posegraph>("graph", 50);
     else
       pub=nh.advertise<state_graph_builder::posestampedgraph>("graph",50);

     ROS_INFO("Switch is True. Ready to record data %d", n);
     for (int i=1; i < n+1; i++){
       //std::string sub_topic = "/uav" + std::to_string(i) + "/ground_truth/position";
       //uav_subs.push_back( nh.subscribe<geometry_msgs::PointStamped>(sub_topic, 10, boost::bind(&Builder::uav_subCallback,this,_1, i)) );

       
       //std::string sub_topic2 = "/ugv" + std::to_string(i) + "/cmd_vel_mux/input/teleop";
      
       //ugv_sub2.push_back( nh.subscribe<geometry_msgs::Twist>(sub_topic2, 10, boost::bind(&Builder::ugv2_subCallback,this,_1, i-1)) );
       if (env==0){
	 std::string sub_topic3 = "/ugv" + std::to_string(i) + "/odom";
	 ugv_sub1.push_back( nh.subscribe<nav_msgs::Odometry>(sub_topic3, 10, boost::bind(&Builder::ugv1_subCallback,this,_1, i-1)) );
       }
       else{
	 std::string sub_topic3 = "/R" + std::to_string(i);
	 ugv_subout.push_back( nh.subscribe<geometry_msgs::PoseStamped>(sub_topic3, 10, boost::bind(&Builder::ugvout_subCallback,this,_1, i-1)) );
       }
     }
     ROS_INFO("All publisher initialized");

     ros::Rate rate(100);
     uint time_count=0;

       while(ros::ok()) {

	 if (env==0){
       	 posemsg.poses.clear();
       	 posemsg.id.clear();
       	 posemsg.time=time_count;
       	 for (int i=0; i<n; i++){
       	   //msg.points.push_back(uav_list[i]);
       	   //msg.points.push_back(ugv_list2[i]);
	   posemsg.poses.push_back(ugv_list1[i]);
       	   posemsg.id.push_back(id);
       	 }

       	 pub.publish(posemsg);
	 
          //Delays untill it is time to send another message
       	  time_count+=1;
          rate.sleep();
	  ros::spinOnce();
	 }
	 else{
	   posetmsg.poses.clear();
	   posetmsg.id.clear();
	   posetmsg.time=time_count;
	   for (int i=0; i<n; i++){
	     //msg.points.push_back(uav_list[i]);
	     //msg.points.push_back(ugv_list2[i]);
	     posetmsg.poses.push_back(ugvout_list[i]);
	     posetmsg.id.push_back(id);
	   }
	   pub.publish(posetmsg);
	 
          //Delays untill it is time to send another message
       	  time_count+=1;
          rate.sleep();
	  ros::spinOnce();
	 }
       }
    
}

Builder::~Builder()
{
  ros::shutdown();
}

int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "builder");

     Builder builder;
    
     ros::spin();
          

     return 0;
     
}
